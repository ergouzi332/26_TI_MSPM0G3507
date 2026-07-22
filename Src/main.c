#include "OLED.h"
#include "MOTOR.h"
#include "MPU6050.h"
#include "KEY.h"
#include "GRAYSCALE.h"
#include "uart3.h"
#include "ti_msp_dl_config.h"

// 系统滴答定时器毫秒计数，中断自增
volatile uint32_t tick_ms = 0;
void SysTick_Handler(void) { tick_ms++; }

// 编码器速度环参数定义
#define TARGET_PULSE    15      // 预留脉冲目标参数（当前未启用）
#define PWM_FF          100.0f  // 速度环前馈PWM基础值
#define KP              5.0f    // 速度环比例系数
#define KI              0.0f    // 速度环积分系数（当前关闭积分）

// 循迹横向PD控制器参数
#define KP_LINE         2.0f    // 循迹偏差比例系数
#define KD_LINE         0.3f    // 循迹微分项系数

// 左右轮目标脉冲上下限限制
#define TARGET_MIN      4       // 最低脉冲计数
#define TARGET_MAX      30      // 最高脉冲计数

int main(void)
{
    // TI MSPM0芯片底层系统初始化（SYSCFG图形配置工具生成）
    SYSCFG_DL_init();
    // 外设初始化：屏幕、陀螺仪、电机、按键、灰度传感器
    OLED_Init(); MPU6050_Init(); Motor_Init(); KEY_Init(); Grayscale_Init();
    // 陀螺仪自动校准零偏、重置航向角Yaw
    MPU6050_CalibrateGyro(); MPU6050_ResetYaw();
    // 电机初始PWM置0，初始化串口3并发送测试信息
    Motor_SetPWM(0, 0); UART3_Init(); UART3_Test();

    // 配置SysTick定时器，32MHz系统时钟，1ms中断一次
    SysTick->LOAD = 32000UL - 1; SysTick->VAL = 0; SysTick->CTRL = 0x07;
    // OLED清屏，打印初始化完成提示
    OLED_Clear(); OLED_WriteString(0, 0, "INIT DONE");

    float integralL = 0.0f, integralR = 0.0f;   // 左右轮速度环积分累加值
    int16_t pwmL = 0, pwmR = 0;                 // 左右轮输出PWM值
    float er_filt = 0.0f, last_er_filt = 0.0f;  // 滤波后循迹偏差、上一帧偏差（微分项用）
    uint8_t motor_run = 0;                      // 电机运行标志：0停止 1运行

    uint16_t d_pwmL = 0, d_pwmR = 0;    // 缓存左右轮PWM，用于串口打印
    int16_t d_pL = 0, d_pR = 0;         // 缓存左右轮编码器脉冲数
    int16_t d_er = 0;                   // 原始循迹误差值
    int16_t d_yaw = 0;                  // 缩放后的航向角Yaw（放大10倍）
    int16_t d_gz = 0;                   // 陀螺仪Z轴原始角速度
    uint16_t d_gray = 0;                // 8路灰度传感器原始采集值
    uint32_t last_20ms = 0, last_500ms = 0; // 20ms控制周期、500ms串口打印计时戳

    // 陀螺仪静止零偏累计校准变量
    float gz_bias_sum = 0;  // 多帧角速度求和
    int gz_bias_cnt = 0;    // 累计采样次数

    // 8路灰度权重数组：CH0最左(-7) ~ CH7最右(+7)；黑线=0参与计算，白线=1不计入
    int8_t w[8] = {-7,-5,-3,-1,1,3,5,7};
    uint8_t sys_st = 0;
    int16_t yaw_start = 0;       // 转弯起始yaw，用于判断转够90度
    uint8_t cs_ph = 0;
    uint8_t cs_tm = 0;

    while (1) // 主循环死循环
    {
        uint32_t now = tick_ms; // 获取当前系统毫秒时间戳

        // ========== 20ms定时控制周期：循迹+速度闭环+陀螺仪校准 ==========
        if (now - last_20ms >= 20) {
            last_20ms = now; // 更新20ms计时基准

            // 1. 读取陀螺仪Z轴角速度，积分更新航向角Yaw，积分步长0.02s
            d_gz = MPU6050_ReadGZ();
            MPU6050_UpdateYawFromRaw(d_gz, 0.02f);
            d_yaw = (int16_t)(MPU6050_GetYaw() * 10.0f); // Yaw放大10倍方便串口显示

            // 2. 静止时自动校准陀螺仪零偏，电机运转时跳过校准
            if (!motor_run)
            {
                // 角速度波动很小判定车身静止
                if (d_gz > -500 && d_gz < 500)
                {
                    gz_bias_sum += d_gz;
                    gz_bias_cnt++;
                    // 累计50帧后计算平均零偏并写入陀螺仪偏移寄存器
                    if (gz_bias_cnt >= 50)
                    {
                        float new_offset = gz_bias_sum / gz_bias_cnt;
                        MPU6050_SetGzOffset(new_offset);
                        gz_bias_sum = 0; gz_bias_cnt = 0; // 清空累计缓存
                    }
                }
                else // 车身有晃动，重置校准计数
                {
                    gz_bias_sum = 0; gz_bias_cnt = 0;
                }
            }
            else // 电机运行，禁止零偏自校准
            {
                gz_bias_sum = 0; gz_bias_cnt = 0;
            }

            // 3. 按键扫描处理启停逻辑
            uint8_t key = KEY_Scan();
            // KEY1按下：启动小车，清空积分项，重置编码器脉冲基准
            if (key & KEY_1)
            {
                sys_st = 1; motor_run = 1; Motor_SetForward();
                integralL = 0.0f; integralR = 0.0f;
                Motor_GetLeftPulses(); Motor_GetRightPulses();
            }
            // KEY2按下：刹车停机，PWM清零，复位所有控制变量
            if (key & KEY_2)
            {
                sys_st = 2; cs_ph = 0; cs_tm = 0; motor_run = 1; Motor_SetForward();
                integralL = 0.0f; integralR = 0.0f;
                Motor_GetLeftPulses(); Motor_GetRightPulses();
            }
            if (key & KEY_3)
            {
                sys_st = 0; motor_run = 0; Motor_Stop();
            }
            // KEY2: square tracking (TODO)

            // 4. 读取8路灰度传感器，计算循迹误差
            uint16_t line = Grayscale_ReadAll() & 0xFF;
            d_gray = line;
            int sum = 0;
            // 遍历8路传感器，检测到黑线(对应bit=0)则叠加对应权重
            for (int i = 0; i < 8; i++)
            {
                if ((line & (1 << i)) == 0) sum += w[i];
            }

            d_er = (int16_t)sum; // 原始总偏差，左偏负数、右偏正数

            // 5. 一阶低通滤波平滑偏差 + 死区消除微小抖动
            er_filt += ((float)d_er - er_filt) * 0.7f;
            if (fabsf(er_filt) < 2.0f) er_filt = 0.0f;

            // 6. 循迹PD控制器计算转向修正量
            float diff_er = er_filt - last_er_filt; // 偏差微分项（当前-上一帧）
            last_er_filt = er_filt;
            float steer = er_filt * KP_LINE + diff_er * KD_LINE;

            // 7. 弯道减速逻辑：直线基础速度20，偏差越大速度越低，最低限速8
            int32_t base_speed = 20 - (int32_t)(fabsf(er_filt) * 0.5f);
            if (sys_st == 2 && base_speed > 12) base_speed = 12;  // square mode: slower
            if (base_speed < 8) base_speed = 8;

            // 8. 差速转向分配左右轮目标脉冲
            int32_t targetL = base_speed - (int32_t)(steer + 0.5f);
            int32_t targetR = base_speed + (int32_t)(steer + 0.5f);
            // 限制左右轮脉冲输出区间 [TARGET_MIN, TARGET_MAX]
            if (targetL < TARGET_MIN) targetL = TARGET_MIN;
            if (targetL > TARGET_MAX) targetL = TARGET_MAX;
            if (targetR < TARGET_MIN) targetR = TARGET_MIN;
            if (targetR > TARGET_MAX) targetR = TARGET_MAX;

            // 9. 电机运行时执行速度环PI闭环控制
            switch (sys_st)
            {
                case 0:
                 
                d_pL = 0; d_pR = 0;
          
                    break;
                case 1:
                {

                // 左轮速度PI
                int32_t pL = (int32_t)Motor_GetLeftPulses();
                d_pL = (int16_t)pL;
                float errL = (float)(targetL - pL); // 速度误差=目标脉冲-当前脉冲
                integralL += errL;                  // 积分累加
                // 积分限幅防积分饱和
                if (integralL > 500.0f) integralL = 500.0f;
                if (integralL < -500.0f) integralL = -500.0f;
                // PI输出 = 前馈 + P调节 + I调节
                float outL = PWM_FF + KP * errL + KI * integralL;
                // PWM输出限幅 0~300
                if (outL < 0.0f) outL = 0.0f;
                if (outL > 300.0f) outL = 300.0f;

                // 右轮速度PI，逻辑同左轮
                int32_t pR = (int32_t)Motor_GetRightPulses();
                d_pR = (int16_t)pR;
                float errR = (float)(targetR - pR);
                integralR += errR;
                if (integralR > 500.0f) integralR = 500.0f;
                if (integralR < -500.0f) integralR = -500.0f;
                float outR = PWM_FF + KP * errR + KI * integralR;
                if (outR < 0.0f) outR = 0.0f;
                if (outR > 300.0f) outR = 300.0f;

                // 浮点转整形输出电机PWM
                pwmL = (int16_t)(outL + 0.5f);
                pwmR = (int16_t)(outR + 0.5f);
                Motor_SetPWM((uint16_t)pwmL, (uint16_t)pwmR);
           
                    break;
                }
                case 2:
                {
                    if (cs_ph == 0)
                    {
                        // ===== Normal tracking with PI speed control =====
                        int32_t pL = (int32_t)Motor_GetLeftPulses();
                        d_pL = (int16_t)pL;
                        float errL = (float)(targetL - pL);
                        integralL += errL;
                        if (integralL > 500.0f) integralL = 500.0f;
                        if (integralL < -500.0f) integralL = -500.0f;
                        float outL = PWM_FF + KP * errL + KI * integralL;
                        if (outL < 0.0f) outL = 0.0f;
                        if (outL > 300.0f) outL = 300.0f;

                        int32_t pR = (int32_t)Motor_GetRightPulses();
                        d_pR = (int16_t)pR;
                        float errR = (float)(targetR - pR);
                        integralR += errR;
                        if (integralR > 500.0f) integralR = 500.0f;
                        if (integralR < -500.0f) integralR = -500.0f;
                        float outR = PWM_FF + KP * errR + KI * integralR;
                        if (outR < 0.0f) outR = 0.0f;
                        if (outR > 300.0f) outR = 300.0f;

                        pwmL = (int16_t)(outL + 0.5f);
                        pwmR = (int16_t)(outR + 0.5f);
                        Motor_SetPWM((uint16_t)pwmL, (uint16_t)pwmR);
                        
                        // Cross detection: all black + centered
                        if (d_gray == 0xFF && (d_er < 0 ? -d_er : d_er) < 3)
                        {
                            cs_ph = 1; cs_tm = 0; yaw_start = d_yaw; Motor_Stop(); pwmL = 0; pwmR = 0; d_pL = 0; d_pR = 0;
                        }
                    }
                    else  // cs_ph != 0 -> turning phase
                    {
                        cs_tm++;  // increment every 20ms
                        
                        if (cs_tm < 3)
                        {
                            // Phase A: brake (60ms)
                            Motor_Stop();
                            pwmL = 0; pwmR = 0;
                            d_pL = 0; d_pR = 0;
                        }
                        else
                        {
                            // Phase B: tank left turn with yaw-based exit
                            Motor_TankLeft();
                            Motor_SetPWM(150, 150);
                            pwmL = 150; pwmR = 150;
                            d_pL = 0; d_pR = 0;
                            
                            // Yaw-based exit: detect ~85deg turn, with wrap-around handling
                            int16_t yaw_delta = d_yaw - yaw_start;
                            // Fix wrap-around at +/-180deg: left turn should give negative delta
                            if (yaw_delta > 1800) yaw_delta -= 3600;  // wrapped -180 -> +180
                            if (yaw_delta < -1800) yaw_delta += 3600;  // wrapped +180 -> -180
                            if ((cs_tm >= 10 && yaw_delta <= -800) || cs_tm >= 60)
                            {
                                cs_ph = 0; cs_tm = 0;
                                integralL = 0.0f; integralR = 0.0f;
                                Motor_GetLeftPulses(); Motor_GetRightPulses();
                                Motor_SetForward();
                            }
                        }  // closes else (Phase B)
                    }  // closes outer else (cs_ph != 0)
                    break;
                }  // closes case 2


































































































                default:

                    

                d_pL = 0; d_pR = 0;

            

                    break;

            }

            d_pwmL = (uint16_t)pwmL; d_pwmR = (uint16_t)pwmR;

        }



        // ========== 500ms定时串口打印，上传所有调试数据 ==========

        if (now - last_500ms >= 500)

        {

            last_500ms = now;

            char b[80]; int p = 0; // 串口发送缓冲区、字符指针

            // 拼接字符串：PWM值、左右脉冲、循迹误差、灰度传感器、航向角、陀螺仪Z轴

            b[p++]='P';b[p++]='W';b[p++]='M';b[p++]=':';

            b[p++]='0'+(d_pwmL/100)%10;b[p++]='0'+(d_pwmL/10)%10;b[p++]='0'+(d_pwmL%10);

            b[p++]=' ';b[p++]='L';b[p++]=':'; int16_t v=d_pL;

            if(v<0){b[p++]='-';v=-v;}else b[p++]='+';

            b[p++]='0'+(v/10)%10;b[p++]='0'+(v%10);

            b[p++]=' ';b[p++]='R';b[p++]=':'; v=d_pR;

            if(v<0){b[p++]='-';v=-v;}else b[p++]='+';

            b[p++]='0'+(v/10)%10;b[p++]='0'+(v%10);

            b[p++]=' ';b[p++]='E';b[p++]='R';b[p++]=':'; v=d_er;

            if(v<0){b[p++]='-';v=-v;}else b[p++]='+';

            b[p++]='0'+(v/10)%10;b[p++]='0'+(v%10);

            b[p++]=' ';b[p++]='G';b[p++]='R';b[p++]=':';

            // 从高位到低位打印8路灰度0/1状态

            for(int i=7;i>=0;i--) b[p++]=(d_gray&(1<<i))?'0':'1';

            b[p++]=' ';b[p++]='Y';b[p++]=':'; v=d_yaw;

            if(v<0){b[p++]='-';v=-v;}else b[p++]='+';

            b[p++]='0'+(v/100)%10;b[p++]='0'+(v/10)%10;b[p++]='0'+(v%10);

            b[p++]=' ';b[p++]='G';b[p++]='Z';b[p++]=':'; v=d_gz;

            if(v<0){b[p++]='-';v=-v;}else b[p++]='+';

            b[p++]='0'+(v/10000)%10;b[p++]='0'+(v/1000)%10;

            b[p++]='0'+(v/100)%10;b[p++]='0'+(v/10)%10;b[p++]='0'+(v%10);

            b[p++]='\r';b[p++]='\n';

            // 逐字节发送串口数据

            for(int i=0;i<p;i++) UART3_SendByte((uint8_t)b[i]);

        }

    }

}


