#include "OLED.h"
#include "MOTOR.h"
#include "MPU6050.h"
#include "KEY.h"
#include "GRAYSCALE.h"
#include "uart3.h"
#include "ti_msp_dl_config.h"

// 系统滴答定时器全局计时变量，每1ms自增1
volatile uint32_t tick_ms = 0;
// SysTick中断服务函数，1ms进一次中断，毫秒计时累加
void SysTick_Handler(void) { tick_ms++; }

// 速度环PID宏定义
#define TARGET_PULSE    15      // 20ms控制周期内，电机编码器目标基础脉冲数（基础行驶速度）
#define PWM_FF          100.0f // 速度环前馈PWM值，小车基础行驶占空比
#define KP              5.0f    // 电机速度环比例系数
#define KI              0.0f    // 电机速度环积分系数（当前关闭积分）

// 循迹横向PD控制器参数
#define KP_LINE         2.0f    // 循迹误差比例系数
#define KD_LINE         0.3f    // 循迹误差微分系数

// 左右轮目标脉冲限幅，限制最低、最高行驶速度
#define TARGET_MIN      4
#define TARGET_MAX      30

int main(void)
{
    // MSPM0芯片底层外设、时钟初始化（SYSCFG图形配置工具自动生成）
    SYSCFG_DL_init();
    // 外设初始化：OLED屏幕、MPU6050陀螺仪、电机驱动、按键、8路灰度传感器
    OLED_Init(); MPU6050_Init(); Motor_Init(); KEY_Init(); Grayscale_Init();
    // 陀螺仪静止零偏校准、航向角Yaw初始归零
    MPU6050_CalibrateGyro(); MPU6050_ResetYaw();
    // 电机初始刹车停止，串口3初始化并发送自检测试帧
    Motor_SetPWM(0, 0); UART3_Init(); UART3_Test();

    // 配置SysTick定时器，32MHz系统时钟，产生1ms定时中断
    SysTick->LOAD = 32000UL - 1; SysTick->VAL = 0; SysTick->CTRL = 0x07;
    OLED_Clear(); OLED_WriteString(0, 0, "INIT DONE"); // OLED屏幕打印初始化完成提示

    float integralL = 0.0f, integralR = 0.0f;   // 左右电机速度环积分累加缓存
    int16_t pwmL = 0, pwmR = 0;                 // 左右轮最终输出PWM数值
    float er_filt = 0.0f, last_er_filt = 0.0f;  // 滤波后循迹误差、上一帧误差（用于微分项计算）
    uint8_t motor_run = 0;                     // 小车运行标志：0=停止，1=启动循迹

    // 串口打印临时缓存变量，保存实时采集数据
    uint16_t d_pwmL = 0, d_pwmR = 0;    // 当前左右轮PWM缓存
    int16_t d_pL = 0, d_pR = 0;         // 左右轮编码器脉冲计数缓存
    int16_t d_er = 0, d_yaw = 0, d_gz = 0; // 循迹偏差、航向角、陀螺仪Z轴原始角速度缓存
    uint16_t d_gray = 0;                // 8路灰度传感器原始采集二进制值
    uint32_t last_20ms = 0, last_500ms = 0; // 20ms控制周期计时、500ms串口打印计时标记

    // 陀螺仪静止自动校准零偏累加变量
    float gz_bias_sum = 0;
    int gz_bias_cnt = 0;

    // 8路灰度传感器权重数组：通道0最左侧，通道7最右侧；检测到黑线时bit为0
    // 左侧黑线赋负权重、右侧黑线赋正权重，加权计算轨道中心偏差
    int8_t w[8] = {-7,-5,-3,-1,1,3,5,7};

    while (1)
    {
        uint32_t now = tick_ms; // 获取当前系统毫秒时间戳

        // ========== 20ms固定控制周期：整车闭环控制核心逻辑 ==========
        if (now - last_20ms >= 20) {
            last_20ms = now;

            // 读取MPU6050 Z轴原始角速度数据
            d_gz = MPU6050_ReadGZ();
            // 对角速度积分更新航向角Yaw，积分步长20ms即0.02s
            MPU6050_UpdateYawFromRaw(d_gz, 0.02f);
            d_yaw = (int16_t)(MPU6050_GetYaw() * 10.0f); // Yaw放大10倍，方便串口直观查看小数

            // 小车静止状态下，自动采集陀螺仪零偏补偿值，消除静止漂移
            if (!motor_run) {
                // Z轴角速度接近0，判定车辆完全静止
                if (d_gz > -500 && d_gz < 500) {
                    gz_bias_sum += d_gz;
                    gz_bias_cnt++;
                    // 累计采集50组静止样本后，计算平均偏移写入陀螺仪补偿
                    if (gz_bias_cnt >= 50) {
                        float new_offset = gz_bias_sum / gz_bias_cnt;
                        MPU6050_SetGzOffset(new_offset);
                        gz_bias_sum = 0; gz_bias_cnt = 0;
                    }
                } else {
                    // 车辆晃动/有角速度，清空校准计数，重新累积采样
                    gz_bias_sum = 0; gz_bias_cnt = 0;
                }
            } else {
                // 小车行驶中禁止校准零偏，清空校准缓存
                gz_bias_sum = 0; gz_bias_cnt = 0;
            }

            // 按键扫描检测
            uint8_t key = KEY_Scan();
            // KEY1按键：启动小车循迹
            if (key & KEY_1) {
                if (!motor_run) {
                    Motor_SetForward();    // 设置电机正转行驶方向
                    motor_run = 1;         // 置位运行标志
                    integralL = 0.0f; integralR = 0.0f; // 清空速度积分，防止启动瞬间积分饱和炸车
                    Motor_GetLeftPulses(); Motor_GetRightPulses(); // 重置编码器计数起点
                }
            }
            // KEY2按键：小车刹车停止
            if (key & KEY_2) {
                Motor_SetBrake(); Motor_SetPWM(0, 0); // 电机机械刹车、PWM输出置0
                motor_run = 0; pwmL = 0; pwmR = 0;
                integralL = 0.0f; integralR = 0.0f;
                Motor_GetLeftPulses(); Motor_GetRightPulses();
            }

            // 读取8路灰度传感器数据，bit0左，bit7右；0代表黑线，1代表白纸
            uint16_t line = Grayscale_ReadAll() & 0xFF;
            d_gray = line;
            int sum = 0;
            // 遍历8路传感器，加权求和计算轨道总偏差
            for (int i = 0; i < 8; i++) {
                if ((line & (1 << i)) == 0) sum += w[i];
            }
            d_er = (int16_t)sum;

            // 一阶低通滤波平滑循迹误差，抑制传感器跳变、减少车身抖动
            er_filt += ((float)d_er - er_filt) * 0.7f;
            // 误差死区处理：偏差小于2视为车身居中，消除微小抖动
            if (fabsf(er_filt) < 2.0f) er_filt = 0.0f;

            // 循迹PD控制器计算：比例项 + 微分项（当前误差-上一帧误差）
            float diff_er = er_filt - last_er_filt;
            last_er_filt = er_filt;
            float steer = er_filt * KP_LINE + diff_er * KD_LINE;

            // 根据转向偏差修正左右轮目标脉冲：左轮减速、右轮加速实现向右转向，反之向左
            int32_t targetL = TARGET_PULSE - (int32_t)(steer + 0.5f);
            int32_t targetR = TARGET_PULSE + (int32_t)(steer + 0.5f);
            // 限制单轮目标脉冲上下限，防止单侧轮停转或超速
            if (targetL < TARGET_MIN) targetL = TARGET_MIN;
            if (targetL > TARGET_MAX) targetL = TARGET_MAX;
            if (targetR < TARGET_MIN) targetR = TARGET_MIN;
            if (targetR > TARGET_MAX) targetR = TARGET_MAX;

            // 小车运行时执行左右轮独立速度闭环PI控制
            if (motor_run) {
                // 读取左轮当前编码器脉冲计数
                int32_t pL = (int32_t)Motor_GetLeftPulses();
                d_pL = (int16_t)pL;
                float errL = (float)(targetL - pL); // 左轮速度误差 = 目标脉冲 - 实际脉冲
                integralL += errL;                  // 误差累加进入积分项
                // 积分限幅，避免积分饱和导致过冲
                if (integralL > 500.0f) integralL = 500.0f;
                if (integralL < -500.0f) integralL = -500.0f;
                // 左轮PWM输出公式：前馈基础值 + 比例调节 + 积分调节
                float outL = PWM_FF + KP * errL + KI * integralL;
                // PWM输出硬限幅 0~300
                if (outL < 0.0f) outL = 0.0f;
                if (outL > 300.0f) outL = 300.0f;

                // 右轮速度闭环PI控制，逻辑与左轮完全一致
                int32_t pR = (int32_t)Motor_GetRightPulses();
                d_pR = (int16_t)pR;
                float errR = (float)(targetR - pR);
                integralR += errR;
                if (integralR > 500.0f) integralR = 500.0f;
                if (integralR < -500.0f) integralR = -500.0f;
                float outR = PWM_FF + KP * errR + KI * integralR;
                if (outR < 0.0f) outR = 0.0f;
                if (outR > 300.0f) outR = 300.0f;

                // 浮点PWM值四舍五入转为整型，输出给电机驱动
                pwmL = (int16_t)(outL + 0.5f);
                pwmR = (int16_t)(outR + 0.5f);
                Motor_SetPWM((uint16_t)pwmL, (uint16_t)pwmR);
            } else {
                // 小车停止，清空编码器缓存数据
                d_pL = 0; d_pR = 0;
            }
            // 保存当前PWM数值，用于500ms串口打印
            d_pwmL = (uint16_t)pwmL; d_pwmR = (uint16_t)pwmR;
        }

        // ========== 500ms串口打印周期：输出全部调试参数，上位机看曲线 ==========
        if (now - last_500ms >= 500) {
            last_500ms = now;
            char b[80]; int p = 0; // 串口发送字符缓冲区，p为数组写入索引
            b[p++]='P';b[p++]='W';b[p++]='M';b[p++]=':';
            // 打印左轮PWM，固定3位数字
            b[p++]='0'+(d_pwmL/100)%10;b[p++]='0'+(d_pwmL/10)%10;b[p++]='0'+(d_pwmL%10);
            b[p++]=' ';b[p++]='L';b[p++]=':'; int16_t v=d_pL;
            // 打印左轮编码器脉冲，带正负符号，2位数字
            if(v<0){b[p++]='-';v=-v;}else b[p++]='+';
            b[p++]='0'+(v/10)%10;b[p++]='0'+(v%10);
            b[p++]=' ';b[p++]='R';b[p++]=':'; v=d_pR;
            // 打印右轮编码器脉冲，带正负符号，2位数字
            if(v<0){b[p++]='-';v=-v;}else b[p++]='+';
            b[p++]='0'+(v/10)%10;b[p++]='0'+(v%10);
            b[p++]=' ';b[p++]='E';b[p++]='R';b[p++]=':'; v=d_er;
            // 打印循迹偏差值，带正负符号
            if(v<0){b[p++]='-';v=-v;}else b[p++]='+';
            b[p++]='0'+(v/10)%10;b[p++]='0'+(v%10);
            b[p++]=' ';b[p++]='G';b[p++]='R';b[p++]=':';
            // 从高位到低位输出8路灰度传感器二进制状态 bit7 ~ bit0
            for(int i=7;i>=0;i--) b[p++]=(d_gray&(1<<i))?'0':'1';
            b[p++]=' ';b[p++]='Y';b[p++]=':'; v=d_yaw;
            // 打印航向角Yaw，带正负，固定3位数字
            if(v<0){b[p++]='-';v=-v;}else b[p++]='+';
            b[p++]='0'+(v/100)%10;b[p++]='0'+(v/10)%10;b[p++]='0'+(v%10);
            b[p++]=' ';b[p++]='G';b[p++]='Z';b[p++]=':'; v=d_gz;
            // 打印陀螺仪Z轴原始角速度，带正负，5位数字
            if(v<0){b[p++]='-';v=-v;}else b[p++]='+';
            b[p++]='0'+(v/10000)%10;b[p++]='0'+(v/1000)%10;
            b[p++]='0'+(v/100)%10;b[p++]='0'+(v/10)%10;b[p++]='0'+(v%10);
            b[p++]='\r';b[p++]='\n';
            // 循环逐字节发送整组调试字符串
            for(int i=0;i<p;i++) UART3_SendByte((uint8_t)b[i]);
        }
    }
}