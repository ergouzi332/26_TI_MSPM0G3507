#include "OLED.h"
#include "MOTOR.h"
#include "MPU6050.h"
#include "KEY.h"
#include "GRAYSCALE.h"
#include "ti_msp_dl_config.h"

// 系统1ms时基计数器，SysTick中断自增
volatile uint32_t tick_ms = 0;

/**
 * @brief SysTick中断服务函数，每1ms触发一次，更新全局时基
 */
void SysTick_Handler(void)
{
    tick_ms++;
}

// 编码器参数：车轮单圈总脉冲数
#define PULSE_PER_REV  300
// 循迹差速转向增益系数
#define PWM_LIMIT     500
#define STEER_GAIN    10

/**
 * @brief OLED无符号5位数字打印，完整填充5位字符，消除显存残留
 * @param x 横向坐标
 * @param y 纵向坐标
 * @param val 需要显示的非负整数
 */
static void oled_show_val(uint8_t x, uint8_t y, uint16_t val)
{
    char buf[6];
    buf[0] = (val / 10000) % 10 + '0';
    buf[1] = (val / 1000) % 10 + '0';
    buf[2] = (val / 100) % 10 + '0';
    buf[3] = (val / 10) % 10 + '0';
    buf[4] = val % 10 + '0';
    buf[5] = 0;
    OLED_WriteString(x, y, buf);
}

/**
 * @brief OLED带符号数值打印，首位+/-符号，4位数字全覆盖，无残影
 * @param x 横向坐标
 * @param y 纵向坐标
 * @param val 有符号整型数据
 */
static void oled_show_signed(uint8_t x, uint8_t y, int16_t val)
{
    char buf[7];
    if (val < 0) { buf[0] = '-'; val = -val; }
    else { buf[0] = '+'; }
    buf[1] = (val / 1000) % 10 + '0';
    buf[2] = (val / 100) % 10 + '0';
    buf[3] = (val / 10) % 10 + '0';
    buf[4] = val % 10 + '0';
    buf[5] = 0;
    OLED_WriteString(x, y, buf);
}

int main(void)
{
    // SYSCFG图形配置生成的底层外设初始化
    SYSCFG_DL_init();
    // 外设模块初始化
    OLED_Init();
    MPU6050_Init();
    Motor_Init();
    KEY_Init();
    Grayscale_Init();
    // 陀螺仪校准、重置航向角积分
    MPU6050_CalibrateGyro();
    MPU6050_ResetYaw();

    // 上电电机PWM置0，小车静止
    Motor_SetPWM(0, 0);

    // SysTick配置：32MHz系统时钟，1ms中断
    SysTick->LOAD = 32000UL - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x07;   // 内部时钟、开启中断、使能定时器

    // OLED屏幕标题初始化
    OLED_Clear();
    OLED_WriteString(0, 0, "READY");
    OLED_WriteString(0, 1, "L:");
    OLED_WriteString(40, 1, "R:");
    OLED_WriteString(0, 2, "WL:");
    OLED_WriteString(40, 2, "WR:");
    OLED_WriteString(0, 3, "GR:");
    OLED_WriteString(0, 4, "Y:");
    OLED_WriteString(0, 5, "G:");

    // ===================== 电机速度PID相关变量 =====================
    uint32_t lastL = 0, lastR = 0;         // 左右轮上一次编码器脉冲值
    float smoothL = 0.0f, smoothR = 0.0f; // 滤波后实时转速
    float intL = 0.0f, intR = 0.0f;       // PID积分项
    float outL = 0.0f, outR = 0.0f;       // PID输出PWM值

    // ===================== 整车控制变量 =====================
    float base_target = 0.0f;     // 基础目标行驶速度，0代表停止
    int8_t track_error = 0;       // 灰度循迹偏移误差
    float soft_speed = 0.0f;      // 缓加速平滑速度，防止起步冲击
    float yaw = 0.0f;             // MPU积分得到的航向角(°)
    int16_t gz = 0;               // MPU原始Z轴陀螺仪数据
    uint8_t motor_running = 0;    // 电机运行标志位
    uint8_t key_evt = 0;          // 按键事件标志

    // ===================== 分时调度时间戳 =====================
    uint32_t last_5ms = 0, last_30ms_pid = 0;
    uint32_t last_500ms = 0;

    while (1)
    {
        // ===================== 5ms周期任务：灰度采集+计算循迹误差 =====================
        if (tick_ms - last_5ms >= 5) {
            last_5ms = tick_ms;
            uint16_t gray_raw = Grayscale_ReadAll();
            uint16_t line = gray_raw & 0xFF;  // 低8位为8路循迹传感器，高电平代表检测到黑线

            // 加权计算轨道偏移误差
            int sum = 0, cnt = 0;
            int8_t weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
            for (int bi = 0; bi < 8; bi++) {
                if (line & (1 << bi)) { sum += weight[bi]; cnt++; }
            }
            track_error = (cnt > 0) ? (sum / cnt) : 0; // 无黑线时误差清零
        }

        // ===================== 30ms周期任务：MPU姿态、电机PID、按键扫描 =====================
        if (tick_ms - last_30ms_pid >= 30) {
            last_30ms_pid = tick_ms;
            float dt = 0.03f; // 本次计算周期30ms

            // 读取陀螺仪，更新航向角积分
            gz = MPU6050_ReadGZ();
            MPU6050_UpdateYawFromRaw(gz, dt);
            yaw = MPU6050_GetYaw();

            // 缓加速逻辑，平滑起步
            if (base_target > 0.0f) {
                if (soft_speed < base_target) {
                    soft_speed += 5.0f;
                    if (soft_speed > base_target)
                        soft_speed = base_target;
                }
            } else {
                soft_speed = 0.0f;
            }

            // 左右轮速度目标一致（循迹由PID输出叠加差速负责）
            float targetL = soft_speed;
            float targetR = soft_speed;

            // ---------------------- 左轮速度闭环PID ----------------------
            uint32_t nowL = Motor_GetLeftPulses();
            uint32_t pulseL = nowL - lastL; // 周期内新增脉冲
            lastL = nowL;
            // 脉冲换算转速：转/分钟
            float rawL = (float)pulseL * 60.0f / PULSE_PER_REV / dt;
            smoothL += (rawL - smoothL) * 0.1f; // 一阶低通滤波平滑转速

            float errL = targetL - smoothL;
            intL += errL * dt;
            // 积分限幅，防止积分饱和
            if (intL > 100.0f) intL = 100.0f;
            if (intL < -100.0f) intL = -100.0f;
            outL = 80.0f + 0.2f * errL + 0.3f * intL;
            // PWM输出限幅
            if (outL > PWM_LIMIT) outL = (float)PWM_LIMIT;
            if (outL < 0.0f) outL = 0.0f;
            // 停止状态强制PWM为0
            if (base_target == 0.0f) outL = 0.0f;

            // ---------------------- 右轮速度闭环PID ----------------------
            uint32_t nowR = Motor_GetRightPulses();
            uint32_t pulseR = nowR - lastR;
            lastR = nowR;
            float rawR = (float)pulseR * 60.0f / PULSE_PER_REV / dt;
            smoothR += (rawR - smoothR) * 0.1f;

            float errR = targetR - smoothR;
            intR += errR * dt;
            if (intR > 100.0f) intR = 100.0f;
            if (intR < -100.0f) intR = -100.0f;
            outR = 80.0f + 0.2f * errR + 0.3f * intR;
            if (outR > PWM_LIMIT) outR = (float)PWM_LIMIT;
            if (outR < 0.0f) outR = 0.0f;
            if (base_target == 0.0f) outR = 0.0f;

            // PID输出叠加循迹差速转向修正
            outL += track_error * STEER_GAIN;
            outR -= track_error * STEER_GAIN;

            // 最终PWM限幅
            if (outL > PWM_LIMIT) outL = (float)PWM_LIMIT;  if (outL < 0) outL = 0;
            if (outR > PWM_LIMIT) outR = (float)PWM_LIMIT;  if (outR < 0) outR = 0;

            // 运行输出PWM，停止则清零
            if (base_target > 0.0f)
                Motor_SetPWM((uint16_t)outL, (uint16_t)outR);
            else
                Motor_SetPWM(0, 0);

            // 按键扫描处理
            key_evt = KEY_Scan();
            if (key_evt & KEY_1) {
                // KEY1启动小车，重置积分与平滑速度
                if (!motor_running) {
                    Motor_SetForward();
                    motor_running = 1;
                }
                base_target = 100.0f;
                soft_speed = 0.0f;
                intL = 0.0f;
                intR = 0.0f;
            }
            if (key_evt & KEY_2) {
                // KEY2刹车停车，清空速度与积分项
                Motor_SetBrake();
                Motor_SetPWM(0, 0);
                base_target = 0.0f;
                soft_speed = 0.0f;
                intL = 0.0f;
                intR = 0.0f;
                motor_running = 0;
            }
        }

        // ===================== 500ms周期任务：OLED屏幕刷新 =====================
        if (tick_ms - last_500ms >= 500) {
            last_500ms = tick_ms;

            // 显示左右轮滤波后实时转速
            oled_show_val(24, 1, (uint16_t)(smoothL + 0.5f));
            oled_show_val(56, 1, (uint16_t)(smoothR + 0.5f));
            // 显示左右轮当前输出PWM
            oled_show_val(24, 2, (uint16_t)outL);
            oled_show_val(56, 2, (uint16_t)outR);

            // 打印8路灰度传感器黑白状态
            uint16_t gray = Grayscale_ReadAll();
            char gb[10]; gb[8] = 0;
            for (int bi = 0; bi < 8; bi++)
                gb[bi] = (gray & (1 << bi)) ? '0' : '1';
            OLED_WriteString(24, 3, gb);

            // 显示航向角yaw(放大10倍保留1位小数)、原始陀螺仪GZ值
            oled_show_signed(24, 4, (int16_t)(yaw * 10.0f));
            oled_show_signed(24, 5, gz);
        }
    }
}