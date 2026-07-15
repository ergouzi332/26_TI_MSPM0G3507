#include "OLED.h"
#include "MOTOR.h"
#include "MPU6050.h"
#include "KEY.h"
#include "GRAYSCALE.h"
#include "ti_msp_dl_config.h"

/* SysTick 1ms 中断计数器 */
volatile uint32_t tick_ms = 0;

void SysTick_Handler(void)
{
    tick_ms++;
}

#define PULSE_PER_REV  300

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

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    MPU6050_Init();
    Motor_Init();
    KEY_Init();
    Grayscale_Init();
    MPU6050_CalibrateGyro();
    MPU6050_ResetYaw();

    /* 初始刹车 */
    Motor_SetPWM(0, 0);

    /* SysTick: 1ms tick @ 32MHz, 开中断 */
    SysTick->LOAD = 32000UL - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x07;   /* CLKSOURCE=1, TICKINT=1, ENABLE=1 */

    OLED_Clear();
    OLED_WriteString(0, 0, "READY");
    OLED_WriteString(0, 1, "L:");
    OLED_WriteString(40, 1, "R:");
    OLED_WriteString(0, 2, "WL:");
    OLED_WriteString(40, 2, "WR:");
    OLED_WriteString(0, 3, "GR:");
    OLED_WriteString(0, 4, "Y:");
    OLED_WriteString(0, 5, "G:");
    OLED_WriteString(0, 7, "KEY:");

    /* ========== PID 状态变量 ========== */
    uint32_t lastL = 0, lastR = 0;
    float smoothL = 0.0f, smoothR = 0.0f;
    float intL = 0.0f, intR = 0.0f;
    float outL = 0.0f, outR = 0.0f;

    /* ========== 控制变量 ========== */
    float base_target = 0.0f;
    float soft_speed = 0.0f;
    float yaw = 0.0f;
    int16_t gz = 0;
    uint8_t motor_running = 0;
    uint8_t key_evt = 0;

    /* ========== 调度时间戳 ========== */
    uint32_t last_5ms = 0, last_100ms_pid = 0;
    uint32_t last_500ms = 0;

    while (1)
    {
        /* ========== 5ms: 灰度读取 + 循迹 ========== */
        if (tick_ms - last_5ms >= 5) {
            last_5ms = tick_ms;
            /* TODO: Gray_Read() + Track_Control() */
        }

        /* ========== 50ms: 电机PID + MPU + KEY ========== */
        if (tick_ms - last_100ms_pid >= 100) {
            last_100ms_pid = tick_ms;

            float dt = 0.10f;

            /* ---- MPU 更新 ---- */
            gz = MPU6050_ReadGZ();
            MPU6050_UpdateYawFromRaw(gz, dt);
            yaw = MPU6050_GetYaw();

            /* ---- 基础速度斜坡 ---- */
            if (base_target > 0.0f) {
                if (soft_speed < base_target) {
                    soft_speed += 30.0f;
                    if (soft_speed > base_target)
                        soft_speed = base_target;
                }
            } else {
                soft_speed = 0.0f;
            }

            float targetL = soft_speed;
            float targetR = soft_speed;

            /* === 左轮 PID === */
            uint32_t nowL = Motor_GetLeftPulses();
            uint32_t pulseL = nowL - lastL;
            lastL = nowL;
            float rawL = (float)pulseL * 60.0f / PULSE_PER_REV / dt;
            smoothL += (rawL - smoothL) * 0.1f;

            float errL = targetL - smoothL;
            intL += errL * dt;
            if (intL > 100.0f) intL = 100.0f;
            if (intL < -100.0f) intL = -100.0f;
            outL = 80.0f + 0.2f * errL + 0.3f * intL;
            if (outL > 200.0f) outL = 200.0f;
            if (outL < 0.0f) outL = 0.0f;
            if (base_target == 0.0f) outL = 0.0f;

            /* === 右轮 PID === */
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
            if (outR > 200.0f) outR = 200.0f;
            if (outR < 0.0f) outR = 0.0f;
            if (base_target == 0.0f) outR = 0.0f;

            Motor_SetPWM((uint16_t)outL, (uint16_t)outR);

            /* ---- KEY scan (same 50ms cycle) ---- */
            key_evt = KEY_Scan();
            if (key_evt & KEY_1) {
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
                Motor_SetBrake();
                Motor_SetPWM(0, 0);
                base_target = 0.0f;
                soft_speed = 0.0f;
                intL = 0.0f;
                intR = 0.0f;
                motor_running = 0;
            }
        }



        /* ========== 500ms: OLED 刷新 ========== */
        if (tick_ms - last_500ms >= 500) {
            last_500ms = tick_ms;

            oled_show_val(24, 1, (uint16_t)(smoothL + 0.5f));
            oled_show_val(56, 1, (uint16_t)(smoothR + 0.5f));
            oled_show_val(24, 2, (uint16_t)outL);
            oled_show_val(56, 2, (uint16_t)outR);
            uint16_t gray = Grayscale_ReadAll();
            OLED_WriteNum(24, 3, gray, 4);
            OLED_WriteInt(24, 4, (int16_t)(yaw * 10.0f), 5);
            OLED_WriteInt(24, 5, gz, 6);
            OLED_WriteInt(24, 7, (int32_t)key_evt, 2);
        }
    }
}