#include "OLED.h"
#include "MOTOR.h"
#include "MPU6050.h"
#include "KEY.h"
#include "GRAYSCALE.h"
#include "ti_msp_dl_config.h"

#define PULSE_PER_REV  600

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

    /* 初始刹车（Motor_Init已经设为刹车） */
    Motor_SetPWM(0, 0);

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

    uint32_t lastL = 0, lastR = 0;
    uint32_t cnt = 0;
    float smoothL = 0.0f, smoothR = 0.0f;
    float intL = 0.0f, intR = 0.0f;
    float target = 0.0f;
    float dt = 0.048f;
    float soft_target = 0.0f;
    float yaw = 0.0f;
    uint8_t motor_running = 0;

    while (1)
    {
        cnt++;

        if (cnt >= 500000)
        {
            cnt = 0;

            uint8_t key_evt = KEY_Scan();
            if (key_evt & KEY_1) {
                if (!motor_running) {
                    Motor_SetForward();     /* !! 关键: 前进模式 !! */
                    motor_running = 1;
                }
                target = 200.0f;
                soft_target = 0.0f;
                intL = 0.0f;
                intR = 0.0f;
            }
            if (key_evt & KEY_2) {
                Motor_SetBrake();           /* !! 关键: 刹车 !! */
                Motor_SetPWM(0, 0);
                target = 0.0f;
                soft_target = 0.0f;
                intL = 0.0f;
                intR = 0.0f;
                motor_running = 0;
            }

            int16_t gz = MPU6050_ReadGZ();
            MPU6050_UpdateYawFromRaw(gz, dt);
            yaw = MPU6050_GetYaw();

            if (target > 0.0f) {
                if (soft_target < target) {
                    soft_target += 30.0f;
                    if (soft_target > target) soft_target = target;
                }
            } else {
                soft_target = 0.0f;
            }

            /* left */
            uint32_t nowL = Motor_GetLeftPulses();
            uint32_t pulseL = nowL - lastL;
            lastL = nowL;
            float rawL = (float)pulseL * 60.0f / PULSE_PER_REV / dt;
            smoothL += (rawL - smoothL) * 0.3f;

            float tL = soft_target;
            float errL = tL - smoothL;
            float outL = 1.2f * errL + 1.0f * intL;
            if (outL > 500.0f) outL = 500.0f;
            if (outL < 60.0f && target > 0.0f) outL = 60.0f;
            if (target == 0.0f) outL = 0.0f;
            if (outL >= 500.0f) { intL *= 0.95f; }
            else { intL += errL * dt; }
            if (intL > 150.0f) intL = 150.0f;
            if (intL < 0.0f)   intL = 0.0f;

            /* right */
            uint32_t nowR = Motor_GetRightPulses();
            uint32_t pulseR = nowR - lastR;
            lastR = nowR;
            float rawR = (float)pulseR * 60.0f / PULSE_PER_REV / dt;
            smoothR += (rawR - smoothR) * 0.3f;

            float tR = soft_target;
            float errR = tR - smoothR;
            float outR = 1.2f * errR + 1.0f * intR;
            if (outR > 500.0f) outR = 500.0f;
            if (outR < 60.0f && target > 0.0f) outR = 60.0f;
            if (target == 0.0f) outR = 0.0f;
            if (outR >= 500.0f) { intR *= 0.95f; }
            else { intR += errR * dt; }
            if (intR > 150.0f) intR = 150.0f;
            if (intR < 0.0f)   intR = 0.0f;

            Motor_SetPWM((uint16_t)outL, (uint16_t)outR);

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
