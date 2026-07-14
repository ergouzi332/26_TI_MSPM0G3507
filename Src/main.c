#include "OLED.h"
#include "MOTOR.h"
#include "MPU6050.h"
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
    buf[5] = '\0';
    OLED_WriteString(x, y, buf);
}

int main(void)
{
    SYSCFG_DL_init();
    /* SysTick as free-run counter (no interrupt) */
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x00000005;
    OLED_Init();
    MPU6050_Init();
    Motor_Init();
    MPU6050_CalibrateGyro();
    MPU6050_ResetYaw();

    OLED_Clear();
    OLED_WriteString(0, 0, "DUAL+YAWA");
    OLED_WriteString(0, 1, "L:");
    OLED_WriteString(40, 1, "R:");
    OLED_WriteString(0, 2, "WL:");
    OLED_WriteString(40, 2, "WR:");
    OLED_WriteString(0, 4, "Y:");
    OLED_WriteString(0, 5, "G:");

    uint32_t lastL = 0, lastR = 0;
    uint32_t cnt = 0;
    float smoothL = 0.0f, smoothR = 0.0f;
    float intL = 0.0f, intR = 0.0f;
    float target = 200.0f;
    float dt = 0.048f;
    uint32_t tick_last = 0;
    float soft_target = 0.0f;

    float yaw = 0.0f;
    float my_yaw = 0.0f;

    while (1)
    {
        cnt++;
        if (cnt >= 500000)
        {
            cnt = 0;

            /* ---- yaw (local integration + auto bias) ---- */
            int16_t gz = MPU6050_ReadGZ();
            {
                static float bias = 0.0f;
                /* real dt from free-run counter */
                uint32_t tick_now = SysTick->VAL;
                float dt_yaw = (float)((tick_last - tick_now) & 0x00FFFFFF) / 32000000.0f;
                tick_last = tick_now;
                if (dt_yaw > 0.5f) dt_yaw = dt;
                static int bias_cnt = 0;

                if (bias_cnt < 100) {
                    bias += (float)gz;
                    bias_cnt++;
                    if (bias_cnt == 100) bias /= 100.0f;
                }

                float gz_dps = ((float)gz - bias) * 250.0f / 32768.0f;

                if (fabsf(gz_dps) < 2.0f) {
                    gz_dps = 0.0f;
                    bias = bias * 0.99f + (float)gz * 0.01f;
                }

                my_yaw += gz_dps * dt_yaw;
                if (my_yaw > 180.0f) my_yaw -= 360.0f;
                if (my_yaw < -180.0f) my_yaw += 360.0f;
                yaw = my_yaw;
            }

            /* soft target */
            if (soft_target < target) {
                soft_target += 30.0f;
                if (soft_target > target) soft_target = target;
            }

            float tL = soft_target;
            float tR = soft_target;

            /* ---- left ---- */
            uint32_t nowL = Motor_GetLeftPulses();
            uint32_t pulseL = nowL - lastL;
            lastL = nowL;
            float rawL = (float)pulseL * 60.0f / PULSE_PER_REV / dt;
            smoothL += (rawL - smoothL) * 0.3f;

            float errL = tL - smoothL;
            float outL = 1.2f * errL + 1.0f * intL;
            if (outL > 500.0f) outL = 500.0f;
            if (outL < 60.0f)  outL = 60.0f;
            if (outL >= 500.0f) { intL *= 0.95f; }
            else { intL += errL * dt; }
            if (intL > 150.0f) intL = 150.0f;
            if (intL < 0.0f)   intL = 0.0f;

            /* ---- right ---- */
            uint32_t nowR = Motor_GetRightPulses();
            uint32_t pulseR = nowR - lastR;
            lastR = nowR;
            float rawR = (float)pulseR * 60.0f / PULSE_PER_REV / dt;
            smoothR += (rawR - smoothR) * 0.3f;

            float errR = tR - smoothR;
            float outR = 1.2f * errR + 1.0f * intR;
            if (outR > 500.0f) outR = 500.0f;
            if (outR < 60.0f)  outR = 60.0f;
            if (outR >= 500.0f) { intR *= 0.95f; }
            else { intR += errR * dt; }
            if (intR > 150.0f) intR = 150.0f;
            if (intR < 0.0f)   intR = 0.0f;

            Motor_SetPWM((uint16_t)outL, (uint16_t)outR);

            oled_show_val(24, 1, (uint16_t)(smoothL + 0.5f));
            oled_show_val(56, 1, (uint16_t)(smoothR + 0.5f));
            oled_show_val(24, 2, (uint16_t)outL);
            oled_show_val(56, 2, (uint16_t)outR);
            OLED_WriteInt(24, 4, (int16_t)(yaw * 10.0f), 5);
            OLED_WriteInt(24, 5, gz, 6);
        }
    }
}
