#include "OLED.h"
#include "MOTOR.h"
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
    OLED_Init();
    Motor_Init();

    OLED_Clear();
    OLED_WriteString(0, 0, "DUAL PID");
    OLED_WriteString(0, 1, "L:");   /* ???? */
    OLED_WriteString(0, 2, "R:");   /* ???? */
    OLED_WriteString(0, 3, "WL:");  /* ??PWM */
    OLED_WriteString(0, 4, "WR:");  /* ??PWM */

    uint32_t lastL = 0, lastR = 0;
    uint32_t cnt = 0;
    float smoothL = 0.0f, smoothR = 0.0f;
    float intL = 0.0f, intR = 0.0f;
    float target = 200.0f;
    float dt = 0.048f;

    while (1)
    {
        cnt++;
        if (cnt >= 500000)
        {
            cnt = 0;

            /* ---- ?? ---- */
            uint32_t nowL = Motor_GetLeftPulses();
            uint32_t pulseL = nowL - lastL;
            lastL = nowL;
            float rawL = (float)pulseL * 60.0f / PULSE_PER_REV / dt;
            smoothL += (rawL - smoothL) * 0.3f;

            float errL = target - smoothL;
            float outL = 1.2f * errL + 1.0f * intL;
            if (outL > 500.0f) outL = 500.0f;
            if (outL < 60.0f)  outL = 60.0f;
            if (outL < 500.0f) {
                intL += errL * dt;
                if (intL > 400.0f) intL = 400.0f;
                if (intL < 0.0f)   intL = 0.0f;
            }

            /* ---- ?? ---- */
            uint32_t nowR = Motor_GetRightPulses();
            uint32_t pulseR = nowR - lastR;
            lastR = nowR;
            float rawR = (float)pulseR * 60.0f / PULSE_PER_REV / dt;
            smoothR += (rawR - smoothR) * 0.3f;

            float errR = target - smoothR;
            float outR = 1.2f * errR + 1.0f * intR;
            if (outR > 500.0f) outR = 500.0f;
            if (outR < 60.0f)  outR = 60.0f;
            if (outR < 500.0f) {
                intR += errR * dt;
                if (intR > 400.0f) intR = 400.0f;
                if (intR < 0.0f)   intR = 0.0f;
            }

            Motor_SetPWM((uint16_t)outL, (uint16_t)outR);

            oled_show_val(24, 1, (uint16_t)(smoothL + 0.5f));
            oled_show_val(24, 2, (uint16_t)(smoothR + 0.5f));
            oled_show_val(24, 3, (uint16_t)outL);
            oled_show_val(24, 4, (uint16_t)outR);
        }
    }
}
