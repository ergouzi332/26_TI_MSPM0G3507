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
    OLED_WriteString(0, 0, "PID TEST");
    OLED_WriteString(0, 1, "R:");
    OLED_WriteString(0, 2, "P:");
    OLED_WriteString(0, 3, "W:");

    uint32_t last = 0;
    uint32_t cnt = 0;
    float smooth_rpm = 0.0f;
    float integrator = 0.0f;
    float target = 200.0f;
    float soft_target = 0.0f;
    float dt = 0.048f;

    while (1)
    {
        cnt++;
        if (cnt >= 500000)
        {
            cnt = 0;

            /* ???????+40 */
            if (soft_target < target)
            {
                soft_target += 40.0f;
                if (soft_target > target) soft_target = target;
            }

            /* ?? */
            uint32_t now = Motor_GetPulses();
            uint32_t pulse = now - last;
            last = now;

            float raw_rpm = (float)pulse * 60.0f / (float)PULSE_PER_REV / dt;
            smooth_rpm += (raw_rpm - smooth_rpm) * 0.3f;

            /* PID */
            float err = soft_target - smooth_rpm;
            float Kp = 1.2f;
            float Ki = 1.0f;
            float output = Kp * err + Ki * integrator;

            if (output > 500.0f) output = 500.0f;
            if (output < 60.0f)  output = 60.0f;

            if (output < 500.0f)
            {
                integrator += err * dt;
                if (integrator > 400.0f) integrator = 400.0f;
                if (integrator < 0.0f)   integrator = 0.0f;
            }

            uint16_t pwm = (uint16_t)output;
            PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = pwm;

            oled_show_val(24, 1, (uint16_t)(smooth_rpm + 0.5f));
            oled_show_val(24, 2, (uint16_t)pulse);
            oled_show_val(24, 3, pwm);
        }
    }
}
