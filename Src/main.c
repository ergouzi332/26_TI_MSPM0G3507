#include "OLED.h"
#include "MOTOR.h"
#include "MPU6050.h"
#include "KEY.h"
#include "GRAYSCALE.h"
#include "uart3.h"
#include "ti_msp_dl_config.h"

volatile uint32_t tick_ms = 0;

void SysTick_Handler(void) { tick_ms++; }

#define PPR             520
#define TARGET_RPM      150.0f
#define PWM_FEEDFORWARD 100.0f
#define KP              0.08f
#define SPEED_ALPHA     0.4f

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
    SYSCFG_DL_init();
    OLED_Init(); MPU6050_Init(); Motor_Init(); KEY_Init(); Grayscale_Init();
    MPU6050_CalibrateGyro(); MPU6050_ResetYaw();
    Motor_SetPWM(0, 0); UART3_Init(); UART3_Test();

    SysTick->LOAD = 32000UL - 1; SysTick->VAL = 0; SysTick->CTRL = 0x07;

    OLED_Clear();
    OLED_WriteString(0, 0, "READY");
    OLED_WriteString(0, 1, "L:"); OLED_WriteString(40, 1, "R:");
    OLED_WriteString(0, 2, "WL:"); OLED_WriteString(40, 2, "WR:");
    OLED_WriteString(0, 3, "GR:"); OLED_WriteString(0, 4, "Y:");
    OLED_WriteString(0, 5, "G:");

    uint32_t last_encL = 0, last_encR = 0;
    float smoothL = 0.0f, smoothR = 0.0f;
    uint8_t motor_run = 0;
    uint16_t pwm_outL = 0, pwm_outR = 0;
    int16_t track_error = 0, gz = 0;
    float yaw = 0.0f;

    uint32_t last_5ms = 0, last_10ms = 0, last_20ms = 0;
    uint32_t last_100ms = 0, last_500ms = 0;

    while (1)
    {
        uint32_t now = tick_ms;

        // 5ms: grayscale
        if (now - last_5ms >= 5) {
            last_5ms = now;
            uint16_t line = Grayscale_ReadAll() & 0xFF;
            int sum = 0;
            int8_t w[8] = {-7,-5,-3,-1,1,3,5,7};
            for (int i = 0; i < 8; i++) { if (line & (1 << i)) sum += w[i]; }
            track_error = sum;
        }

        // 10ms: key scan
        if (now - last_10ms >= 10) {
            last_10ms = now;
            uint8_t key = KEY_Scan();
            if (key & KEY_1) {
                if (!motor_run) {
                    Motor_SetForward(); motor_run = 1;
                    last_encL = Motor_GetLeftPulses();
                    last_encR = Motor_GetRightPulses();
                    smoothL = 0.0f; smoothR = 0.0f;
                }
            }
            if (key & KEY_2) {
                Motor_SetBrake(); Motor_SetPWM(0, 0);
                motor_run = 0; pwm_outL = 0; pwm_outR = 0;
                smoothL = 0.0f; smoothR = 0.0f;
            }
        }

        // 20ms: MPU
        if (now - last_20ms >= 20) {
            last_20ms = now;
            gz = MPU6050_ReadGZ();
            MPU6050_UpdateYawFromRaw(gz, 0.02f);
            yaw = MPU6050_GetYaw();
        }

        // 100ms: speed closed-loop (P-only)
        if (now - last_100ms >= 100) {
            last_100ms = now;
            float dt = 0.1f;

            if (motor_run) {
                uint32_t nL = Motor_GetLeftPulses();
                uint32_t pL = nL - last_encL;
                last_encL = nL;
                float rawL = (float)pL * 60.0f / (float)PPR / dt;
                smoothL += (rawL - smoothL) * SPEED_ALPHA;

                uint32_t nR = Motor_GetRightPulses();
                uint32_t pR = nR - last_encR;
                last_encR = nR;
                float rawR = (float)pR * 60.0f / (float)PPR / dt;
                smoothR += (rawR - smoothR) * SPEED_ALPHA;

                float errL = TARGET_RPM - smoothL;
                float errR = TARGET_RPM - smoothR;
                float pwmL = PWM_FEEDFORWARD + KP * errL;
                float pwmR = PWM_FEEDFORWARD + KP * errR;

                if (pwmL < 0.0f) pwmL = 0.0f;
                if (pwmL > 300.0f) pwmL = 300.0f;
                if (pwmR < 0.0f) pwmR = 0.0f;
                if (pwmR > 300.0f) pwmR = 300.0f;

                pwm_outL = (uint16_t)(pwmL + 0.5f);
                pwm_outR = (uint16_t)(pwmR + 0.5f);

                Motor_SetPWM(pwm_outL, pwm_outR);
            }

            UART3_PrintRPM(pwm_outL, (uint16_t)(smoothL + 0.5f),
                                     (uint16_t)(smoothR + 0.5f));
        }

        // 500ms: OLED
        if (now - last_500ms >= 500) {
            last_500ms = now;
            oled_show_val(24, 1, (uint16_t)(smoothL + 0.5f));
            oled_show_val(56, 1, (uint16_t)(smoothR + 0.5f));
            oled_show_val(24, 2, pwm_outL);
            oled_show_val(56, 2, pwm_outR);

            uint16_t g = Grayscale_ReadAll();
            char gb[10]; gb[8] = 0;
            for (int i = 0; i < 8; i++)
                gb[i] = (g & (1 << i)) ? '0' : '1';
            OLED_WriteString(24, 3, gb);

            oled_show_signed(24, 4, (int16_t)(yaw * 10.0f));
            oled_show_signed(24, 5, gz);
        }
    }
}

