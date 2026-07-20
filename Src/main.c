#include "OLED.h"
#include "MOTOR.h"
#include "MPU6050.h"
#include "KEY.h"
#include "GRAYSCALE.h"
#include "uart3.h"
#include "ti_msp_dl_config.h"

volatile uint32_t tick_ms = 0;
void SysTick_Handler(void) { tick_ms++; }

// === 脉冲 PID 参数 ===
#define TARGET_PULSE    9           // PWM100 时 ~10 脉冲/20ms
#define PWM_FF          100.0f
#define KP              5.0f
#define KI              0.2f
#define INTEGRAL_LIMIT  500.0f

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init(); MPU6050_Init(); Motor_Init(); KEY_Init(); Grayscale_Init();
    MPU6050_CalibrateGyro(); MPU6050_ResetYaw();
    Motor_SetPWM(0, 0); UART3_Init(); UART3_Test();

    SysTick->LOAD = 32000UL - 1; SysTick->VAL = 0; SysTick->CTRL = 0x07;

    OLED_Clear(); OLED_WriteString(0, 0, "INIT DONE");

    float integralL = 0.0f, integralR = 0.0f;
    int16_t pwmL = 0, pwmR = 0;
    uint8_t motor_run = 0;

    // 调试变量
    uint16_t d_pwmL = 0, d_pwmR = 0;
    int16_t d_pL = 0, d_pR = 0;
    int16_t d_er = 0, d_yaw = 0, d_gz = 0;
    uint16_t d_gray = 0;

    uint32_t last_20ms = 0, last_500ms = 0;

    while (1)
    {
        uint32_t now = tick_ms;

        // ========== 20ms ==========
        if (now - last_20ms >= 20) {
            last_20ms = now;

            // MPU
            d_gz = MPU6050_ReadGZ();
            MPU6050_UpdateYawFromRaw(d_gz, 0.02f);
            d_yaw = (int16_t)(MPU6050_GetYaw() * 10.0f);

            // 按键
            uint8_t key = KEY_Scan();
            if (key & KEY_1) {
                if (!motor_run) {
                    Motor_SetForward();
                    motor_run = 1;
                    integralL = 0.0f; integralR = 0.0f;
                    Motor_GetLeftPulses();  // 清零
                    Motor_GetRightPulses();
                }
            }
            if (key & KEY_2) {
                Motor_SetBrake(); Motor_SetPWM(0, 0);
                motor_run = 0; pwmL = 0; pwmR = 0;
                integralL = 0.0f; integralR = 0.0f;
                Motor_GetLeftPulses();
                Motor_GetRightPulses();
            }

            // 脉冲 PID（读后自动清零）
            if (motor_run) {
                int32_t pL = (int32_t)Motor_GetLeftPulses();
                d_pL = (int16_t)pL;
                float errL = (float)(TARGET_PULSE - pL);
                integralL += errL;
                if (integralL > INTEGRAL_LIMIT) integralL = INTEGRAL_LIMIT;
                if (integralL < -INTEGRAL_LIMIT) integralL = -INTEGRAL_LIMIT;
                float outL = PWM_FF + KP * errL + KI * integralL;
                if (outL < 0.0f) outL = 0.0f;
                if (outL > 300.0f) outL = 300.0f;
                pwmL = (int16_t)(outL + 0.5f);

                int32_t pR = (int32_t)Motor_GetRightPulses();
                d_pR = (int16_t)pR;
                float errR = (float)(TARGET_PULSE - pR);
                integralR += errR;
                if (integralR > INTEGRAL_LIMIT) integralR = INTEGRAL_LIMIT;
                if (integralR < -INTEGRAL_LIMIT) integralR = -INTEGRAL_LIMIT;
                float outR = PWM_FF + KP * errR + KI * integralR;
                if (outR < 0.0f) outR = 0.0f;
                if (outR > 300.0f) outR = 300.0f;
                pwmR = (int16_t)(outR + 0.5f);

                Motor_SetPWM((uint16_t)pwmL, (uint16_t)pwmR);
            } else {
                d_pL = 0; d_pR = 0;
            }

            d_pwmL = (uint16_t)pwmL;
            d_pwmR = (uint16_t)pwmR;

            // 灰度
            uint16_t line = Grayscale_ReadAll() & 0xFF;
            d_gray = line;
            int sum = 0;
            int8_t w[8] = {-7,-5,-3,-1,1,3,5,7};
            for (int i = 0; i < 8; i++) { if (line & (1 << i)) sum += w[i]; }
            d_er = (int16_t)sum;
        }

        // ========== 500ms: 串口 ==========
        if (now - last_500ms >= 500) {
            last_500ms = now;

            // "PWM:xxx L:+xx R:+xx ER:+xx GR:xxxxxxxx Y:+xxxx GZ:+xxxxx\r\n"
            UART3_SendByte('P'); UART3_SendByte('W'); UART3_SendByte('M'); UART3_SendByte(':');
            UART3_SendByte('0' + (d_pwmL / 100) % 10);
            UART3_SendByte('0' + (d_pwmL / 10) % 10);
            UART3_SendByte('0' + (d_pwmL % 10));
            UART3_SendByte(' '); UART3_SendByte('L'); UART3_SendByte(':');
            int16_t v = d_pL;
            if (v < 0) { UART3_SendByte('-'); v = -v; } else { UART3_SendByte('+'); }
            UART3_SendByte('0' + (v / 10) % 10);
            UART3_SendByte('0' + (v % 10));
            UART3_SendByte(' '); UART3_SendByte('R'); UART3_SendByte(':');
            v = d_pR;
            if (v < 0) { UART3_SendByte('-'); v = -v; } else { UART3_SendByte('+'); }
            UART3_SendByte('0' + (v / 10) % 10);
            UART3_SendByte('0' + (v % 10));

            UART3_SendByte(' '); UART3_SendByte('E'); UART3_SendByte('R'); UART3_SendByte(':');
            v = d_er;
            if (v < 0) { UART3_SendByte('-'); v = -v; } else { UART3_SendByte('+'); }
            UART3_SendByte('0' + (v / 10) % 10);
            UART3_SendByte('0' + (v % 10));

            UART3_SendByte(' '); UART3_SendByte('G'); UART3_SendByte('R'); UART3_SendByte(':');
            for (int b = 7; b >= 0; b--) {
                UART3_SendByte((d_gray & (1 << b)) ? '0' : '1');
            }

            UART3_SendByte(' '); UART3_SendByte('Y'); UART3_SendByte(':');
            v = d_yaw;
            if (v < 0) { UART3_SendByte('-'); v = -v; } else { UART3_SendByte('+'); }
            UART3_SendByte('0' + (v / 100) % 10);
            UART3_SendByte('0' + (v / 10) % 10);
            UART3_SendByte('0' + (v % 10));

            UART3_SendByte(' '); UART3_SendByte('G'); UART3_SendByte('Z'); UART3_SendByte(':');
            v = d_gz;
            if (v < 0) { UART3_SendByte('-'); v = -v; } else { UART3_SendByte('+'); }
            UART3_SendByte('0' + (v / 10000) % 10);
            UART3_SendByte('0' + (v / 1000) % 10);
            UART3_SendByte('0' + (v / 100) % 10);
            UART3_SendByte('0' + (v / 10) % 10);
            UART3_SendByte('0' + (v % 10));

            UART3_SendByte('\r'); UART3_SendByte('\n');
        }
    }
}
