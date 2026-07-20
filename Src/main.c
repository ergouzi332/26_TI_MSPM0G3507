#include "OLED.h"
#include "MOTOR.h"
#include "MPU6050.h"
#include "KEY.h"
#include "GRAYSCALE.h"
#include "uart3.h"
#include "ti_msp_dl_config.h"

volatile uint32_t tick_ms = 0;
void SysTick_Handler(void) { tick_ms++; }

#define TARGET_PULSE    12
#define PWM_FF          100.0f
#define KP              5.0f
#define KI              0.0f

#define KP_LINE         2.0f
#define KD_LINE         0.3f

#define TARGET_MIN      4
#define TARGET_MAX      30

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
    float er_filt = 0.0f, last_er_filt = 0.0f;
    uint8_t motor_run = 0;

    uint16_t d_pwmL = 0, d_pwmR = 0;
    int16_t d_pL = 0, d_pR = 0, d_er = 0, d_yaw = 0, d_gz = 0;
    uint16_t d_gray = 0;
    uint32_t last_20ms = 0, last_500ms = 0;

    // ???????CH1(?bit0)~CH8(?bit7)?0=?
    int8_t w[8] = {-7,-5,-3,-1,1,3,5,7};

    while (1)
    {
        uint32_t now = tick_ms;

        if (now - last_20ms >= 20) {
            last_20ms = now;

            // MPU
            d_gz = MPU6050_ReadGZ();
            MPU6050_UpdateYawFromRaw(d_gz, 0.02f);
            d_yaw = (int16_t)(MPU6050_GetYaw() * 10.0f);

            // ??
            uint8_t key = KEY_Scan();
            if (key & KEY_1) {
                if (!motor_run) {
                    Motor_SetForward(); motor_run = 1;
                    integralL = 0.0f; integralR = 0.0f;
                    Motor_GetLeftPulses(); Motor_GetRightPulses();
                }
            }
            if (key & KEY_2) {
                Motor_SetBrake(); Motor_SetPWM(0, 0);
                motor_run = 0; pwmL = 0; pwmR = 0;
                integralL = 0.0f; integralR = 0.0f;
                Motor_GetLeftPulses(); Motor_GetRightPulses();
            }

            // ???0=??1=??
            uint16_t line = Grayscale_ReadAll() & 0xFF;
            d_gray = line;
            int sum = 0;
            for (int i = 0; i < 8; i++) {
                if ((line & (1 << i)) == 0) sum += w[i];  // 0=?
            }
            d_er = (int16_t)sum;

            // ?? + ??
            er_filt += ((float)d_er - er_filt) * 0.3f;
            if (fabsf(er_filt) < 2.0f) er_filt = 0.0f;

            // ??PD
            float diff_er = er_filt - last_er_filt;
            last_er_filt = er_filt;
            float steer = er_filt * KP_LINE + diff_er * KD_LINE;

            // ????
            int32_t targetL = TARGET_PULSE - (int32_t)(steer + 0.5f);
            int32_t targetR = TARGET_PULSE + (int32_t)(steer + 0.5f);
            if (targetL < TARGET_MIN) targetL = TARGET_MIN;
            if (targetL > TARGET_MAX) targetL = TARGET_MAX;
            if (targetR < TARGET_MIN) targetR = TARGET_MIN;
            if (targetR > TARGET_MAX) targetR = TARGET_MAX;

            // ??P??
            if (motor_run) {
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
            } else {
                d_pL = 0; d_pR = 0;
            }
            d_pwmL = (uint16_t)pwmL; d_pwmR = (uint16_t)pwmR;
        }

        // 500ms??
        if (now - last_500ms >= 500) {
            last_500ms = now;
            char b[80]; int p = 0;
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
            for(int i=7;i>=0;i--) b[p++]=(d_gray&(1<<i))?'0':'1';
            b[p++]=' ';b[p++]='Y';b[p++]=':'; v=d_yaw;
            if(v<0){b[p++]='-';v=-v;}else b[p++]='+';
            b[p++]='0'+(v/100)%10;b[p++]='0'+(v/10)%10;b[p++]='0'+(v%10);
            b[p++]=' ';b[p++]='G';b[p++]='Z';b[p++]=':'; v=d_gz;
            if(v<0){b[p++]='-';v=-v;}else b[p++]='+';
            b[p++]='0'+(v/10000)%10;b[p++]='0'+(v/1000)%10;
            b[p++]='0'+(v/100)%10;b[p++]='0'+(v/10)%10;b[p++]='0'+(v%10);
            b[p++]='\r';b[p++]='\n';
            for(int i=0;i<p;i++) UART3_SendByte((uint8_t)b[i]);
        }
    }
}
