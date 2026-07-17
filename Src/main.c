#include "OLED.h"
#include "MOTOR.h"
#include "MPU6050.h"
#include "KEY.h"
#include "GRAYSCALE.h"
#include "ti_msp_dl_config.h"

/* SysTick 1ms ä¸­æ–­è®¡æ•°å™?*/
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
    OLED_Init();
    MPU6050_Init();
    Motor_Init();
    KEY_Init();
    Grayscale_Init();
    MPU6050_CalibrateGyro();
    MPU6050_ResetYaw();

    /* åˆå§‹åˆ¹è½¦ */
    Motor_SetPWM(0, 0);

    /* SysTick: 1ms tick @ 32MHz, å¼€ä¸­æ–­ */
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

    /* ========== PID çŠ¶æ€å˜é‡?========== */
    uint32_t lastL = 0, lastR = 0;
    float smoothL = 0.0f, smoothR = 0.0f;
    float intL = 0.0f, intR = 0.0f;
    float outL = 0.0f, outR = 0.0f;

    /* ========== æŽ§åˆ¶å˜é‡ ========== */
    float base_target = 0.0f;
    int8_t track_error = 0;
    float soft_speed = 0.0f;
    float yaw = 0.0f;
    int16_t gz = 0;
    uint8_t motor_running = 0;
    uint8_t key_evt = 0;

    /* ========== è°ƒåº¦æ—¶é—´æˆ?========== */
    uint32_t last_5ms = 0, last_100ms_pid = 0;
    uint32_t last_500ms = 0;

    while (1)
    {

        /* ========== 5ms: »Ò¶È¶ÁÈ¡ + ¼ÆËãÎó²î ========== */
        if (tick_ms - last_5ms >= 5) {
            last_5ms = tick_ms;
            uint16_t gray_raw = Grayscale_ReadAll();
            uint16_t line = gray_raw & 0xFF;  /* 1=ºÚÏß(´«¸ÐÆ÷¸ßµçÆ½±íÊ¾ºÚ) */

            /* ¼ÓÈ¨Æ½¾ù¼ÆËãÆ«ÒÆ */
            int sum = 0, cnt = 0;
            int8_t weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
            for (int bi = 0; bi < 8; bi++) {
                if (line & (1 << bi)) { sum += weight[bi]; cnt++; }
            }
            track_error = (cnt > 0) ? (sum / cnt) : 0;
        }

        /* ========== 50ms: ç”µæœºPID + MPU + KEY ========== */
        if (tick_ms - last_100ms_pid >= 100) {
            last_100ms_pid = tick_ms;

            float dt = 0.10f;

            /* ---- MPU æ›´æ–° ---- */
            gz = MPU6050_ReadGZ();
            MPU6050_UpdateYawFromRaw(gz, dt);
            yaw = MPU6050_GetYaw();

            /* ---- åŸºç¡€é€Ÿåº¦æ–œå¡ ---- */
            if (base_target > 0.0f) {
                if (soft_speed < base_target) {
                    soft_speed += 30.0f;
                    if (soft_speed > base_target)
                        soft_speed = base_target;
                }
            } else {
                soft_speed = 0.0f;
            }

            float targetL = soft_speed + track_error * 3;
            float targetR = soft_speed - track_error * 3;

            /* === å·¦è½® PID === */
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

            /* === å³è½® PID === */
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



        /* ========== 500ms: OLED Ë¢ÐÂ ========== */
        if (tick_ms - last_500ms >= 500) {
            last_500ms = tick_ms;

            oled_show_val(24, 1, (uint16_t)(smoothL + 0.5f));
            oled_show_val(56, 1, (uint16_t)(smoothR + 0.5f));
            oled_show_val(24, 2, (uint16_t)outL);
            oled_show_val(56, 2, (uint16_t)outR);

            uint16_t gray = Grayscale_ReadAll();
            char gb[10]; gb[8] = 0;
            for (int bi = 0; bi < 8; bi++)
                gb[bi] = (gray & (1 << bi)) ? '0' : '1';
            OLED_WriteString(24, 3, gb);

            oled_show_signed(24, 4, (int16_t)(yaw * 10.0f));
            oled_show_signed(24, 5, gz);
        }
    }
}
