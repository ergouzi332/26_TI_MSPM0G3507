#include "OLED.h"
#include "MPU6050.h"
#include "MOTOR.h"
#include "ENCODER.h"
#include "GRAYSCALE.h"
#include "BUZZER.h"
#include "ti_msp_dl_config.h"

volatile uint32_t tick_ms = 0;
void SysTick_Handler(void) { tick_ms++; }

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(32000000 / 1000);

    OLED_Init();
    MPU6050_Init();
    MPU6050_CalibrateGyro();
    Motor_Init();
    Encoder_Init();
    Grayscale_Init();
    Buzzer_Init();

    OLED_Clear();
    OLED_WriteString(0, 0, "RDY");
    float off = MPU6050_GetGzOffset();
    OLED_WriteString(0, 1, "O:");
    if (off < 0) { OLED_WriteString(24, 1, "-"); off = -off; }
    OLED_WriteNum(32, 1, (uint32_t)off, 5);
    OLED_WriteString(0, 2, "Y:");
    OLED_WriteNum(24, 2, 0, 4);
    OLED_WriteString(56, 2, ".0");

    uint32_t t_last_imu = 0, t50 = 0, t200 = 0;
    float yaw = 0.0f;

    while (1)
    {
        uint32_t now = tick_ms;

        /* ========== 10ms 任务：IMU + 编码器 ========== */
        if (now - t_last_imu >= 10)
        {
            float dt = (float)(now - t_last_imu) / 1000.0f;
            t_last_imu = now;
            int16_t gz = MPU6050_ReadGZ();
            MPU6050_UpdateYawFromRaw(gz, dt);
            yaw = MPU6050_GetYaw();
        }

        /* ========== 50ms 任务：控制计算 ========== */
        if (now - t50 >= 50)
        {
            t50 = now;
            /* TODO: 循迹 PID + 电机控制 */
        }

        /* ========== 200ms 任务：OLED 显示 ========== */
        if (now - t200 >= 200)
        {
            t200 = now;

            float y = yaw;
            uint8_t sign = 0;
            if (y < 0) { sign = 1; y = -y; }
            int16_t yi = (int16_t)y;
            uint8_t yd = (uint8_t)((y - (float)yi) * 10.0f + 0.5f);
            OLED_WriteString(0, 2, "Y:");
            if (sign) OLED_WriteString(24, 2, "-");
            OLED_WriteNum(32, 2, (uint32_t)yi, 3);
            OLED_WriteString(56, 2, ".");
            OLED_WriteNum(64, 2, (uint32_t)yd, 1);

            int16_t gz = MPU6050_GetLastGz();
            OLED_WriteString(0, 4, "G:");
            if (gz < 0) { OLED_WriteString(24, 4, "-"); gz = -gz; }
            OLED_WriteNum(32, 4, (uint32_t)gz, 5);
        }
    }
}
