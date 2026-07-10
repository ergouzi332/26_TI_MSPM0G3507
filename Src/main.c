#include "../HardWare/Hardware.h"
#include "../HardWare/OLED.h"
#include "../HardWare/MPU6050.h"
#include "ti_msp_dl_config.h"

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();

    MPU6050_Init();

    OLED_Clear();
    OLED_WriteString(0, 0, "RAW:");
    OLED_WriteString(0, 1, "DPS:");
    OLED_WriteString(0, 2, "YAW:");



    while (1)
    {
        int16_t gz = MPU6050_ReadGZ();
        float gz_dps = (float)gz * 250.0f / 32768.0f;

        /* 调用标准 API 更新 YAW（包含 gz_offset 补偿和积分） */
        MPU6050_UpdateYawFromRaw(gz, 0.01f);
        float yaw_val = MPU6050_GetYaw();

        /* Line 0: RAW */
        OLED_WriteString(24, 0, "         ");
        int16_t gz_abs = gz;
        if (gz_abs < 0) { OLED_WriteString(24, 0, "-"); gz_abs = -gz_abs; }
        OLED_WriteNum(32, 0, (uint32_t)gz_abs, 5);

        /* Line 1: DPS + OFFSET 调试 */
        int32_t d100 = (int32_t)(gz_dps * 100.0f);
        OLED_WriteString(24, 1, "          ");
        if (d100 < 0) { OLED_WriteString(24, 1, "-"); d100 = -d100; }
        OLED_WriteNum(32, 1, (uint32_t)(d100 / 100), 3);
        OLED_WriteString(50, 1, ".");
        OLED_WriteNum(56, 1, (uint32_t)(d100 % 100), 2);

        /* Line 2: YAW */
        OLED_WriteString(24, 2, "          ");
        float yaw_disp = yaw_val;
        if (yaw_disp < 0) { OLED_WriteString(24, 2, "-"); yaw_disp = -yaw_disp; }
        uint32_t yi = (uint32_t)yaw_disp;
        uint32_t yd = (uint32_t)((yaw_disp - yi) * 100.0f + 0.5f);
        if (yd > 99) yd = 99;
        OLED_WriteNum(32, 2, yi, 3);
        OLED_WriteString(50, 2, ".");
        OLED_WriteNum(56, 2, yd, 2);

        DL_Common_delayCycles(320000U);
    }
}

