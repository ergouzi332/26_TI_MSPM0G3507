#include "../HardWare/Hardware.h"
#include "../HardWare/OLED.h"
#include "../HardWare/MPU6050.h"
#include "../HardWare/MOTOR.h"
#include "ti_msp_dl_config.h"

int main(void)
{
    Hardware_Init();
    Motor_Init();
    OLED_Clear();

    OLED_WriteString(0, 0, "T:");
    OLED_WriteString(0, 1, "A:");

    Motor_SetTargetRPM(220);

    while (1)
    {
        Motor_SpeedUpdate();

        OLED_WriteNum(16, 0, Motor_GetTargetRPM(), 4);
        OLED_WriteNum(16, 1, Motor_GetActualRPM(), 4);

        DL_Common_delayCycles(320000U);
    }
}
