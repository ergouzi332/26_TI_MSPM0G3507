#include "OLED.h"
#include "MOTOR.h"
#include "ti_msp_dl_config.h"

volatile uint32_t tick_ms = 0;
void SysTick_Handler(void) { tick_ms++; }

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(32000000 / 1000);

    OLED_Init();
    Motor_Init();

    Motor_SetTargetRPM(200);

    OLED_Clear();
    OLED_WriteString(0, 0, "MOTOR TEST");

    uint32_t t50 = 0, t200 = 0;

    while (1)
    {
        uint32_t now = tick_ms;

        /* 50ms motor */
        if (now - t50 >= 50)
        {
            t50 = now;
            Motor_SpeedUpdate();
        }

        /* 200ms OLED */
        if (now - t200 >= 200)
        {
            t200 = now;

            OLED_WriteString(0, 2, "R:");
            OLED_WriteNum(24, 2, (uint32_t)Motor_GetActualRPM(), 4);
            OLED_WriteString(56, 2, "T:");
            OLED_WriteNum(80, 2, (uint32_t)Motor_GetTargetRPM(), 3);
        }
    }
}
