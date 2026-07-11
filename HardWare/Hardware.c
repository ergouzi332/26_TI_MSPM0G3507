#include "Hardware.h"
#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "MPU6050.h"
#include "MOTOR.h"
#include "ENCODER.h"
#include "GRAYSCALE.h"
#include "BUZZER.h"

void Hardware_Init(void)
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

    Buzzer_Beep(100);  /* 上电提示音 */
}
