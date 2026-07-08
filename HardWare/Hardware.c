#include "Hardware.h"
#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "MPU6050.h"
#include "ENCODER.h"
#include "BUZZER.h"

void Hardware_Init(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    MPU6050_Init();
    //Encoder_Init();
    //Buzzer_Init();
}
