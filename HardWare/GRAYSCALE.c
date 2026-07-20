#include "GRAYSCALE.h"
#include "uart3.h"
#include "ti_msp_dl_config.h"

void Grayscale_Init(void) {}

// 按你给的映射表： AD2 AD1 AD0 → CH
static void select_channel(uint8_t ch)
{
    // 根据 ch 设置 AD2 AD1 AD0
    uint8_t a2 = (ch >> 2) & 1;
    uint8_t a1 = (ch >> 1) & 1;
    uint8_t a0 = ch & 1;

    DL_GPIO_clearPins(Grayscale_Sensor_PORT,
        Grayscale_Sensor_AD0_PIN | Grayscale_Sensor_AD1_PIN | Grayscale_Sensor_AD2_PIN);
    if (a0) DL_GPIO_setPins(Grayscale_Sensor_PORT, Grayscale_Sensor_AD0_PIN);
    if (a1) DL_GPIO_setPins(Grayscale_Sensor_PORT, Grayscale_Sensor_AD1_PIN);
    if (a2) DL_GPIO_setPins(Grayscale_Sensor_PORT, Grayscale_Sensor_AD2_PIN);
    { volatile uint32_t w = 2000; while (--w); }
}

uint8_t Grayscale_ReadChannel(uint8_t ch)
{
    if (ch > 7) return 0;
    select_channel(ch);
    return (DL_GPIO_readPins(Grayscale_Sensor_PORT, Grayscale_Sensor_OUT_PIN) != 0) ? 1 : 0;
}

uint16_t Grayscale_ReadAll(void)
{
    uint16_t val = 0;
    for (uint8_t ch = 0; ch < 8; ch++)
    {
        if (Grayscale_ReadChannel(ch)) val |= (1 << ch);
    }
    return val;
}
