#include "GRAYSCALE.h"
#include "ti_msp_dl_config.h"

void Grayscale_Init(void) {}

static void select_channel(uint8_t ch)
{
    DL_GPIO_clearPins(Grayscale_Sensor_PORT, Grayscale_Sensor_AD0_PIN | Grayscale_Sensor_AD1_PIN | Grayscale_Sensor_AD2_PIN);
    if (ch & 0x01) DL_GPIO_setPins(Grayscale_Sensor_PORT, Grayscale_Sensor_AD0_PIN);
    if (ch & 0x02) DL_GPIO_setPins(Grayscale_Sensor_PORT, Grayscale_Sensor_AD1_PIN);
    if (ch & 0x04) DL_GPIO_setPins(Grayscale_Sensor_PORT, Grayscale_Sensor_AD2_PIN);
    { volatile uint32_t w = 200; while (--w); }
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
