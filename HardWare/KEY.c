#include "KEY.h"
#include "ti_msp_dl_config.h"

static uint8_t key_last = 0xFF;

void KEY_Init(void)
{
    DL_GPIO_initDigitalInputFeatures(KEY_KEY_1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY_KEY_2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY_KEY_3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY_KEY_4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

static uint8_t read_keys(void)
{
    uint8_t val = 0;
    if (!(DL_GPIO_readPins(KEY_PORT, KEY_KEY_1_PIN) & KEY_KEY_1_PIN)) val |= KEY_1;
    if (!(DL_GPIO_readPins(KEY_PORT, KEY_KEY_2_PIN) & KEY_KEY_2_PIN)) val |= KEY_2;
    if (!(DL_GPIO_readPins(KEY_PORT, KEY_KEY_3_PIN) & KEY_KEY_3_PIN)) val |= KEY_3;
    if (!(DL_GPIO_readPins(KEY_PORT, KEY_KEY_4_PIN) & KEY_KEY_4_PIN)) val |= KEY_4;
    return val;
}

uint8_t KEY_Scan(void)
{
    uint8_t now = read_keys();
    { volatile uint32_t w = 5000; while (--w); }
    uint8_t stable = read_keys();
    if (now != stable) return 0;
    uint8_t pressed = now & ~key_last;
    key_last = now;
    return pressed;
}

uint8_t KEY_GetState(void)
{
    return read_keys();
}
