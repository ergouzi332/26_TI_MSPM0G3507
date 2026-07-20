#include "GRAYSCALE.h"
#include "uart3.h"
#include "ti_msp_dl_config.h"

/**
 * @brief 8路灰度传感器初始化
 * 硬件IO初始化在SYSCFG图形工具完成，此处空函数占位
 */
void Grayscale_Init(void) {}

/**
 * @brief 通道选择函数，通过AD2/AD1/AD0三根地址线选通对应灰度通道
 * @param ch 通道号 0~7
 */
static void select_channel(uint8_t ch)
{
    // 分离通道号的二进制三位，对应地址线AD2 AD1 AD0
    uint8_t a2 = (ch >> 2) & 1;
    uint8_t a1 = (ch >> 1) & 1;
    uint8_t a0 = ch & 1;

    // 先全部拉低地址线
    DL_GPIO_clearPins(Grayscale_Sensor_PORT,
        Grayscale_Sensor_AD0_PIN | Grayscale_Sensor_AD1_PIN | Grayscale_Sensor_AD2_PIN);
    // 根据通道二进制位拉高对应地址引脚
    if (a0) DL_GPIO_setPins(Grayscale_Sensor_PORT, Grayscale_Sensor_AD0_PIN);
    if (a1) DL_GPIO_setPins(Grayscale_Sensor_PORT, Grayscale_Sensor_AD1_PIN);
    if (a2) DL_GPIO_setPins(Grayscale_Sensor_PORT, Grayscale_Sensor_AD2_PIN);
    // 短延时等待多路开关稳定输出
    { volatile uint32_t w = 2000; while (--w); }
}

/**
 * @brief 读取单路灰度传感器状态
 * @param ch 通道0~7
 * @return 1=检测到白纸(高电平)，0=检测到黑线(低电平)
 */
uint8_t Grayscale_ReadChannel(uint8_t ch)
{
    // 通道超出0~7范围直接返回0
    if (ch > 7) return 0;
    select_channel(ch);
    // 读取传感器输出引脚电平，高电平返回1，低电平返回0
    return (DL_GPIO_readPins(Grayscale_Sensor_PORT, Grayscale_Sensor_OUT_PIN) != 0) ? 1 : 0;
}

/**
 * @brief 一次性读取全部8路灰度，打包成16bit数据
 * @return uint16_t bit0~bit7对应CH0~CH7，bit=0代表黑线，bit=1代表白纸
 */
uint16_t Grayscale_ReadAll(void)
{
    uint16_t val = 0;
    // 依次读取8个通道
    for (uint8_t ch = 0; ch < 8; ch++)
    {
        // 该通道为白纸则对应bit置1
        if (Grayscale_ReadChannel(ch)) val |= (1 << ch);
    }
    return val;
}