#include "KEY.h"
#include "ti_msp_dl_config.h"

// 保存上一次按键电平状态，用于检测按键按下边沿
static uint8_t key_last = 0xFF;

/**
 * @brief 按键硬件初始化
 * 配置4个按键引脚为带上拉电阻的数字输入，无反向、无施密特、不唤醒
 * 按键默认高电平，按下拉低（低电平有效）
 */
void KEY_Init(void)
{
    // KEY1引脚配置
    DL_GPIO_initDigitalInputFeatures(KEY_KEY_1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    // KEY2引脚配置
    DL_GPIO_initDigitalInputFeatures(KEY_KEY_2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    // KEY3引脚配置
    DL_GPIO_initDigitalInputFeatures(KEY_KEY_3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    // KEY4引脚配置
    DL_GPIO_initDigitalInputFeatures(KEY_KEY_4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

/**
 * @brief 底层读取当前4路按键原始电平
 * @return 按键按下对应bit置1，松开为0
 * 引脚带上拉，按下引脚变低，取反后标记按键
 */
static uint8_t read_keys(void)
{
    uint8_t val = 0;
    // KEY1低电平=按下，对应掩码置1
    if (!(DL_GPIO_readPins(KEY_KEY_1_PORT, KEY_KEY_1_PIN) & KEY_KEY_1_PIN)) val |= KEY_1;
    // KEY2按下检测
    if (!(DL_GPIO_readPins(KEY_KEY_1_PORT, KEY_KEY_2_PIN) & KEY_KEY_2_PIN)) val |= KEY_2;
    // KEY3按下检测
    if (!(DL_GPIO_readPins(KEY_KEY_1_PORT, KEY_KEY_3_PIN) & KEY_KEY_3_PIN)) val |= KEY_3;
    // KEY4按下检测
    if (!(DL_GPIO_readPins(KEY_KEY_1_PORT, KEY_KEY_4_PIN) & KEY_KEY_4_PIN)) val |= KEY_4;
    return val;
}

/**
 * @brief 按键扫描函数，消抖+边沿检测，只返回单次按下触发信号
 * @return 按下按键掩码，无按键返回0，松手不触发
 */
uint8_t KEY_Scan(void)
{
    uint8_t now = read_keys();
    // 简易软件消抖：短暂延时后再次读取确认电平稳定
    { volatile uint32_t w = 200; while (--w); }
    uint8_t stable = read_keys();
    // 前后两次读取不一致，判定抖动，直接返回无按键
    if (now != stable) return 0;

    // 边沿检测：上一帧松开(bit=1)、当前帧按下(bit=0)，提取下降沿（按键按下瞬间）
    uint8_t pressed = (~now) & key_last;
    // 更新历史状态为本次稳定电平
    key_last = now;
    return pressed;
}

/**
 * @brief 获取按键实时电平状态
 * @return 当前所有按键持续按住的掩码，长按会持续输出
 */
uint8_t KEY_GetState(void)
{
    return read_keys();
}