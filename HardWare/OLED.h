#ifndef OLED_H
#define OLED_H

#include <stdint.h>

/**
 * @brief SSD1306 128*64 软件I2C OLED驱动头文件
 * 硬件引脚：PA15=SCL，PA16=SDA
 * 依赖：ti_msp_dl_config.h（SysConfig生成GPIO宏）
 */

/**
 * @brief OLED屏幕初始化
 * @warning 主函数必须先调用 SYSCFG_DL_init() 再执行本函数
 */
void OLED_Init(void);

/**
 * @brief 全屏清屏，所有像素熄灭
 */
void OLED_Clear(void);

/**
 * @brief 在指定坐标打印ASCII字符串（数字、英文、符号）
 * @param x 横向像素起点 范围：0 ~ 127
 * @param y 纵向页面号 范围：0 ~ 7（一页8个像素高度）
 * @param s 待打印字符串指针，仅支持ASCII 32~127
 */
void OLED_WriteString(uint8_t x, uint8_t y, const char* s);

/**
 * @brief 打印固定长度无符号数字，不足位数自动补空格
 * @param x 横向像素起点
 * @param y 纵向页面号
 * @param num 需要显示的十进制数字（uint32_t）
 * @param len 固定显示字符总长度
 */
void OLED_WriteNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len);
void OLED_WriteInt(uint8_t x, uint8_t y, int32_t num, uint8_t len);

#endif
