/* OLED driver (SSD1306) using software I2C bit-bang on PA15/PA16
 * PA15 = SCL, PA16 = SDA
 * 由于板上硬件 I2C 线路异常，改为 GPIO 模拟 I2C 通信
 * 适配MSPM0G3507，SysConfig已预先将PA15/PA16配置为Digital Output
 * 修复点：删除浮空输出、读写SDA动态切换输入输出、不覆盖SysConfig引脚配置
 */
#include "OLED.h"
#include "ti_msp_dl_config.h"
#include <string.h>

// SSD1306 OLED I2C设备地址，多数0.96寸屏为0x3C，部分屏为0x3D
#define SSD1306_ADDR 0x3C
// I2C通信控制字节：00代表后续发送命令，40代表后续发送显存数据
#define SSD1306_CONTROL_CMD  0x00
#define SSD1306_CONTROL_DATA 0x40

/**
 * @brief 6x8 ASCII标准点阵字库
 * 偏移32对应空格开始，每个字符占用6字节纵向点阵，仅支持ASCII 32~127
 */
static const unsigned char F6X8[] = {
0x00,0x00,0x00,0x00,0x00,0x00,// 空格 32
0x00,0x00,0x5F,0x00,0x00,0x00,// !    33
0x00,0x07,0x00,0x07,0x00,0x00,// "    34
0x14,0x7F,0x14,0x7F,0x14,0x00,// #    35
0x24,0x2A,0x7F,0x2A,0x12,0x00,// $    36
0x23,0x13,0x08,0x64,0x62,0x00,// %    37
0x36,0x49,0x55,0x22,0x50,0x00,// &    38
0x00,0x05,0x03,0x00,0x00,0x00,// '    39
0x00,0x1C,0x22,0x41,0x00,0x00,// (    40
0x00,0x41,0x22,0x1C,0x00,0x00,// )    41
0x08,0x2A,0x1C,0x2A,0x08,0x00,// *    42
0x08,0x08,0x3E,0x08,0x08,0x00,// +    43
0x00,0x50,0x30,0x00,0x00,0x00,// ,    44
0x08,0x08,0x08,0x08,0x08,0x00,// -    45
0x00,0x60,0x60,0x00,0x00,0x00,// .    46
0x20,0x10,0x08,0x04,0x02,0x00,// /    47
0x3E,0x51,0x49,0x45,0x3E,0x00,// 0    48
0x00,0x42,0x7F,0x40,0x00,0x00,// 1    49
0x42,0x61,0x51,0x49,0x46,0x00,// 2    50
0x21,0x41,0x45,0x4B,0x3A,0x00,// 3    51
0x18,0x14,0x12,0x7F,0x10,0x00,// 4    52
0x27,0x45,0x45,0x45,0x39,0x00,// 5    53
0x3C,0x4A,0x49,0x49,0x30,0x00,// 6    54
0x01,0x71,0x09,0x05,0x03,0x00,// 7    55
0x36,0x49,0x49,0x49,0x36,0x00,// 8    56
0x06,0x49,0x49,0x4A,0x3E,0x00,// 9    57
0x00,0x36,0x36,0x00,0x00,0x00,// :    58
0x00,0x56,0x36,0x00,0x00,0x00,// ;    59
0x08,0x14,0x22,0x41,0x00,0x00,// <    60
0x14,0x14,0x14,0x14,0x14,0x00,// =    61
0x00,0x41,0x22,0x14,0x08,0x00,// >    62
0x02,0x01,0x51,0x09,0x06,0x00,// ?    63
0x32,0x49,0x79,0x41,0x3E,0x00,// @    64
0x7E,0x11,0x11,0x11,0x7E,0x00,// A    65
0x7F,0x49,0x49,0x49,0x36,0x00,// B    66
0x3E,0x41,0x41,0x41,0x22,0x00,// C    67
0x7F,0x41,0x41,0x41,0x3E,0x00,// D    68
0x7F,0x49,0x49,0x49,0x41,0x00,// E    69
0x7F,0x09,0x09,0x09,0x01,0x00,// F    70
0x3E,0x41,0x49,0x49,0x7A,0x00,// G    71
0x7F,0x08,0x08,0x08,0x7F,0x00,// H    72
0x00,0x41,0x7F,0x41,0x00,0x00,// I    73
0x20,0x40,0x41,0x3F,0x01,0x00,// J    74
0x7F,0x08,0x14,0x22,0x41,0x00,// K    75
0x7F,0x40,0x40,0x40,0x40,0x00,// L    76
0x7F,0x06,0x1C,0x06,0x7F,0x00,// M    77
0x7F,0x02,0x0C,0x30,0x7F,0x00,// N    78
0x3E,0x41,0x41,0x41,0x3E,0x00,// O    79
0x7F,0x09,0x09,0x09,0x06,0x00,// P    80
0x3E,0x41,0x51,0x21,0x5E,0x00,// Q    81
0x7F,0x09,0x19,0x29,0x46,0x00,// R    82
0x46,0x49,0x49,0x49,0x31,0x00,// S    83
0x01,0x01,0x7F,0x01,0x01,0x00,// T    84
0x3F,0x40,0x40,0x40,0x3F,0x00,// U    85
0x1F,0x20,0x40,0x20,0x1F,0x00,// V    86
0x3F,0x40,0x38,0x40,0x3F,0x00,// W    87
0x63,0x14,0x08,0x14,0x63,0x00,// X    88
0x07,0x08,0x70,0x08,0x07,0x00,// Y    89
0x61,0x51,0x49,0x45,0x43,0x00 // Z    90
};

/*I2C时序延时函数，适配32MHz系统时钟*/
static void oled_delay(void)
{
    volatile uint32_t i = 120;
    while (i--) {}
}

/*SCL引脚拉低（PA15，推挽输出，全程不关闭输出使能）*/
static void oled_scl_low(void)
{
    DL_GPIO_clearPins(GPIOA, OLED_OLED_SCL_PIN);
}

/*SCL引脚拉高（PA15，推挽输出强驱动高电平，无浮空）*/
static void oled_scl_high(void)
{
    DL_GPIO_setPins(GPIOA, OLED_OLED_SCL_PIN);
}

/*SDA引脚拉低（PA16，推挽输出）*/
static void oled_sda_low(void)
{
    DL_GPIO_clearPins(GPIOA, OLED_OLED_SDA_PIN);
}

/*SDA引脚拉高（PA16，推挽输出强驱动高电平）*/
static void oled_sda_high(void)
{
    DL_GPIO_setPins(GPIOA, OLED_OLED_SDA_PIN);
}

/**
 * @brief 读取SDA电平，用于获取从机应答ACK
 * @retval 1：高电平无应答；0：低电平正常应答
 * @note 发送时SDA为输出，读应答临时切换带上拉输入，读完切回输出
 */
static uint8_t oled_sda_read(void)
{
    uint8_t val;
    // 临时切换为带上拉输入，读取从机电平
    DL_GPIO_initDigitalInputFeatures(OLED_OLED_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    oled_delay();
    // DL_GPIO_readPins(端口, 引脚掩码) 两个参数缺一不可
    val = (DL_GPIO_readPins(GPIOA, OLED_OLED_SDA_PIN) != 0) ? 1 : 0;
    // 恢复为SysConfig配置的数字输出模式
    DL_GPIO_initDigitalOutput(OLED_OLED_SDA_IOMUX);
    return val;
}

/**
 * @brief 软件I2C引脚初始化
 * @note SysConfig已配置PA15/PA16为Digital Output，此处仅设置空闲高电平，不修改引脚模式
 */
static void oled_i2c_init_gpio(void)
{
    // 强制释放I2C总线，恢复空闲高电平
    oled_sda_high();
    oled_delay();
    oled_scl_high();
    // 加长等待，等待OLED总线恢复
    for(uint16_t t=0;t<5000;t++) oled_delay();
}

/**
 * @brief I2C起始信号：SDA高→低，SCL保持高
 */
static void oled_i2c_start(void)
{
    oled_sda_high();
    oled_scl_high();
    oled_delay();
    oled_sda_low();
    oled_delay();
    oled_scl_low();
    oled_delay();
}

/**
 * @brief I2C停止信号：SCL高时，SDA低→高
 */
static void oled_i2c_stop(void)
{
    oled_sda_low();
    oled_delay();
    oled_scl_high();
    oled_delay();
    oled_sda_high();
    oled_delay();
}

/**
 * @brief I2C发送单bit
 * @param bit 待发送位 0/1
 */
static void oled_i2c_write_bit(uint8_t bit)
{
    if (bit)
        oled_sda_high();
    else
        oled_sda_low();
    oled_delay();
    oled_scl_high();
    oled_delay();
    oled_scl_low();
    oled_delay();
}

/**
 * @brief 等待OLED从机应答信号
 * @retval 1：收到ACK应答；0：无应答通信异常
 */
static uint8_t oled_i2c_wait_ack(void)
{
    uint8_t ack;
    oled_sda_high();
    oled_delay();
    oled_scl_high();
    oled_delay();
    ack = (oled_sda_read() == 0);
    oled_scl_low();
    oled_delay();
    return ack;
}

/**
 * @brief I2C发送1字节数据，发送后自动等待ACK
 * @param byte 待发送单字节
 */
static void oled_i2c_send_byte(uint8_t byte)
{
    // 高位先发
    for (uint8_t i = 0; i < 8; i++)
    {
        oled_i2c_write_bit((byte & 0x80) ? 1 : 0);
        byte <<= 1;
    }
    oled_i2c_wait_ack();
}

/**
 * @brief I2C原始批量发送函数
 * @param buf 发送数据缓冲区
 * @param len 数据字节长度
 */
static void oled_write_raw(uint8_t *buf, uint16_t len)
{
    oled_i2c_start();
    // 设备写地址 (ADDR << 1 | 0 代表写操作)
    oled_i2c_send_byte((SSD1306_ADDR << 1) | 0x00);
    for (uint16_t i = 0; i < len; i++)
    {
        oled_i2c_send_byte(buf[i]);
    }
    oled_i2c_stop();
}

/**
 * @brief 向SSD1306发送寄存器配置命令
 * @param cmd OLED寄存器命令码
 * @retval 固定返回0，无错误处理
 */
static int OLED_WriteCommand(uint8_t cmd)
{
    uint8_t buf[2] = { SSD1306_CONTROL_CMD, cmd };
    oled_write_raw(buf, 2);
    return 0;
}

/**
 * @brief 向OLED显存批量写入点阵显示数据
 * @param data 点阵数据指针
 * @param len 数据长度
 * @retval 0 执行完成
 * @note 超过240字节自动分包发送，防止栈溢出
 */
static int OLED_WriteData(const uint8_t* data, int len)
{
	if (len <= 0) return 0;
	uint8_t tmp[256];
	// 数据较短一次性发送
	if (len + 1 <= (int)sizeof(tmp)) {
		tmp[0] = SSD1306_CONTROL_DATA;
		memcpy(&tmp[1], data, len);
		oled_write_raw(tmp, len + 1);
		return 0;
	}
	// 长数据分片发送
	int sent = 0;
	while (sent < len) {
		int chunk = (len - sent > 240) ? 240 : (len - sent);
		tmp[0] = SSD1306_CONTROL_DATA;
		memcpy(&tmp[1], &data[sent], chunk);
		oled_write_raw(tmp, chunk + 1);
		sent += chunk;
	}
	return 0;
}

/**
 * @brief OLED屏幕初始化完整流程
 * @note 标准SSD1306 128*64初始化序列，包含时钟、分辨率、充电泵、对比度、扫描方向配置
 * @warning 主函数必须先执行SYSCFG_DL_init()再调用OLED_Init()
 */
void OLED_Init(void)
{
    oled_i2c_init_gpio();
    // 新增：发送停止信号解锁总线
    oled_i2c_stop();
    oled_delay();

    OLED_WriteCommand(0xAE); // 关闭显示
    // 后面原有初始化命令不变
    OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14);
    OLED_WriteCommand(0x20); OLED_WriteCommand(0x00);
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB); OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xA6);
    OLED_Clear();
    OLED_WriteCommand(0xAF); // 开启显示
}


/**
 * @brief 全屏清屏，所有像素熄灭
 * @note 128*64屏幕分为8页，每页8行像素，逐页填充0
 */
void OLED_Clear(void)
{
	uint8_t zeros[128];
	memset(zeros, 0x00, sizeof(zeros));
	for (uint8_t page = 0; page < 8; page++) {
		OLED_WriteCommand(0xB0 | page); // 设置页地址
		OLED_WriteCommand(0x00);        // 列低4位
		OLED_WriteCommand(0x10);        // 列高4位
		OLED_WriteData(zeros, sizeof(zeros));
	}
}

/**
 * @brief 设置字符显示坐标
 * @param x 横向像素坐标 0~127
 * @param y 纵向页号 0~7（每页8像素高度）
 */
static void OLED_SetPos(uint8_t x, uint8_t y)
{
    OLED_WriteCommand(0xB0 + y);
    OLED_WriteCommand(((x & 0xF0) >> 4) | 0x10);
    OLED_WriteCommand(x & 0x0F);
}

/**
 * @brief 指定坐标打印ASCII字符串（英文/数字/符号）
 * @param x 横向起点
 * @param y 纵向页起点
 * @param s 字符串指针
 * @note 超出32~127的非法字符自动替换为空格
 */
void OLED_WriteString(uint8_t x, uint8_t y, const char* s)
{
    OLED_SetPos(x, y);
    while (*s)
    {
        uint8_t ch = *s;

        // 小写字母转大写，匹配现有大写字库
        if(ch >= 'a' && ch <= 'z')
        {
            ch -= 32;
        }

        // 超出可显示范围全部替换为空格
        if(ch < 32 || ch > 90)
            ch = 32;

        // 偏移取6字节字库
        const uint8_t *font = F6X8 + (ch - 32) * 6;
        OLED_WriteData(font, 6);
        s++;
    }
}

/**
 * @brief 次方计算辅助函数，用于数字打印
 */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t res = 1;
	while(Y--)
	{
		res *= X;
	}
	return res;
}

/**
 * @brief 指定坐标打印数字，可固定显示位数
 * @param x 横向起点
 * @param y 纵向页起点
 * @param num 要打印无符号数字
 * @param len 固定显示总长度，不足补空格
 */
void OLED_WriteNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len)
{
	char buf[16];
	uint8_t i = 0;
	if(num == 0) buf[i++] = '0';
	else
	{
		while(num)
		{
			buf[i++] = num % 10 + '0';
			num /= 10;
		}
	}
	// 长度不足末尾补空格
	while(i < len) buf[i++] = ' ';
	// 反转字符顺序
	uint8_t a = 0, b = i - 1;
	while(a < b)
	{
		char t = buf[a];
		buf[a] = buf[b];
		buf[b] = t;
		a++; b--;
	}
	buf[i] = '\0';
	OLED_WriteString(x, y, buf);
}

void OLED_WriteInt(uint8_t x, uint8_t y, int32_t num, uint8_t len)
{
	char buf[16];
	uint8_t i = 0;
	uint8_t neg = 0;
	uint32_t val;

	if(num < 0)
	{
		neg = 1;
		val = (uint32_t)(-(num + 1)) + 1U;
	}
	else
	{
		val = (uint32_t)num;
	}

	if(val == 0) buf[i++] = '0';
	else
	{
		while(val && i < (sizeof(buf) - 2))
		{
			buf[i++] = val % 10 + '0';
			val /= 10;
		}
	}

	if(neg && i < (sizeof(buf) - 1)) buf[i++] = '-';
	while(i < len && i < (sizeof(buf) - 1)) buf[i++] = ' ';

	uint8_t a = 0, b = i - 1;
	while(a < b)
	{
		char t = buf[a];
		buf[a] = buf[b];
		buf[b] = t;
		a++; b--;
	}
	buf[i] = '\0';
	OLED_WriteString(x, y, buf);
}
