/* OLED driver (SSD1306) using software I2C bit-bang on PA15/PA16
 * PA15 = SCL, PA16 = SDA
 * 鐢变簬鏉夸笂纭欢 I2C 绾胯矾寮傚父锛屾敼涓?GPIO 妯℃嫙 I2C 閫氫俊
 * 閫傞厤MSPM0G3507锛孲ysConfig宸查鍏堝皢PA15/PA16閰嶇疆涓篋igital Output
 * 淇鐐癸細鍒犻櫎娴┖杈撳嚭銆佽鍐橲DA鍔ㄦ€佸垏鎹㈣緭鍏ヨ緭鍑恒€佷笉瑕嗙洊SysConfig寮曡剼閰嶇疆
 */
#include "OLED.h"
#include "ti_msp_dl_config.h"
#include <string.h>

// SSD1306 OLED I2C璁惧鍦板潃锛屽鏁?.96瀵稿睆涓?x3C锛岄儴鍒嗗睆涓?x3D
#define SSD1306_ADDR 0x3C
// I2C閫氫俊鎺у埗瀛楄妭锛?0浠ｈ〃鍚庣画鍙戦€佸懡浠わ紝40浠ｈ〃鍚庣画鍙戦€佹樉瀛樻暟鎹?
#define SSD1306_CONTROL_CMD  0x00
#define SSD1306_CONTROL_DATA 0x40

/**
 * @brief 6x8 ASCII鏍囧噯鐐归樀瀛楀簱
 * 鍋忕Щ32瀵瑰簲绌烘牸寮€濮嬶紝姣忎釜瀛楃鍗犵敤6瀛楄妭绾靛悜鐐归樀锛屼粎鏀寔ASCII 32~127
 */
static const unsigned char F6X8[] = {
0x00,0x00,0x00,0x00,0x00,0x00,// 绌烘牸 32
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

/*I2C鏃跺簭寤舵椂鍑芥暟锛岄€傞厤32MHz绯荤粺鏃堕挓*/
static void oled_delay(void)
{
    volatile uint32_t i = 120;
    while (i--) {}
}

/*SCL寮曡剼鎷変綆锛圥A15锛屾帹鎸借緭鍑猴紝鍏ㄧ▼涓嶅叧闂緭鍑轰娇鑳斤級*/
static void oled_scl_low(void)
{
    DL_GPIO_clearPins(GPIOA, OLED_OLED_SCL_PIN);
}

/*SCL寮曡剼鎷夐珮锛圥A15锛屾帹鎸借緭鍑哄己椹卞姩楂樼數骞筹紝鏃犳诞绌猴級*/
static void oled_scl_high(void)
{
    DL_GPIO_setPins(GPIOA, OLED_OLED_SCL_PIN);
}

/*SDA寮曡剼鎷変綆锛圥A16锛屾帹鎸借緭鍑猴級*/
static void oled_sda_low(void)
{
    DL_GPIO_clearPins(GPIOA, OLED_OLED_SDA_PIN);
}

/*SDA寮曡剼鎷夐珮锛圥A16锛屾帹鎸借緭鍑哄己椹卞姩楂樼數骞筹級*/
static void oled_sda_high(void)
{
    DL_GPIO_setPins(GPIOA, OLED_OLED_SDA_PIN);
}

/**
 * @brief 璇诲彇SDA鐢靛钩锛岀敤浜庤幏鍙栦粠鏈哄簲绛擜CK
 * @retval 1锛氶珮鐢靛钩鏃犲簲绛旓紱0锛氫綆鐢靛钩姝ｅ父搴旂瓟
 * @note 鍙戦€佹椂SDA涓鸿緭鍑猴紝璇诲簲绛斾复鏃跺垏鎹㈠甫涓婃媺杈撳叆锛岃瀹屽垏鍥炶緭鍑?
 */
static uint8_t oled_sda_read(void)
{
    uint8_t val;
    // 涓存椂鍒囨崲涓哄甫涓婃媺杈撳叆锛岃鍙栦粠鏈虹數骞?
    DL_GPIO_initDigitalInputFeatures(OLED_OLED_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    oled_delay();
    // DL_GPIO_readPins(绔彛, 寮曡剼鎺╃爜) 涓や釜鍙傛暟缂轰竴涓嶅彲
    val = (DL_GPIO_readPins(GPIOA, OLED_OLED_SDA_PIN) != 0) ? 1 : 0;
    // 鎭㈠涓篠ysConfig閰嶇疆鐨勬暟瀛楄緭鍑烘ā寮?
    DL_GPIO_initDigitalOutput(OLED_OLED_SDA_IOMUX);
    return val;
}

/**
 * @brief 杞欢I2C寮曡剼鍒濆鍖?
 * @note SysConfig宸查厤缃甈A15/PA16涓篋igital Output锛屾澶勪粎璁剧疆绌洪棽楂樼數骞筹紝涓嶄慨鏀瑰紩鑴氭ā寮?
 */
static void oled_i2c_init_gpio(void)
{
    // 寮哄埗閲婃斁I2C鎬荤嚎锛屾仮澶嶇┖闂查珮鐢靛钩
    oled_sda_high();
    oled_delay();
    oled_scl_high();
    // 鍔犻暱绛夊緟锛岀瓑寰匫LED鎬荤嚎鎭㈠
    for(uint16_t t=0;t<5000;t++) oled_delay();
}

/**
 * @brief I2C璧峰淇″彿锛歋DA楂樷啋浣庯紝SCL淇濇寔楂?
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
 * @brief I2C鍋滄淇″彿锛歋CL楂樻椂锛孲DA浣庘啋楂?
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
 * @brief I2C鍙戦€佸崟bit
 * @param bit 寰呭彂閫佷綅 0/1
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
 * @brief 绛夊緟OLED浠庢満搴旂瓟淇″彿
 * @retval 1锛氭敹鍒癆CK搴旂瓟锛?锛氭棤搴旂瓟閫氫俊寮傚父
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
 * @brief I2C鍙戦€?瀛楄妭鏁版嵁锛屽彂閫佸悗鑷姩绛夊緟ACK
 * @param byte 寰呭彂閫佸崟瀛楄妭
 */
static void oled_i2c_send_byte(uint8_t byte)
{
    // 楂樹綅鍏堝彂
    for (uint8_t i = 0; i < 8; i++)
    {
        oled_i2c_write_bit((byte & 0x80) ? 1 : 0);
        byte <<= 1;
    }
    oled_i2c_wait_ack();
}

/**
 * @brief I2C鍘熷鎵归噺鍙戦€佸嚱鏁?
 * @param buf 鍙戦€佹暟鎹紦鍐插尯
 * @param len 鏁版嵁瀛楄妭闀垮害
 */
static void oled_write_raw(uint8_t *buf, uint16_t len)
{
    oled_i2c_start();
    // 璁惧鍐欏湴鍧€ (ADDR << 1 | 0 浠ｈ〃鍐欐搷浣?
    oled_i2c_send_byte((SSD1306_ADDR << 1) | 0x00);
    for (uint16_t i = 0; i < len; i++)
    {
        oled_i2c_send_byte(buf[i]);
    }
    oled_i2c_stop();
}

/**
 * @brief 鍚慡SD1306鍙戦€佸瘎瀛樺櫒閰嶇疆鍛戒护
 * @param cmd OLED瀵勫瓨鍣ㄥ懡浠ょ爜
 * @retval 鍥哄畾杩斿洖0锛屾棤閿欒澶勭悊
 */
static int OLED_WriteCommand(uint8_t cmd)
{
    uint8_t buf[2] = { SSD1306_CONTROL_CMD, cmd };
    oled_write_raw(buf, 2);
    return 0;
}

/**
 * @brief 鍚慜LED鏄惧瓨鎵归噺鍐欏叆鐐归樀鏄剧ず鏁版嵁
 * @param data 鐐归樀鏁版嵁鎸囬拡
 * @param len 鏁版嵁闀垮害
 * @retval 0 鎵ц瀹屾垚
 * @note 瓒呰繃240瀛楄妭鑷姩鍒嗗寘鍙戦€侊紝闃叉鏍堟孩鍑?
 */
static int OLED_WriteData(const uint8_t* data, int len)
{
	if (len <= 0) return 0;
	static uint8_t tmp[256];
	// 鏁版嵁杈冪煭涓€娆℃€у彂閫?
	if (len + 1 <= (int)sizeof(tmp)) {
		tmp[0] = SSD1306_CONTROL_DATA;
		memcpy(&tmp[1], data, len);
		oled_write_raw(tmp, len + 1);
		return 0;
	}
	// 闀挎暟鎹垎鐗囧彂閫?
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
 * @brief OLED灞忓箷鍒濆鍖栧畬鏁存祦绋?
 * @note 鏍囧噯SSD1306 128*64鍒濆鍖栧簭鍒楋紝鍖呭惈鏃堕挓銆佸垎杈ㄧ巼銆佸厖鐢垫车銆佸姣斿害銆佹壂鎻忔柟鍚戦厤缃?
 * @warning 涓诲嚱鏁板繀椤诲厛鎵цSYSCFG_DL_init()鍐嶈皟鐢∣LED_Init()
 */
void OLED_Init(void)
{
    oled_i2c_init_gpio();
    // 鏂板锛氬彂閫佸仠姝俊鍙疯В閿佹€荤嚎
    oled_i2c_stop();
    oled_delay();

    OLED_WriteCommand(0xAE); // 鍏抽棴鏄剧ず
    // 鍚庨潰鍘熸湁鍒濆鍖栧懡浠や笉鍙?
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
    OLED_WriteCommand(0xAF); // 寮€鍚樉绀?
}


/**
 * @brief 鍏ㄥ睆娓呭睆锛屾墍鏈夊儚绱犵唲鐏?
 * @note 128*64灞忓箷鍒嗕负8椤碉紝姣忛〉8琛屽儚绱狅紝閫愰〉濉厖0
 */
void OLED_Clear(void)
{
	static const uint8_t zeros[128] = {0};
	
	for (uint8_t page = 0; page < 8; page++) {
		OLED_WriteCommand(0xB0 | page); // 璁剧疆椤靛湴鍧€
		OLED_WriteCommand(0x00);        // 鍒椾綆4浣?
		OLED_WriteCommand(0x10);        // 鍒楅珮4浣?
		OLED_WriteData(zeros, sizeof(zeros));
	}
}

/**
 * @brief 璁剧疆瀛楃鏄剧ず鍧愭爣
 * @param x 妯悜鍍忕礌鍧愭爣 0~127
 * @param y 绾靛悜椤靛彿 0~7锛堟瘡椤?鍍忕礌楂樺害锛?
 */
static void OLED_SetPos(uint8_t x, uint8_t y)
{
    OLED_WriteCommand(0xB0 + y);
    OLED_WriteCommand(((x & 0xF0) >> 4) | 0x10);
    OLED_WriteCommand(x & 0x0F);
}

/**
 * @brief 鎸囧畾鍧愭爣鎵撳嵃ASCII瀛楃涓诧紙鑻辨枃/鏁板瓧/绗﹀彿锛?
 * @param x 妯悜璧风偣
 * @param y 绾靛悜椤佃捣鐐?
 * @param s 瀛楃涓叉寚閽?
 * @note 瓒呭嚭32~127鐨勯潪娉曞瓧绗﹁嚜鍔ㄦ浛鎹负绌烘牸
 */
void OLED_WriteString(uint8_t x, uint8_t y, const char* s)
{
    OLED_SetPos(x, y);
    while (*s)
    {
        uint8_t ch = *s;

        // 灏忓啓瀛楁瘝杞ぇ鍐欙紝鍖归厤鐜版湁澶у啓瀛楀簱
        if(ch >= 'a' && ch <= 'z')
        {
            ch -= 32;
        }

        // 瓒呭嚭鍙樉绀鸿寖鍥村叏閮ㄦ浛鎹负绌烘牸
        if(ch < 32 || ch > 90)
            ch = 32;

        // 鍋忕Щ鍙?瀛楄妭瀛楀簱
        const uint8_t *font = F6X8 + (ch - 32) * 6;
        OLED_WriteData(font, 6);
        s++;
    }
}

/**
 * @brief 娆℃柟璁＄畻杈呭姪鍑芥暟锛岀敤浜庢暟瀛楁墦鍗?
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
 * @brief 鎸囧畾鍧愭爣鎵撳嵃鏁板瓧锛屽彲鍥哄畾鏄剧ず浣嶆暟
 * @param x 妯悜璧风偣
 * @param y 绾靛悜椤佃捣鐐?
 * @param num 瑕佹墦鍗版棤绗﹀彿鏁板瓧
 * @param len 鍥哄畾鏄剧ず鎬婚暱搴︼紝涓嶈冻琛ョ┖鏍?
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
	// 闀垮害涓嶈冻鏈熬琛ョ┖鏍?
	while(i < len) buf[i++] = ' ';
	// 鍙嶈浆瀛楃椤哄簭
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


