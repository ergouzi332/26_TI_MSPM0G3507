#include "uart3.h"

/**
 * @brief UART3初始化预处理
 * 硬件波特率、引脚、数据位等在SYSCFG图形工具配置完成
 * 此处仅清空接收缓存、等待发送总线空闲，避免上电残留数据干扰
 */
void UART3_Init(void)
{
    // 循环读取清空RX接收FIFO，冲刷上电残留杂波
    while (!DL_UART_isRXFIFOEmpty(UART_3_INST)) {
        DL_UART_receiveData(UART_3_INST);
    }
    // 等待上一轮发送全部传输完成，TX总线空闲
    while (DL_UART_isBusy(UART_3_INST));
}

/**
 * @brief 阻塞式发送单个字节
 * @param data 待发送单字节数据
 * 115200波特率下单字节传输约87us，FIFO满则循环等待
 */
void UART3_SendByte(uint8_t data)
{
    // 发送FIFO已满时阻塞等待有空位
    while (DL_UART_isTXFIFOFull(UART_3_INST));
    // 写入数据到发送FIFO，硬件自动发送
    DL_UART_transmitData(UART_3_INST, data);
}

/**
 * @brief 发送完整字符串（以'\0'结尾）
 * @param str 字符串指针
 */
void UART3_SendString(const char *str)
{
    // 遍历直到字符串结束符
    while (*str) {
        UART3_SendByte((uint8_t)(*str++));
    }
}

/**
 * @brief 发送指定长度字节数组
 * @param buf 待发送缓冲区首地址
 * @param len 发送字节总长度
 */
void UART3_SendArray(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        UART3_SendByte(buf[i]);
    }
}

/**
 * @brief 查询是否存在接收数据
 * @return 1=RX FIFO有数据可读；0=无接收数据
 */
uint8_t UART3_DataAvailable(void)
{
    return !DL_UART_isRXFIFOEmpty(UART_3_INST);
}

/**
 * @brief 阻塞读取单个接收字节
 * @return 读到的串口字节
 * 无数据时循环等待，直到收到数据再返回
 */
uint8_t UART3_ReceiveByte(void)
{
    while (DL_UART_isRXFIFOEmpty(UART_3_INST));
    return DL_UART_receiveData(UART_3_INST);
}

/**
 * @brief 串口自检测试函数
 * 上电调用，打印串口参数+数字测试串，验证收发硬件正常
 */
void UART3_Test(void)
{
    UART3_SendString("UART3 OK\r\n");
    UART3_SendString("PB2=TX PB3=RX @115200\r\n");
    // 发送0~9数字
    for (uint8_t i = 0; i < 10; i++) {
        UART3_SendByte('0' + i);
    }
    UART3_SendString("\r\nDone\r\n");
}

/**
 * @brief 无sprintf数字打印，逐字节输出PWM与转速
 * @param pwmL 左轮PWM值
 * @param pwmL 右轮PWM值
 * @param rpmL 左轮转速
 * @param rpmR 右轮转速
 * 手动拆分百位、十位、个位转字符发送，节省浮点/格式化库占用
 */
void UART3_PrintRPM(uint16_t pwmL, uint16_t pwmR,
                    uint16_t rpmL, uint16_t rpmR)
{
    // 打印 PWM:xxx
    UART3_SendByte('P'); UART3_SendByte('W'); UART3_SendByte('M'); UART3_SendByte(':');
    UART3_SendByte('0' + (pwmL / 100) % 10);
    UART3_SendByte('0' + (pwmL / 10) % 10);
    UART3_SendByte('0' + (pwmL % 10));
    // 打印 L:xxx
    UART3_SendByte(' '); UART3_SendByte('L'); UART3_SendByte(':');
    UART3_SendByte('0' + (rpmL / 100) % 10);
    UART3_SendByte('0' + (rpmL / 10) % 10);
    UART3_SendByte('0' + (rpmL % 10));
    // 打印 R:xxx
    UART3_SendByte(' '); UART3_SendByte('R'); UART3_SendByte(':');
    UART3_SendByte('0' + (rpmR / 100) % 10);
    UART3_SendByte('0' + (rpmR / 10) % 10);
    UART3_SendByte('0' + (rpmR % 10));
    // 换行符
    UART3_SendByte('\r'); UART3_SendByte('\n');
}