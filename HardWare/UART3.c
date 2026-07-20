#include "uart3.h"

void UART3_Init(void)
{
    // 冲刷 RX FIFO
    while (!DL_UART_isRXFIFOEmpty(UART_3_INST)) {
        DL_UART_receiveData(UART_3_INST);
    }
    // 确保 TX 空闲
    while (DL_UART_isBusy(UART_3_INST));
}

// 阻塞发送一个字节（115200 下 ~87us/byte）
void UART3_SendByte(uint8_t data)
{
    while (DL_UART_isTXFIFOFull(UART_3_INST));
    DL_UART_transmitData(UART_3_INST, data);
}

void UART3_SendString(const char *str)
{
    while (*str) {
        UART3_SendByte((uint8_t)(*str++));
    }
}

void UART3_SendArray(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        UART3_SendByte(buf[i]);
    }
}

uint8_t UART3_DataAvailable(void)
{
    return !DL_UART_isRXFIFOEmpty(UART_3_INST);
}

uint8_t UART3_ReceiveByte(void)
{
    while (DL_UART_isRXFIFOEmpty(UART_3_INST));
    return DL_UART_receiveData(UART_3_INST);
}

void UART3_Test(void)
{
    UART3_SendString("UART3 OK\r\n");
    UART3_SendString("PB2=TX PB3=RX @115200\r\n");
    for (uint8_t i = 0; i < 10; i++) {
        UART3_SendByte('0' + i);
    }
    UART3_SendString("\r\nDone\r\n");
}

void UART3_PrintRPM(uint16_t pwmL, uint16_t pwmR,
                    uint16_t rpmL, uint16_t rpmR)
{
    // "PWM:xxx L:xxx R:xxx\r\n"  逐字节发送，不上 sprintf
    UART3_SendByte('P'); UART3_SendByte('W'); UART3_SendByte('M'); UART3_SendByte(':');
    UART3_SendByte('0' + (pwmL / 100) % 10);
    UART3_SendByte('0' + (pwmL / 10) % 10);
    UART3_SendByte('0' + (pwmL % 10));
    UART3_SendByte(' '); UART3_SendByte('L'); UART3_SendByte(':');
    UART3_SendByte('0' + (rpmL / 100) % 10);
    UART3_SendByte('0' + (rpmL / 10) % 10);
    UART3_SendByte('0' + (rpmL % 10));
    UART3_SendByte(' '); UART3_SendByte('R'); UART3_SendByte(':');
    UART3_SendByte('0' + (rpmR / 100) % 10);
    UART3_SendByte('0' + (rpmR / 10) % 10);
    UART3_SendByte('0' + (rpmR % 10));
    UART3_SendByte('\r'); UART3_SendByte('\n');
}
