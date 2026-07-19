#include "uart3.h"

void UART3_Init(void)
{
    // UART3 ?? SYSCFG_DL_init() ???????
    // ???????FIFO??
    while (!DL_UART_isRXFIFOEmpty(UART_3_INST)) {
        DL_UART_receiveData(UART_3_INST);
    }
}

void UART3_SendByte(uint8_t data)
{
    // ??TX FIFO?????????
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

// ============================================================
// ???????????????????? 9600 8N1 ??
// ============================================================
void UART3_Test(void)
{
    UART3_SendString("UART3 OK\r\n");
    UART3_SendString("PB2=TX PB3=RX @9600\r\n");

    // ??0~9??????
    for (uint8_t i = 0; i < 10; i++) {
        UART3_SendByte('0' + i);
    }
    UART3_SendString("\r\nDone\r\n");
}


void UART3_PrintRPM(uint16_t pwm, uint16_t rpmL, uint16_t rpmR)
{
    UART3_SendString("PWM:");
    UART3_SendByte(48 + pwm / 100);
    UART3_SendByte(48 + (pwm / 10) % 10);
    UART3_SendByte(48 + (pwm % 10));
    UART3_SendByte(32);
    UART3_SendString("L:");
    UART3_SendByte(48 + rpmL / 100);
    UART3_SendByte(48 + (rpmL / 10) % 10);
    UART3_SendByte(48 + (rpmL % 10));
    UART3_SendByte(32);
    UART3_SendString("R:");
    UART3_SendByte(48 + rpmR / 100);
    UART3_SendByte(48 + (rpmR / 10) % 10);
    UART3_SendByte(48 + (rpmR % 10));
    UART3_SendString("\r\n");
}
