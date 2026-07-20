#ifndef __UART3_H
#define __UART3_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

void UART3_Init(void);
void UART3_SendByte(uint8_t data);
void UART3_SendString(const char *str);
void UART3_SendArray(const uint8_t *buf, uint16_t len);
uint8_t UART3_ReceiveByte(void);
uint8_t UART3_DataAvailable(void);
void UART3_Test(void);

void UART3_PrintRPM(uint16_t pwmL, uint16_t pwmR,
                    uint16_t rpmL, uint16_t rpmR);

#endif
