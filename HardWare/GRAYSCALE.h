#ifndef GRAYSCALE_H
#define GRAYSCALE_H

#include <stdint.h>

void Grayscale_Init(void);
uint8_t Grayscale_ReadChannel(uint8_t ch);
uint16_t Grayscale_ReadAll(void);

#endif
