#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
int32_t Encoder_GetPulse(void);
void Encoder_Reset(void);

#endif
