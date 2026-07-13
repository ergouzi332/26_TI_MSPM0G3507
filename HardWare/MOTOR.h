#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_Stop(void);
uint32_t Motor_GetPulses(void);

#endif
