#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_Stop(void);
void Motor_SetPWM(uint16_t left, uint16_t right);
void Motor_SetForward(void);
void Motor_SetBrake(void);
uint32_t Motor_GetLeftPulses(void);
uint32_t Motor_GetRightPulses(void);

#endif
