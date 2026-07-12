#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_Stop(void);
void Motor_SetTargetRPM(int16_t rpm);
int16_t Motor_GetTargetRPM(void);
int16_t Motor_GetActualRPM(void);
void Motor_SpeedUpdate(void);

#endif
