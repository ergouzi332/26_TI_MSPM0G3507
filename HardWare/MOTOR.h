#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_SetSpeed(int16_t speed);   /* -1000 ~ +1000 */
void Motor_Stop(void);

#endif
