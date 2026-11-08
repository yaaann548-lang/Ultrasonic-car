#ifndef MOTOR_H
#define MOTOR_H
#include "board.h"
void Motor_Init(void);
void Motor_Stop(void);
void Motor_SetSpeed(s8 a, s8 b);
void Motor_Task(void);
s8 Motor_GetA(void);
s8 Motor_GetB(void);
#endif
