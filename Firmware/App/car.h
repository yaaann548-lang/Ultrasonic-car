#ifndef CAR_H
#define CAR_H
#include "board.h"
#define CAR_WAIT 0
#define CAR_STOP 1
#define CAR_FORWARD 2
#define CAR_REVERSE 3
#define CAR_FAULT 4
#define CAR_FAR 5
void Car_Init(void);
void Car_Task(void);
u16 Car_Distance(void);
u8 Car_Status(void);
u8 Car_State(void);
#endif
