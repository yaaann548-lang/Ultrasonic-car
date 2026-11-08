#ifndef TIMEBASE_H
#define TIMEBASE_H
#include "board.h"
void Timebase_Init(void);
u16 Timebase_Now(void);
void Timebase_DelayMs(u16 ms);
void Timebase_ShortDelay(void);
#endif
