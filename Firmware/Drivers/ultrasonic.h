#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include "board.h"
#define US_OK       0
#define US_NO_ECHO  1
#define US_HIGH     2
#define US_RANGE    3
void Ultrasonic_Init(void);
/* Bounded polling; T0 interrupts and hardware PWM remain running.
   mm is set to zero on failure. Call no faster than SAMPLE_PERIOD_MS. */
u8 Ultrasonic_Measure(u16 *mm);
#endif
