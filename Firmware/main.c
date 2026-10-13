/* Integrated follower test with temporary distance scale correction. */
#include "board.h"
#include "timebase.h"
#include "motor.h"
#include "ultrasonic.h"
#include "display.h"
#include "car.h"

void main(void)
{
    EA = 0;
    P_SW2 |= 0x80;
    CLKDIV = 0;
    Timebase_Init();
    Motor_Init();
    Ultrasonic_Init();
    EA = 1;
    Timebase_DelayMs(300);
    Display_Init();
    Car_Init();
    while (1) {
        Car_Task();
        Motor_Task();
        Display_Task();
    }
}
