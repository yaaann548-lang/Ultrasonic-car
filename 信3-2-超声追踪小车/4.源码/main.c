/* Integrated follower test with temporary distance scale correction. */
#include "board.h"
#include "timebase.h"
#include "motor.h"
#include "ultrasonic.h"
#include "display.h"
#include "car.h"
#include "config.h"
#include "line.h"

void main(void)
{
    EA = 0;
    P_SW2 |= 0x80;
    //CLKDIV = 0;  //10.14×¢ÊÍ
    Timebase_Init();
    Motor_Init();
#if !LINE_FOLLOW_ENABLE
    Ultrasonic_Init();
#endif
    EA = 1;
    Timebase_DelayMs(300);
#if LINE_FOLLOW_ENABLE
    Line_Init();
    while (1) {
        Line_Task();
        Motor_Task();
    }
#else
    Display_Init();
    Car_Init();
    while (1) {
        Car_Task();
        Motor_Task();
        Display_Task();
    }
#endif
}
