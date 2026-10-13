#include "timebase.h"
#include "config.h"
static volatile u16 milliseconds;
void Timebase_Init(void)
{
    u16 reload;
    TR0 = 0;
    milliseconds = 0;
    AUXR |= 0x80; /* T0 1T */
    TMOD &= 0xF0; /* STC8H mode 0 = 16-bit auto reload */
    reload = (u16)(65536UL - (SYSCLK_HZ + 500UL) / 1000UL);
    TL0 = (u8)reload;
    TH0 = (u8)(reload >> 8);
    TF0 = 0;
    ET0 = 1;
    TR0 = 1;
}
void Timer0_ISR(void) interrupt 1 using 1
{
    milliseconds++;
}
u16 Timebase_Now(void)
{
    u16 now;
    bit saved_ea;
    saved_ea = EA;
    EA = 0;
    now = milliseconds;
    EA = saved_ea;
    return now;
}
void Timebase_DelayMs(u16 ms)
{
    u16 start;
    start = Timebase_Now();
    while ((u16)(Timebase_Now() - start) < ms) { }
}
/* At least 2 us at 11.0592 MHz, plus function-call overhead.
   Does not reconfigure T0/T1. Interrupts may lengthen it harmlessly. */
void Timebase_ShortDelay(void)
{
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
}
