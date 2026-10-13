#include "ultrasonic.h"
#include "timebase.h"
#include "config.h"
#define T1_TICKS_MS(ms) ((u16)((SYSCLK_HZ / 12UL) * (ms) / 1000UL))
static void T1Start(void)
{
    TR1 = 0; TH1 = 0; TL1 = 0; TF1 = 0; TR1 = 1;
}
static u16 T1Read(void)
{
    u8 h1, h2, l;
    do { h1 = TH1; l = TL1; h2 = TH1; } while (h1 != h2);
    return ((u16)h1 << 8) | l;
}
void Ultrasonic_Init(void)
{
    TR1 = 0; ET1 = 0;
    AUXR &= 0xBF; /* T1 12T */
    TMOD = (TMOD & 0x0F) | 0x10;
    US_TRIG = 0; US_ECHO = 1;
    P3M1 = (P3M1 & 0xEF) | 0x20;
    P3M0 = (P3M0 | 0x10) & 0xDF;
}
u8 Ultrasonic_Measure(u16 *mm)
{
    u16 ticks, value;
    u8 i;
    *mm = 0;
    /* A stuck-high signal is invalid; do not wait or retrigger it. */
    if (US_ECHO) return US_HIGH;
    US_TRIG = 0;
    Timebase_ShortDelay();
    US_TRIG = 1;
    for (i = 0; i < 10; i++) Timebase_ShortDelay();
    US_TRIG = 0;
    T1Start();
    while (!US_ECHO) {
        if (TF1 || T1Read() >= T1_TICKS_MS(60UL)) {
            TR1 = 0; return US_NO_ECHO;
        }
    }
    T1Start();
    while (US_ECHO) {
        if (TF1 || T1Read() >= T1_TICKS_MS(40UL)) {
            TR1 = 0; return US_HIGH;
        }
    }
    TR1 = 0;
    ticks = T1Read();
    /* Nominal conversion for 11.0592 MHz and Timer1 12T. */
    value = (u16)(((u32)ticks * 3430UL + 9216UL) / 18432UL);
    if (value < 20U || value > 4000U) return US_RANGE;
    /* Keep raw range/timeout checks; correct both display and control. */
    value /= US_DISTANCE_DIVISOR;
    if (value < 20U) return US_RANGE;
    *mm = value;
    return US_OK;
}
