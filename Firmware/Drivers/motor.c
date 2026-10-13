#include "motor.h"
#include "timebase.h"
#include "config.h"
static s8 target_a, target_b, actual_a, actual_b;
static u16 last_step, reverse_start;
static u8 reversing;
static void Duty(u8 a, u8 b)
{
    u16 ca, cb;
    ca = (u16)((u32)MOTOR_PWM_COUNTS * a / 100UL);
    cb = (u16)((u32)MOTOR_PWM_COUNTS * b / 100UL);
    PWMB_CR1 |= 0x02; /* defer update while writing both channels */
    PWMB_CCR5H = (u8)(ca >> 8); PWMB_CCR5L = (u8)ca;
    PWMB_CCR6H = (u8)(cb >> 8); PWMB_CCR6L = (u8)cb;
    PWMB_CR1 &= 0xFD;
}
static void Outputs(void)
{
    s8 a, b;
    a = actual_a; b = actual_b;
#if MOTOR_A_INVERT
    a = -a;
#endif
#if MOTOR_B_INVERT
    b = -b;
#endif
    MOTOR_AIN1 = a > 0; MOTOR_AIN2 = a < 0;
    MOTOR_BIN1 = b > 0; MOTOR_BIN2 = b < 0;
    Duty((u8)(a < 0 ? -a : a), (u8)(b < 0 ? -b : b));
}
void Motor_Init(void)
{
    u16 arr;
    P_SW2 |= 0x80;
    PWMB_ENO = 0; PWMB_CR1 = 0; PWMB_BKR = 0;
    PWMA_ENO = 0; /* avoid alternate PWMA pin ownership */
    P2 &= 0xFC; P2M1 &= 0xFC; P2M0 |= 0x03;
    P1 &= 0xE4; P1M1 &= 0xE4; P1M0 |= 0x1B;
    PWMB_PS &= 0xF0;
    PWMB_CR2 = 0; PWMB_SMCR = 0; PWMB_ETR = 0;
    PWMB_IER = 0; PWMB_CCER1 = 0; PWMB_CCER2 = 0;
    PWMB_PSCRH = 0; PWMB_PSCRL = 0;
    arr = MOTOR_PWM_COUNTS - 1;
    PWMB_ARRH = (u8)(arr >> 8); PWMB_ARRL = (u8)arr;
    PWMB_RCR = 0; PWMB_CNTRH = 0; PWMB_CNTRL = 0;
    PWMB_CCMR1 = 0x68; PWMB_CCMR2 = 0x68;
    PWMB_CCR5H = 0; PWMB_CCR5L = 0;
    PWMB_CCR6H = 0; PWMB_CCR6L = 0;
    PWMB_CR1 = 0x80; PWMB_EGR = 1;
    PWMB_SR1 = 0; PWMB_SR2 = 0;
    PWMB_CCER1 = 0x11;
    PWMB_ENO = 0x05; PWMB_BKR = 0x80; PWMB_CR1 = 0x81;
    target_a = 0; target_b = 0; actual_a = 0; actual_b = 0;
    reversing = 0; last_step = Timebase_Now();
}
void Motor_Stop(void)
{
    Duty(0, 0);
    PWMB_EGR = 1; /* immediately load zero duty */
    MOTOR_AIN1 = 0; MOTOR_AIN2 = 0;
    MOTOR_BIN1 = 0; MOTOR_BIN2 = 0;
    target_a = 0; target_b = 0; actual_a = 0; actual_b = 0;
    /* Preserve a stop interval even when next command reverses direction. */
    reversing = 1; reverse_start = Timebase_Now();
}
void Motor_SetSpeed(s8 a, s8 b)
{
#if MOTOR_ENABLE
    if (a > 100) a = 100;
    if (a < -100) a = -100;
    if (b > 100) b = 100;
    if (b < -100) b = -100;
    target_a = a; target_b = b;
#else
    target_a = 0; target_b = 0;
#endif
}
static s8 Ramp(s8 current, s8 target)
{
    int next;
    next = current;
    if (current < target) {
        next += MOTOR_STEP_PERCENT;
        if (next > target) next = target;
    } else if (current > target) {
        next -= MOTOR_STEP_PERCENT;
        if (next < target) next = target;
    }
    return (s8)next;
}
void Motor_Task(void)
{
    u16 now;
    now = Timebase_Now();
    if (reversing) {
        if ((u16)(now - reverse_start) < MOTOR_REVERSE_WAIT_MS) return;
        reversing = 0;
    }
    if ((actual_a > 0 && target_a < 0) || (actual_a < 0 && target_a > 0)
     || (actual_b > 0 && target_b < 0) || (actual_b < 0 && target_b > 0)) {
        Duty(0, 0); PWMB_EGR = 1;
        actual_a = 0; actual_b = 0; Outputs();
        reversing = 1; reverse_start = now; return;
    }
    if ((u16)(now - last_step) < MOTOR_STEP_MS) return;
    last_step = now;
    actual_a = Ramp(actual_a, target_a);
    actual_b = Ramp(actual_b, target_b);
    Outputs();
}
s8 Motor_GetA(void) { return actual_a; }
s8 Motor_GetB(void) { return actual_b; }
