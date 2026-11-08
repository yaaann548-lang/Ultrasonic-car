#ifndef CAR_BOARD_H
#define CAR_BOARD_H
#include "STC8H.H"
typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;
typedef signed char s8;
sbit OLED_SCL = P2^2;
sbit OLED_SDA = P2^3;
sbit US_TRIG = P3^4;
sbit US_ECHO = P3^5;
sbit MOTOR_AIN1 = P1^0;
sbit MOTOR_AIN2 = P1^1;
sbit MOTOR_BIN1 = P1^3;
sbit MOTOR_BIN2 = P1^4;
/* P2.0=PWM5->TB6612 PWMA; P2.1=PWM6->TB6612 PWMB.
   STBY is wired high on the PCB. No software LED owns P2.0 now. */
#endif
