#include "display.h"
#include "oled.h"
#include "car.h"
#include "motor.h"
#include "timebase.h"
#include "ultrasonic.h"
#include "config.h"
static u8 ready, row;
static u16 last_frame;
static void Number3(char *p, u16 n)
{
    p[0] = '0' + n / 100U;
    p[1] = '0' + (n / 10U) % 10U;
    p[2] = '0' + n % 10U;
}
void Display_Init(void)
{
    ready = 0; row = 0; last_frame = Timebase_Now();
#if OLED_ENABLE
    ready = OLED_Init();
#endif
}
void Display_Task(void)
{
    char line[20];
    u16 mm, now;
    s8 a, b;
    if (!ready) return;
    now = Timebase_Now();
    if (row == 0 && (u16)(now - last_frame) < DISPLAY_PERIOD_MS) return;
    if (row == 0) {
        last_frame = now;
        OLED_Text(0, "CAR TARGET 25CM");
    } else if (row == 1) {
        if (Car_Status() != US_OK) OLED_Text(2, "DIST ---.- CM");
        else {
            mm = Car_Distance();
            Number3(line, mm / 10U);
            line[3] = '.'; line[4] = '0' + mm % 10U;
            line[5] = ' '; line[6] = 'C'; line[7] = 'M'; line[8] = 0;
            OLED_Text(2, line);
        }
    } else if (row == 2) {
        if (Car_State() == CAR_WAIT) OLED_Text(4, "WAIT");
        else if (Car_State() == CAR_FORWARD) OLED_Text(4, "FORWARD");
        else if (Car_State() == CAR_REVERSE) OLED_Text(4, "REVERSE");
        else if (Car_State() == CAR_STOP) OLED_Text(4, "STOP");
        else if (Car_State() == CAR_FAR) OLED_Text(4, "TOO FAR STOP");
        else if (Car_Status() == US_NO_ECHO) OLED_Text(4, "NO ECHO STOP");
        else if (Car_Status() == US_HIGH) OLED_Text(4, "ECHO HIGH STOP");
        else OLED_Text(4, "RANGE STOP");
    } else {
        a = Motor_GetA(); b = Motor_GetB();
        line[0] = 'A'; line[1] = a < 0 ? '-' : ' ';
        Number3(line + 2, (u16)(a < 0 ? -a : a));
        line[5] = '%'; line[6] = ' '; line[7] = 'B';
        line[8] = b < 0 ? '-' : ' ';
        Number3(line + 9, (u16)(b < 0 ? -b : b));
        line[12] = '%'; line[13] = 0; OLED_Text(6, line);
    }
    ready = OLED_Ready();
    row = (row + 1) & 3; /* one row per main-loop pass */
}
