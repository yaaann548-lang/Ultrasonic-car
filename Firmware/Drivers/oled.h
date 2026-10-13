#ifndef OLED_H
#define OLED_H
#include "board.h"
u8 OLED_Init(void);
void OLED_Text(u8 page, char *text);
u8 OLED_Ready(void);
#endif
