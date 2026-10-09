/* STC8H8K64U bench test ONLY, Keil C51.
 * SYSCLK = 11.0592 MHz, no extra system divider.
 * Replace existing main.c; do not add a second main.c.
 * Timer0/Timer1 are exclusively owned here; no interrupts used.
 * OLED init follows HS96L03W2C03 manufacturer datasheet, section 4.2.
 * SSD1306-compatible command set; marketplace lists SSD1315.
 * Enable ONLY after 3.3V OLED supply and 5V/3.3V I2C level conversion
 * are connected. Do not connect this OLED directly to 5V signals.
 */
#include <REG51.H>

#define OLED_ENABLE 1//1??oled

sfr AUXR = 0x8E;
sfr P2M1 = 0x95;
sfr P2M0 = 0x96;
sfr P3M1 = 0xB1;
sfr P3M0 = 0xB2;

sbit LED = P2^0;
sbit OLED_SCL = P2^2;
sbit OLED_SDA = P2^3;
sbit US_TRIG = P3^4;
sbit US_ECHO = P3^5;

typedef unsigned char u8;
typedef unsigned int u16;

#define US_OK       0
#define US_NO_ECHO  1
#define US_HIGH     2
#define US_RANGE    3

/* Observable in a debugger; not a USB telemetry implementation. */
volatile u8 test_status = US_NO_ECHO;
volatile u16 test_distance_mm = 0;
volatile u8 oled_ok = 0;

/* Timer tick = 12 / 11059200 s. Delay includes software overhead. */
static void DelayTicks(u16 ticks)
{
    u16 reload_value;
    TR0 = 0;
    reload_value = (u16)(0UL - ticks);
    TH0 = (u8)(reload_value >> 8);
    TL0 = (u8)reload_value;
    TF0 = 0;
    TR0 = 1;
    while (!TF0) { }
    TR0 = 0;
    TF0 = 0;
}

static void Delay10ms(u8 count)
{
    while (count != 0) {
        DelayTicks(9216);
        count--;
    }
}

static void T1Start(void)
{
    TR1 = 0;
    TH1 = 0;
    TL1 = 0;
    TF1 = 0;
    TR1 = 1;
}

static u16 T1Read(void)
{
    u8 high1, high2, low_byte;
    do {
        high1 = TH1;
        low_byte = TL1;
        high2 = TH1;
    } while (high1 != high2);
    return ((u16)high1 << 8) | low_byte;
}

static u8 Measure(void)
{
    u16 ticks;
    test_distance_mm = 0; /* valid only when test_status == US_OK */

    T1Start();
    while (US_ECHO) {
        if (TF1 || T1Read() >= 36864U) {
            TR1 = 0;
            return US_HIGH;
        }
    }
    TR1 = 0;

    US_TRIG = 0;
    DelayTicks(10);
    US_TRIG = 1;
    DelayTicks(20); /* >= 20 us, including call overhead */
    US_TRIG = 0;

    T1Start();
    while (!US_ECHO) {
        if (TF1 || T1Read() >= 55296U) { /* 60 ms */
            TR1 = 0;
            return US_NO_ECHO;
        }
    }

    T1Start();
    while (US_ECHO) {
        if (TF1 || T1Read() >= 36864U) { /* 40 ms */
            TR1 = 0;
            return US_HIGH;
        }
    }
    TR1 = 0;
    ticks = T1Read();
    /* 343 m/s at approximately room temperature; rounded mm. */
    test_distance_mm = (u16)(((unsigned long)ticks * 3430UL
                             + 9216UL) / 18432UL);
    if (test_distance_mm < 20U || test_distance_mm > 4000U) {
        return US_RANGE;
    }
    return US_OK;
}

#if OLED_ENABLE
static u8 oled_address;
static u8 bus_ok;

static void I2CDelay(void) { DelayTicks(5); }

static u8 SCLHigh(void)
{
    u8 count;
    OLED_SCL = 1; /* release: open-drain, never push high */
    for (count = 0; count < 100; count++) {
        if (OLED_SCL) { I2CDelay(); return 1; }
        I2CDelay();
    }
    return 0;
}

static void I2CStop(void)
{
    OLED_SCL = 0;
    OLED_SDA = 0;
    I2CDelay();
    SCLHigh();
    OLED_SDA = 1;
    I2CDelay();
}

static u8 I2CStart(void)
{
    OLED_SDA = 1;
    if (!SCLHigh() || !OLED_SDA) return 0;
    OLED_SDA = 0;
    I2CDelay();
    OLED_SCL = 0;
    return 1;
}

static u8 I2CWrite(u8 value)
{
    u8 i, ack;
    for (i = 0; i < 8; i++) {
        OLED_SCL = 0;
        OLED_SDA = (value & 0x80) ? 1 : 0;
        I2CDelay();
        if (!SCLHigh()) return 0;
        OLED_SCL = 0;
        value <<= 1;
    }
    OLED_SDA = 1;
    I2CDelay();
    if (!SCLHigh()) return 0;
    ack = OLED_SDA ? 0 : 1;
    OLED_SCL = 0;
    return ack;
}

static void Begin(u8 control_byte)
{
    if (!bus_ok) return;
    if (!I2CStart() || !I2CWrite(oled_address << 1)
        || !I2CWrite(control_byte)) {
        bus_ok = 0;
        I2CStop();
    }
}

static void Send(u8 value)
{
    if (bus_ok && !I2CWrite(value)) {
        bus_ok = 0;
        I2CStop();
    }
}

static void Position(u8 page)
{
    Begin(0x00);
    Send(0xB0 | page);
    Send(0x00);
    Send(0x10);
    I2CStop();
}

/* Minimal original 5x7 glyph set, columns, bit0 at top. */
static const char code glyph_names[] = "0123456789ACDEGHIMNORST .:-";
static const u8 code glyphs[][5] = {
 {0x3E,0x51,0x49,0x45,0x3E},{0,0x42,0x7F,0x40,0},
 {0x42,0x61,0x51,0x49,0x46},{0x22,0x41,0x49,0x49,0x36},
 {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
 {0x3C,0x4A,0x49,0x49,0x30},{1,0x71,9,5,3},
 {0x36,0x49,0x49,0x49,0x36},{6,0x49,0x49,0x29,0x1E},
 {0x7E,9,9,9,0x7E},{0x3E,0x41,0x41,0x41,0x22},
 {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},
 {0x3E,0x41,0x49,0x49,0x3A},{0x7F,8,8,8,0x7F},
 {0,0x41,0x7F,0x41,0},{0x7F,2,0x0C,2,0x7F},
 {0x7F,4,8,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
 {0x7F,9,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
 {1,1,0x7F,1,1},{0,0,0,0,0},{0,0x60,0x60,0,0},
 {0,0x36,0x36,0,0},{8,8,8,8,8}
};

static void Text(u8 page, char *text)
{
    u8 i, j, columns;
    Position(page);
    Begin(0x40);
    columns = 0;
    while (*text && columns < 120) {
        for (i = 0; glyph_names[i] && glyph_names[i] != *text; i++) { }
        if (!glyph_names[i]) i = 23; /* blank */
        for (j = 0; j < 5; j++) Send(glyphs[i][j]);
        Send(0);
        columns += 6;
        text++;
    }
    while (columns < 128) { Send(0); columns++; }
    I2CStop();
}

static const u8 code init_cmd[] = {
 0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,
 0x8D,0x14,0x20,0x02,0xA1,0xC8,0xDA,0x12,
 0x81,0x3F,0xD9,0xF1,0xDB,0x30,0xA4,0xA6,0x2E
};

static u8 OLEDInit(void)
{
    u8 i, page;
    OLED_SCL = 1;
    OLED_SDA = 1;
    P2M1 |= 0x0C;
    P2M0 |= 0x0C; /* open drain */
    Delay10ms(20);
    /* Bus recovery, bounded to 9 clocks. */
    for (i = 0; i < 9; i++) {
        OLED_SCL = 0;
        I2CDelay();
        if (!SCLHigh()) { I2CStop(); return 0; }
    }
    I2CStop();
    for (oled_address = 0x3C; oled_address <= 0x3D; oled_address++) {
        bus_ok = 1;
        Begin(0x00);
        I2CStop();
        if (bus_ok) break;
    }
    if (!bus_ok) return 0;
    Begin(0x00);
    for (i = 0; i < sizeof(init_cmd); i++) Send(init_cmd[i]);
    I2CStop();
    for (page = 0; page < 8; page++) Text(page, "");
    Begin(0x00);
    Send(0xAF);
    I2CStop();
    return bus_ok;
}

static void Display(void)
{
    char line[16];
    u16 mm;
    if (!oled_ok) return;
    Text(0, "DISTANCE");
    if (test_status == US_OK) {
        mm = test_distance_mm;
        line[0] = (char)('0' + mm / 1000U);
        line[1] = (char)('0' + (mm / 100U) % 10U);
        line[2] = (char)('0' + (mm / 10U) % 10U);
        line[3] = '.';
        line[4] = (char)('0' + mm % 10U);
        line[5] = ' '; line[6] = 'C'; line[7] = 'M'; line[8] = 0;
        Text(2, line);
        Text(4, "");
    } else {
        Text(2, "---.- CM");
        if (test_status == US_NO_ECHO) Text(4, "NO ECHO");
        else if (test_status == US_HIGH) Text(4, "ECHO HIGH");
        else Text(4, "RANGE");
    }
    oled_ok = bus_ok;
}
#endif

void main(void)
{
    EA = 0;
    TR0 = 0;
    TR1 = 0;
    AUXR &= 0x3F; /* both timers /12 */
    TMOD = 0x11;  /* timers 0 and 1, mode 1, internal clock */
    LED = 1;
    P2M1 &= 0xFE;
    P2M0 |= 0x01;
    US_TRIG = 0;
    US_ECHO = 1;
    P3M1 = (P3M1 & 0xEF) | 0x20; /* Trig push-pull, Echo input */
    P3M0 = (P3M0 | 0x10) & 0xDF;
    Delay10ms(50);
#if OLED_ENABLE
    oled_ok = OLEDInit();
#endif
    while (1) {
        test_status = Measure();
        if (test_status == US_OK) LED = (test_distance_mm <= 250U) ? 0 : 1;
        else LED = !LED;
#if OLED_ENABLE
        Display();
#endif
        Delay10ms(20); /* >=200 ms between tests; never tight retrigger */
    }
}
