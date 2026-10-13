#include "oled.h"
#include "timebase.h"
static u8 oled_address;
static u8 bus_ok;

static void I2CDelay(void) { Timebase_ShortDelay(); Timebase_ShortDelay(); }

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
static const char code glyph_names[] = "0123456789ACDEGHIMNORST .:-BFKLPUVW%";
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
 {0,0x36,0x36,0,0},{8,8,8,8,8},
 {0x7F,0x49,0x49,0x49,0x36},{0x7F,9,9,9,1},
 {0x7F,8,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
 {0x7F,9,9,9,6},{0x3F,0x40,0x40,0x40,0x3F},
 {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
 {0x63,0x13,8,0x64,0x63}
};

void OLED_Text(u8 page, char *text)
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

u8 OLED_Init(void)
{
    u8 i, page;
    OLED_SCL = 1;
    OLED_SDA = 1;
    P2M1 |= 0x0C;
    P2M0 |= 0x0C; /* open drain */
    Timebase_DelayMs(200);
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
    for (page = 0; page < 8; page++) OLED_Text(page, "");
    Begin(0x00);
    Send(0xAF);
    I2CStop();
    return bus_ok;
}


u8 OLED_Ready(void) { return bus_ok; }
