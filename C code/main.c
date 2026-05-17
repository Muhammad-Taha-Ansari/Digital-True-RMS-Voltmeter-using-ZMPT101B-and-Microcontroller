/*
 * ZMPT101B AC Voltmeter
 * ATmega328P @ 16 MHz | Code::Blocks AVR GCC | C89
 *
 * Wiring:
 *   OLED SDA -> A4 (PC4)
 *   OLED SCL -> A5 (PC5)
 *   OLED VCC -> 5V
 *   OLED GND -> GND
 *   ZMPT101B OUT -> A0
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define BAUD      9600UL
#define UBRR_VAL  (F_CPU / 16UL / BAUD - 1)

/* ============================================================
 *  I2C — 400 kHz
 * ============================================================ */
static void i2c_init(void)
{
    TWBR = (uint8_t)(((F_CPU / 400000UL) - 16UL) / 2UL);
    TWSR &= ~((1 << TWPS1) | (1 << TWPS0));
    TWCR  = (1 << TWEN);
}
static void i2c_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}
static void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}
static void i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

/* ============================================================
 *  SSD1306 OLED  (0x3C)
 * ============================================================ */
#define OLED_W  0x78
#define SCR_W   128
#define SCR_PG  8

static void oled_cmd(uint8_t c)
{
    i2c_start(); i2c_write(OLED_W); i2c_write(0x00); i2c_write(c); i2c_stop();
}
static void oled_dat(uint8_t d)
{
    i2c_start(); i2c_write(OLED_W); i2c_write(0x40); i2c_write(d); i2c_stop();
}
static void oled_init(void)
{
    uint8_t i;
    static const uint8_t seq[] PROGMEM = {
        0xAE, 0xD5,0x80, 0xA8,0x3F, 0xD3,0x00, 0x40,
        0x8D,0x14, 0x20,0x00, 0xA1, 0xC8, 0xDA,0x12,
        0x81,0xCF, 0xD9,0xF1, 0xDB,0x40, 0xA4, 0xA6, 0xAF
    };
    _delay_ms(100);
    for (i = 0; i < sizeof(seq); i++) oled_cmd(pgm_read_byte(&seq[i]));
}
static void oled_pos(uint8_t page, uint8_t col)
{
    oled_cmd((uint8_t)(0xB0 | (page & 0x07)));
    oled_cmd((uint8_t)(0x00 | (col  & 0x0F)));
    oled_cmd((uint8_t)(0x10 | (col  >> 4)));
}
static void oled_clear(void)
{
    uint8_t p; uint8_t c;
    for (p = 0; p < SCR_PG; p++) {
        oled_pos(p, 0);
        for (c = 0; c < SCR_W; c++) oled_dat(0x00);
    }
}
static void oled_row(uint8_t page, uint8_t pat)
{
    uint8_t c;
    oled_pos(page, 0);
    for (c = 0; c < SCR_W; c++) oled_dat(pat);
}

/* ============================================================
 *  5x7 FONT
 * ============================================================ */
static const uint8_t FONT[][5] PROGMEM = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
    {0x00,0x07,0x00,0x07,0x00}, /* '"' */
    {0x14,0x7F,0x14,0x7F,0x14}, /* '#' */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' */
    {0x23,0x13,0x08,0x64,0x62}, /* '%' */
    {0x36,0x49,0x55,0x22,0x50}, /* '&' */
    {0x00,0x05,0x03,0x00,0x00}, /* '\'' */
    {0x00,0x1C,0x22,0x41,0x00}, /* '(' */
    {0x00,0x41,0x22,0x1C,0x00}, /* ')' */
    {0x08,0x2A,0x1C,0x2A,0x08}, /* '*' */
    {0x08,0x08,0x3E,0x08,0x08}, /* '+' */
    {0x00,0x50,0x30,0x00,0x00}, /* ',' */
    {0x08,0x08,0x08,0x08,0x08}, /* '-' */
    {0x00,0x60,0x60,0x00,0x00}, /* '.' */
    {0x20,0x10,0x08,0x04,0x02}, /* '/' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' */
    {0x00,0x56,0x36,0x00,0x00}, /* ';' */
    {0x08,0x14,0x22,0x41,0x00}, /* '<' */
    {0x14,0x14,0x14,0x14,0x14}, /* '=' */
    {0x00,0x41,0x22,0x14,0x08}, /* '>' */
    {0x02,0x01,0x51,0x09,0x06}, /* '?' */
    {0x32,0x49,0x79,0x41,0x3E}, /* '@' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
    {0x7F,0x02,0x04,0x02,0x7F}, /* 'M' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */
    {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' */
    {0x61,0x51,0x49,0x45,0x43}, /* 'Z' */
    {0x00,0x7F,0x41,0x41,0x00}, /* '[' */
    {0x02,0x04,0x08,0x10,0x20}, /* '\\' */
    {0x00,0x41,0x41,0x7F,0x00}, /* ']' */
    {0x04,0x02,0x01,0x02,0x04}, /* '^' */
    {0x40,0x40,0x40,0x40,0x40}, /* '_' */
    {0x00,0x01,0x02,0x04,0x00}, /* '`' */
    {0x20,0x54,0x54,0x54,0x78}, /* 'a' */
    {0x7F,0x48,0x44,0x44,0x38}, /* 'b' */
    {0x38,0x44,0x44,0x44,0x20}, /* 'c' */
    {0x38,0x44,0x44,0x48,0x7F}, /* 'd' */
    {0x38,0x54,0x54,0x54,0x18}, /* 'e' */
    {0x08,0x7E,0x09,0x01,0x02}, /* 'f' */
    {0x0C,0x52,0x52,0x52,0x3E}, /* 'g' */
    {0x7F,0x08,0x04,0x04,0x78}, /* 'h' */
    {0x00,0x44,0x7D,0x40,0x00}, /* 'i' */
    {0x20,0x40,0x44,0x3D,0x00}, /* 'j' */
    {0x7F,0x10,0x28,0x44,0x00}, /* 'k' */
    {0x00,0x41,0x7F,0x40,0x00}, /* 'l' */
    {0x7C,0x04,0x18,0x04,0x78}, /* 'm' */
    {0x7C,0x08,0x04,0x04,0x78}, /* 'n' */
    {0x38,0x44,0x44,0x44,0x38}, /* 'o' */
    {0x7C,0x14,0x14,0x14,0x08}, /* 'p' */
    {0x08,0x14,0x14,0x18,0x7C}, /* 'q' */
    {0x7C,0x08,0x04,0x04,0x08}, /* 'r' */
    {0x48,0x54,0x54,0x54,0x20}, /* 's' */
    {0x04,0x3F,0x44,0x40,0x20}, /* 't' */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 'u' */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 'v' */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 'w' */
    {0x44,0x28,0x10,0x28,0x44}, /* 'x' */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 'y' */
    {0x44,0x64,0x54,0x4C,0x44}, /* 'z' */
};

static void oled_ch(uint8_t page, uint8_t col, char c)
{
    uint8_t i;
    if (c < 0x20 || c > 0x7A) c = ' ';
    oled_pos(page, col);
    for (i = 0; i < 5; i++) oled_dat(pgm_read_byte(&FONT[c - 0x20][i]));
    oled_dat(0x00);
}
static void oled_str(uint8_t page, uint8_t col, const char *s)
{
    while (*s && col < SCR_W) { oled_ch(page, col, *s++); col += 6; }
}

/* ============================================================
 *  UART
 * ============================================================ */
static void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR_VAL);
    UCSR0B = (1 << TXEN0);
    UCSR0C = (3 << UCSZ00);
}
static void uart_char(char c) { while(!(UCSR0A&(1<<UDRE0))); UDR0=(uint8_t)c; }
static void uart_str(const char *s) { while(*s) uart_char(*s++); }

/* ============================================================
 *  ADC
 * ============================================================ */
static float adc_read(uint8_t ch)
{
    ADMUX  = (1<<REFS0)|(ch&0x07);
    ADCSRA = (1<<ADEN)|(1<<ADSC)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
    while (ADCSRA & (1<<ADSC));
    return (float)ADC;
}

/* ============================================================
 *  VOLTAGE HELPERS
 * ============================================================ */
static uint8_t findMaxIndex(float arr[], uint8_t size)
{
    int i; uint8_t idx = 0;
    for (i=1; i<size; i++) if (arr[i]>arr[idx]) idx=(uint8_t)i;
    return idx;
}
static uint8_t findMinIndex(float arr[], uint8_t size)
{
    int i; uint8_t idx = 0;
    for (i=1; i<size; i++) if (arr[i]<arr[idx]) idx=(uint8_t)i;
    return idx;
}

/* ============================================================
 *  SPLASH SCREEN
 *
 *  Page 0: ████████████████████  (solid)
 *  Page 1: "   ** VOLTMETER **"
 *  Page 2: "   ZMPT101B Sensor"
 *  Page 3: ████████████████████  (solid)
 *  Page 4: "Lecturer:"
 *  Page 5: "Sir HassanUlHaq"
 *  Page 6: "Lead: M.Taha EE-319"
 *  Page 7: ████████████████████  (solid)
 * ============================================================ */
static void oled_splash(void)
{
    oled_clear();
    oled_row(0, 0xFF);
    oled_str(1,  8, "** VOLTMETER **");
    oled_str(2, 14, "ZMPT101B Sensor");
    oled_row(3, 0xFF);
    oled_str(4,  4, "Lecturer:");
    oled_str(5,  4, "Sir HassanUlHaq");
    oled_str(6,  2, "Lead:M.Taha EE-319");
    oled_row(7, 0xFF);
}

/* ============================================================
 *  LIVE SCREEN — matches image exactly
 *
 *  Page 0: "==== VOLTMETER ===="
 *  Page 1: blank
 *  Page 2: "Vavg:      2.518 V"
 *  Page 3: blank
 *  Page 4: "Vrms:      0.347 V"
 *  Page 5: blank
 *  Page 6: "AC 50Hz  ZMPT101B"
 *  Page 7: blank
 * ============================================================ */
static void oled_update(float vavg, float vrms)
{
    char buf[12];

    /* Page 0 — header centered */
    oled_row(0, 0x00);
    oled_str(0, 4, "==== VOLTMETER ====");

    /* Page 1 — blank */
    oled_row(1, 0x00);

    /* Page 2 — Vavg
     * "Vavg:" label at col 2
     * value at col 56 (big gap like reference image)
     * "V" at col 98
     */
    oled_row(2, 0x00);
    oled_str(2,  2, "Vavg:");
    dtostrf(vavg, 7, 3, buf);
    oled_str(2, 56, buf);
    oled_str(2, 98, "V");

    /* Page 3 — blank */
    oled_row(3, 0x00);

    /* Page 4 — Vrms same layout */
    oled_row(4, 0x00);
    oled_str(4,  2, "Vrms:");
    dtostrf(vrms, 7, 3, buf);
    oled_str(4, 56, buf);
    oled_str(4, 98, "V");

    /* Page 5 — status (moved here so font bleed goes to blank page 6) */
    oled_row(5, 0x00);
    oled_str(5, 4, "AC 50Hz  ZMPT101B");

    /* Page 6 — blank buffer (catches font pixel bleed from page 5) */
    oled_row(6, 0x00);

    /* Page 7 — clear top pixel only (removes font underline bleed)
     * 0x01 = only bottom bit set = top row of page = pixel row 56
     * We write 0x00 to wipe any leftover pixels from page 6 font */
    oled_row(7, 0x00);
}

/* ============================================================
 *  MAIN
 * ============================================================ */
int main(void)
{
    float   voltage[200];
    float   Vavg;
    float   Vrms;
    uint8_t max_idx;
    uint8_t min_idx;
    uint8_t half;
    uint8_t i;
    char    buf[12];

    uart_init();
    i2c_init();
    oled_init();
    oled_clear();

    /* OLED Splash */
    oled_splash();

    /* Tera Term Splash */
    uart_str("\r\n");
    uart_str("==================================\r\n");
    uart_str("        ** VOLTMETER **           \r\n");
    uart_str("      ZMPT101B AC Sensor          \r\n");
    uart_str("----------------------------------\r\n");
    uart_str(" Lecturer : Sir HassanUlHaq      \r\n");
    uart_str(" Lead     : M. Taha  EE-23319    \r\n");
    uart_str("==================================\r\n");
    uart_str("\r\n");

    _delay_ms(2000);
    oled_clear();

    while (1)
    {
        /* 1. Sample 200 points */
        for (i = 0; i < 200; i++)
            voltage[i] = adc_read(0) * (5.0f / 1023.0f);

        /* 2. Peaks */
        max_idx = findMaxIndex(voltage, 200);
        min_idx = findMinIndex(voltage, 200);

        /* 3. DC offset */
        Vavg = 0.0f;
        half = (max_idx > min_idx) ? (max_idx - min_idx)
                                   : (min_idx - max_idx);
        if (half < 1)  half = 1;
        if (half > 99) half = 99;

        for (i = 0; i < (2*half); i++) Vavg += voltage[i];
        Vavg = (float)fabs(Vavg / (2.0f * (float)half));

        /* 4. Remove offset */
        for (i = 0; i < 200; i++) voltage[i] -= Vavg;

        /* 5. RMS */
        Vrms = 0.0f;
        for (i = 0; i < (2*half); i++)
            Vrms += voltage[i] * voltage[i];
        Vrms = (float)sqrt((double)(Vrms / (2.0f * (float)half)));

        /* 6. Sanity guard */
        if (Vavg > 5.0f || Vavg < 0.0f) Vavg = 0.0f;
        if (Vrms > 5.0f || Vrms < 0.0f) Vrms = 0.0f;

        /* 7. UART — new line each update */
        uart_str("  Vavg: ");
        dtostrf(Vavg, 6, 3, buf); uart_str(buf);
        uart_str(" V  |  Vrms: ");
        dtostrf(Vrms, 6, 3, buf); uart_str(buf);
        uart_str(" V\r\n");

        /* 8. OLED */
        oled_update(Vavg, Vrms);

        _delay_ms(500);
    }

    return 0;
}
