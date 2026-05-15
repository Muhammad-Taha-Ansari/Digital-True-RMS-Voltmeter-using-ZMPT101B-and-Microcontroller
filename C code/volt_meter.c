/*
 * ZMPT101B AC Voltage Sensor — AVR C Code
 * Display: SSD1306 OLED 128x64 via I2C
 * UART: Tera Term serial monitor
 *
 * Connections:
 *   OLED SDA -> Arduino A4 (PC4)
 *   OLED SCL -> Arduino A5 (PC5)
 *   ZMPT101B OUT -> Arduino A0 (PC0)
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ─── CPU & UART ────────────────────────────────────────────────
#define F_CPU      16000000UL
#define BAUD       9600
#define UBRR_VAL   ((F_CPU / (16UL * BAUD)) - 1)

// ─── I2C (TWI) ────────────────────────────────────────────────
#define I2C_SCL_FREQ   400000UL
#define TWBR_VAL       ((F_CPU / I2C_SCL_FREQ - 16) / 2)

// ─── OLED SSD1306 ─────────────────────────────────────────────
#define OLED_ADDR      0x3C   // 7-bit I2C address (0x78 write)
#define OLED_CMD       0x00
#define OLED_DATA      0x40
#define OLED_WIDTH     128
#define OLED_PAGES     8      // 64px / 8 = 8 pages

// ─── Function Prototypes ──────────────────────────────────────
uint8_t  findMaxIndex(float arr[], uint8_t size);
uint8_t  findMinIndex(float arr[], uint8_t size);
float    analog_read(uint8_t channel);
void     usart_init(void);
void     usart_send_char(char c);
void     usart_print(const char *str);

// I2C
void     i2c_init(void);
void     i2c_start(void);
void     i2c_stop(void);
void     i2c_write(uint8_t data);

// OLED
void     oled_command(uint8_t cmd);
void     oled_data(uint8_t data);
void     oled_init(void);
void     oled_clear(void);
void     oled_set_cursor(uint8_t page, uint8_t col);
void     oled_print_char(char c);
void     oled_print_str(const char *str);
void     oled_display_voltages(float vavg, float vrms);

// ─── 5x7 Font (ASCII 32–127) ─────────────────────────────────
// Minimal subset — space through Z + lowercase + digits
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // 32 space
    {0x00,0x00,0x5F,0x00,0x00}, // 33 !
    {0x00,0x07,0x00,0x07,0x00}, // 34 "
    {0x14,0x7F,0x14,0x7F,0x14}, // 35 #
    {0x24,0x2A,0x7F,0x2A,0x12}, // 36 $
    {0x23,0x13,0x08,0x64,0x62}, // 37 %
    {0x36,0x49,0x55,0x22,0x50}, // 38 &
    {0x00,0x05,0x03,0x00,0x00}, // 39 '
    {0x00,0x1C,0x22,0x41,0x00}, // 40 (
    {0x00,0x41,0x22,0x1C,0x00}, // 41 )
    {0x14,0x08,0x3E,0x08,0x14}, // 42 *
    {0x08,0x08,0x3E,0x08,0x08}, // 43 +
    {0x00,0x50,0x30,0x00,0x00}, // 44 ,
    {0x08,0x08,0x08,0x08,0x08}, // 45 -
    {0x00,0x60,0x60,0x00,0x00}, // 46 .
    {0x20,0x10,0x08,0x04,0x02}, // 47 /
    {0x3E,0x51,0x49,0x45,0x3E}, // 48 0
    {0x00,0x42,0x7F,0x40,0x00}, // 49 1
    {0x42,0x61,0x51,0x49,0x46}, // 50 2
    {0x21,0x41,0x45,0x4B,0x31}, // 51 3
    {0x18,0x14,0x12,0x7F,0x10}, // 52 4
    {0x27,0x45,0x45,0x45,0x39}, // 53 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 54 6
    {0x01,0x71,0x09,0x05,0x03}, // 55 7
    {0x36,0x49,0x49,0x49,0x36}, // 56 8
    {0x06,0x49,0x49,0x29,0x1E}, // 57 9
    {0x00,0x36,0x36,0x00,0x00}, // 58 :
    {0x00,0x56,0x36,0x00,0x00}, // 59 ;
    {0x08,0x14,0x22,0x41,0x00}, // 60 <
    {0x14,0x14,0x14,0x14,0x14}, // 61 =
    {0x00,0x41,0x22,0x14,0x08}, // 62 >
    {0x02,0x01,0x51,0x09,0x06}, // 63 ?
    {0x32,0x49,0x79,0x41,0x3E}, // 64 @
    {0x7E,0x11,0x11,0x11,0x7E}, // 65 A
    {0x7F,0x49,0x49,0x49,0x36}, // 66 B
    {0x3E,0x41,0x41,0x41,0x22}, // 67 C
    {0x7F,0x41,0x41,0x22,0x1C}, // 68 D
    {0x7F,0x49,0x49,0x49,0x41}, // 69 E
    {0x7F,0x09,0x09,0x09,0x01}, // 70 F
    {0x3E,0x41,0x49,0x49,0x7A}, // 71 G
    {0x7F,0x08,0x08,0x08,0x7F}, // 72 H
    {0x00,0x41,0x7F,0x41,0x00}, // 73 I
    {0x20,0x40,0x41,0x3F,0x01}, // 74 J
    {0x7F,0x08,0x14,0x22,0x41}, // 75 K
    {0x7F,0x40,0x40,0x40,0x40}, // 76 L
    {0x7F,0x02,0x04,0x02,0x7F}, // 77 M
    {0x7F,0x04,0x08,0x10,0x7F}, // 78 N
    {0x3E,0x41,0x41,0x41,0x3E}, // 79 O
    {0x7F,0x09,0x09,0x09,0x06}, // 80 P
    {0x3E,0x41,0x51,0x21,0x5E}, // 81 Q
    {0x7F,0x09,0x19,0x29,0x46}, // 82 R
    {0x46,0x49,0x49,0x49,0x31}, // 83 S
    {0x01,0x01,0x7F,0x01,0x01}, // 84 T
    {0x3F,0x40,0x40,0x40,0x3F}, // 85 U
    {0x1F,0x20,0x40,0x20,0x1F}, // 86 V
    {0x3F,0x40,0x38,0x40,0x3F}, // 87 W
    {0x63,0x14,0x08,0x14,0x63}, // 88 X
    {0x07,0x08,0x70,0x08,0x07}, // 89 Y
    {0x61,0x51,0x49,0x45,0x43}, // 90 Z
    {0x00,0x7F,0x41,0x41,0x00}, // 91 [
    {0x02,0x04,0x08,0x10,0x20}, // 92 backslash
    {0x00,0x41,0x41,0x7F,0x00}, // 93 ]
    {0x04,0x02,0x01,0x02,0x04}, // 94 ^
    {0x40,0x40,0x40,0x40,0x40}, // 95 _
    {0x00,0x01,0x02,0x04,0x00}, // 96 `
    {0x20,0x54,0x54,0x54,0x78}, // 97 a
    {0x7F,0x48,0x44,0x44,0x38}, // 98 b
    {0x38,0x44,0x44,0x44,0x20}, // 99 c
    {0x38,0x44,0x44,0x48,0x7F}, // 100 d
    {0x38,0x54,0x54,0x54,0x18}, // 101 e
    {0x08,0x7E,0x09,0x01,0x02}, // 102 f
    {0x0C,0x52,0x52,0x52,0x3E}, // 103 g
    {0x7F,0x08,0x04,0x04,0x78}, // 104 h
    {0x00,0x44,0x7D,0x40,0x00}, // 105 i
    {0x20,0x40,0x44,0x3D,0x00}, // 106 j
    {0x7F,0x10,0x28,0x44,0x00}, // 107 k
    {0x00,0x41,0x7F,0x40,0x00}, // 108 l
    {0x7C,0x04,0x18,0x04,0x78}, // 109 m
    {0x7C,0x08,0x04,0x04,0x78}, // 110 n
    {0x38,0x44,0x44,0x44,0x38}, // 111 o
    {0x7C,0x14,0x14,0x14,0x08}, // 112 p
    {0x08,0x14,0x14,0x18,0x7C}, // 113 q
    {0x7C,0x08,0x04,0x04,0x08}, // 114 r
    {0x48,0x54,0x54,0x54,0x20}, // 115 s
    {0x04,0x3F,0x44,0x40,0x20}, // 116 t
    {0x3C,0x40,0x40,0x20,0x7C}, // 117 u
    {0x1C,0x20,0x40,0x20,0x1C}, // 118 v
    {0x3C,0x40,0x30,0x40,0x3C}, // 119 w
    {0x44,0x28,0x10,0x28,0x44}, // 120 x
    {0x0C,0x50,0x50,0x50,0x3C}, // 121 y
    {0x44,0x64,0x54,0x4C,0x44}, // 122 z
};

// ══════════════════════════════════════════════════════════════
//  MAIN
// ══════════════════════════════════════════════════════════════
int main(void)
{
    float    voltage[200];
    float    Vavg = 0.0f;
    float    Vrms = 0.0f;
    uint8_t  max_index, min_index;
    char     buf[20];

    usart_init();
    i2c_init();
    oled_init();
    oled_clear();

    /* Splash screen */
    oled_set_cursor(2, 10);
    oled_print_str("ZMPT101B Meter");
    oled_set_cursor(4, 20);
    oled_print_str("Initializing...");
    _delay_ms(1500);
    oled_clear();

    while (1)
    {
        /* ── 1. Sample 200 points ── */
        for (uint8_t i = 0; i < 200; i++) {
            voltage[i] = analog_read(0) * (5.0f / 1023.0f);
        }

        /* ── 2. Find peak indices ── */
        max_index = findMaxIndex(voltage, 200);
        min_index = findMinIndex(voltage, 200);

        /* ── 3. Average voltage (DC offset) ── */
        Vavg = 0.0f;
        uint8_t half_cycle = (max_index > min_index)
                             ? (max_index - min_index)
                             : (min_index - max_index);

        for (uint8_t i = 0; i < (2 * half_cycle); i++) {
            Vavg += voltage[i];
        }
        Vavg = fabs(Vavg / (2.0f * half_cycle));

        /* ── 4. Remove DC offset ── */
        for (uint8_t i = 0; i < 200; i++) {
            voltage[i] -= Vavg;
        }

        /* ── 5. RMS voltage ── */
        Vrms = 0.0f;
        if (max_index > min_index) {
            for (uint8_t i = 0; i < (2 * half_cycle); i++) {
                Vrms += voltage[i] * voltage[i];
            }
            Vrms = sqrt(fabs(Vrms / (2.0f * half_cycle)));
        }

        /* ── 6. UART output ── */
        usart_print("Vavg: ");
        dtostrf(Vavg, 6, 3, buf);
        usart_print(buf);
        usart_print(" V    Vrms: ");
        dtostrf(Vrms, 6, 3, buf);
        usart_print(buf);
        usart_print(" V\r\n");

        /* ── 7. OLED output ── */
        oled_display_voltages(Vavg, Vrms);

        _delay_ms(500);
    }
}

// ══════════════════════════════════════════════════════════════
//  ADC
// ══════════════════════════════════════════════════════════════
float analog_read(uint8_t channel)
{
    ADMUX  = (1 << REFS0) | (channel & 0x07); // AVcc ref, select channel
    ADCSRA = (1 << ADEN) | (1 << ADSC)
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128
    while (ADCSRA & (1 << ADSC));             // wait for conversion
    return (float)ADC;
}

// ══════════════════════════════════════════════════════════════
//  USART
// ══════════════════════════════════════════════════════════════
void usart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR_VAL);
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

void usart_send_char(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void usart_print(const char *str)
{
    while (*str) usart_send_char(*str++);
}

// ══════════════════════════════════════════════════════════════
//  I2C (TWI)
// ══════════════════════════════════════════════════════════════
void i2c_init(void)
{
    TWSR = 0x00;           // prescaler = 1
    TWBR = (uint8_t)TWBR_VAL;
    TWCR = (1 << TWEN);
}

void i2c_start(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

void i2c_stop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
    while (TWCR & (1<<TWSTO));
}

void i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

// ══════════════════════════════════════════════════════════════
//  OLED SSD1306
// ══════════════════════════════════════════════════════════════
void oled_command(uint8_t cmd)
{
    i2c_start();
    i2c_write((OLED_ADDR << 1) | 0); // write address
    i2c_write(OLED_CMD);             // Co=0, D/C#=0
    i2c_write(cmd);
    i2c_stop();
}

void oled_data(uint8_t data)
{
    i2c_start();
    i2c_write((OLED_ADDR << 1) | 0);
    i2c_write(OLED_DATA);            // Co=0, D/C#=1
    i2c_write(data);
    i2c_stop();
}

void oled_init(void)
{
    _delay_ms(100);
    oled_command(0xAE); // display off
    oled_command(0xD5); oled_command(0x80); // clock divide
    oled_command(0xA8); oled_command(0x3F); // multiplex 64
    oled_command(0xD3); oled_command(0x00); // display offset
    oled_command(0x40);                     // start line
    oled_command(0x8D); oled_command(0x14); // charge pump ON
    oled_command(0x20); oled_command(0x00); // horizontal addressing
    oled_command(0xA1);                     // segment remap
    oled_command(0xC8);                     // COM scan direction
    oled_command(0xDA); oled_command(0x12); // COM pins
    oled_command(0x81); oled_command(0xCF); // contrast
    oled_command(0xD9); oled_command(0xF1); // precharge
    oled_command(0xDB); oled_command(0x40); // VCOMH deselect
    oled_command(0xA4);                     // display RAM
    oled_command(0xA6);                     // normal (not inverted)
    oled_command(0xAF);                     // display on
}

void oled_clear(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        oled_set_cursor(page, 0);
        for (uint8_t col = 0; col < OLED_WIDTH; col++) {
            oled_data(0x00);
        }
    }
}

void oled_set_cursor(uint8_t page, uint8_t col)
{
    oled_command(0xB0 | page);              // page address
    oled_command(0x00 | (col & 0x0F));      // lower col nibble
    oled_command(0x10 | (col >> 4));        // upper col nibble
}

void oled_print_char(char c)
{
    if (c < 32 || c > 122) c = ' ';
    const uint8_t *glyph = font5x7[c - 32];
    for (uint8_t i = 0; i < 5; i++) oled_data(glyph[i]);
    oled_data(0x00); // 1px spacing
}

void oled_print_str(const char *str)
{
    while (*str) oled_print_char(*str++);
}

/*
 * oled_display_voltages()
 * Lays out 4 rows on the 128x64 display:
 *
 *  Page 0: ┌─ AC VOLTAGE METER ─┐   (title)
 *  Page 1: │                     │
 *  Page 2: │  Vavg: X.XXX V      │
 *  Page 3: │                     │
 *  Page 4: │  Vrms: X.XXX V      │
 *  Page 5: │                     │
 *  Page 6: │  [ZMPT101B / 50Hz]  │   (status)
 *  Page 7: └─────────────────────┘
 */
void oled_display_voltages(float vavg, float vrms)
{
    char buf[20];

    // ── Title row ──
    oled_set_cursor(0, 4);
    oled_print_str("  AC VOLTAGE METER  ");

    // ── Separator line (solid pixels) ──
    oled_set_cursor(1, 0);
    for (uint8_t i = 0; i < OLED_WIDTH; i++) oled_data(0xFF);

    // ── Vavg ──
    oled_set_cursor(2, 4);
    oled_print_str("Vavg:");
    dtostrf(vavg, 7, 3, buf);
    oled_print_str(buf);
    oled_print_str(" V");

    // ── blank row ──
    oled_set_cursor(3, 0);
    for (uint8_t i = 0; i < OLED_WIDTH; i++) oled_data(0x00);

    // ── Vrms ──
    oled_set_cursor(4, 4);
    oled_print_str("Vrms:");
    dtostrf(vrms, 7, 3, buf);
    oled_print_str(buf);
    oled_print_str(" V");

    // ── blank row ──
    oled_set_cursor(5, 0);
    for (uint8_t i = 0; i < OLED_WIDTH; i++) oled_data(0x00);

    // ── Status row ──
    oled_set_cursor(6, 4);
    oled_print_str("ZMPT101B  50Hz");

    // ── Bottom separator ──
    oled_set_cursor(7, 0);
    for (uint8_t i = 0; i < OLED_WIDTH; i++) oled_data(0xFF);
}

// ══════════════════════════════════════════════════════════════
//  HELPER FUNCTIONS
// ══════════════════════════════════════════════════════════════
uint8_t findMaxIndex(float arr[], uint8_t size)
{
    uint8_t maxIndex = 0;
    for (int i = 1; i < size; i++) {
        if (arr[i] > arr[maxIndex]) maxIndex = i;
    }
    return maxIndex;
}

uint8_t findMinIndex(float arr[], uint8_t size)
{
    uint8_t minIndex = 0;
    for (int i = 1; i < size; i++) {
        if (arr[i] < arr[minIndex]) minIndex = i;
    }
    return minIndex;
}
