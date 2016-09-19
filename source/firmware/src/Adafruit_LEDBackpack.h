// Adafruit_LEDBackpack.h :

/***************************************************
  This is a library for our I2C LED Backpacks

  Designed specifically to work with the Adafruit LED Matrix backpacks
  ----> http://www.adafruit.com/products/
  ----> http://www.adafruit.com/products/

  These displays use I2C to communicate, 2 pins are required to
  interface. There are multiple selectable I2C addresses. For backpacks
  with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
  with 3 Address Select pins: 0x70 thru 0x77

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

/***************************************************
 * modified to the Rocket API, interger support only
 * <credits>
 *   { Vincent Rubiolo, vincent.rubiolo@windriver.com,  },
 *   { David Reyna,  david.reyna@windriver.com,  },
 * </credits>
 ****************************************************/

#ifndef Adafruit_LEDBackpack_h
#define Adafruit_LEDBackpack_h

#include <stdint.h>
#include <stddef.h>

// typedef char uint8_t;
// typedef short uint16_t;
// typedef unsigned int uint32_t;
// typedef uint32_t size_t;
typedef struct device * zephyr_dev;
typedef char boolean;

#define false 0
#define true 1
#define PROGMEM

#define LED_ON 1
#define LED_OFF 0

#define LED_RED 1
#define LED_YELLOW 2
#define LED_GREEN 3

#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3

#define HT16K33_CMD_BRIGHTNESS 0xE0

#define SEVENSEG_DIGITS 5


// // this is the raw HT16K33 controller
// class Adafruit_LEDBackpack {
//  public:
//   Adafruit_LEDBackpack(void);
//   void begin(uint8_t _addr);
//   void setBrightness(uint8_t b);
//   void blinkRate(uint8_t b);
//   void writeDisplay(void);
//   void clear(void);

//   uint16_t displaybuffer[8];

//   void init(uint8_t a);
//  protected:
//   uint8_t i2c_addr;
// };

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0


// class Adafruit_7segment : public Adafruit_LEDBackpack {
//  public:
//   Adafruit_7segment(void);
//   size_t write(uint8_t c);

//   void print(char, int = BYTE);
//   void print(unsigned char, int = BYTE);
//   void print(int, int = DEC);
//   void print(unsigned int, int = DEC);
//   void print(long, int = DEC);
//   void print(unsigned long, int = DEC);
//   void print(double, int = 2);
//   void println(char, int = BYTE);
//   void println(unsigned char, int = BYTE);
//   void println(int, int = DEC);
//   void println(unsigned int, int = DEC);
//   void println(long, int = DEC);
//   void println(unsigned long, int = DEC);
//   void prgintln(double, int = 2);
//   void println(void);

//   void writeDigitRaw(uint8_t x, uint8_t bitmask);
//   void writeDigitNum(uint8_t x, uint8_t num, boolean dot = false);
//   void drawColon(boolean state);
//   void printNumber(long, uint8_t = 2);
//   void printFloat(double, uint8_t = 2, uint8_t = DEC);
//   void printError(void);

//   void writeColon(void);

//  private:
//   uint8_t position;
// };

void bp_begin (void);
void bp_setdevice (zephyr_dev);
void bp_writeDisplay (void);
void bp_clear (void);

//void setup (zephyr_dev dev);
//void loop (void);
//size_t seg_write(uint8_t c);
//void seg_printNumber(long n, uint8_t base);

void seg_writeDigitNum(uint8_t d, uint8_t num, boolean dot);

void seg_writeNumber(uint32_t n);

#ifdef ENABLE_LED_FLOAT
void seg_print(unsigned long n, int base);
void seg_println (unsigned int n, int base);
void seg_printFloat(double n, uint8_t fracDigits, uint8_t base);
void seg_printError(void);
#endif

//void seg_writeDigitRaw(uint8_t d, uint8_t bitmask);
//void seg_drawColon(boolean state);

#endif // Adafruit_LEDBackpack_h

