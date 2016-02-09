// Adafruit_LEDBackpack.c : 

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

#include "Adafruit_LEDBackpack.h"
#include <i2c.h>
#include <stdint.h>

#undef ENABLE_LED_FLOAT

#ifndef _BV
  #define _BV(bit) (1<<(bit))
#endif

static uint8_t i2c_addr = 0x70;
static uint16_t displaybuffer[8];
static struct device * i2c;
static uint8_t position = 0;

static const uint8_t numbertable[] = {
	0x3F, /* 0 */
	0x06, /* 1 */
	0x5B, /* 2 */
	0x4F, /* 3 */
	0x66, /* 4 */
	0x6D, /* 5 */
	0x7D, /* 6 */
	0x07, /* 7 */
	0x7F, /* 8 */
	0x6F, /* 9 */
	0x77, /* a */
	0x7C, /* b */
	0x39, /* C */
	0x5E, /* d */
	0x79, /* E */
	0x71, /* F */
};

void bp_setdevice (zephyr_dev dev)
{
  i2c = dev;
}

void bp_setBrightness(uint8_t b) {
  uint8_t buf[] = {0};
  
  if (b > 15) b = 15;
  
  buf[0] = HT16K33_CMD_BRIGHTNESS | b;
  i2c_polling_write (i2c, buf, sizeof(buf), i2c_addr);
  
  // Wire.beginTransmission(i2c_addr);
  // Wire.write(HT16K33_CMD_BRIGHTNESS | b);
  // Wire.endTransmission();  
}

void bp_blinkRate(uint8_t b) {
  // Wire.beginTransmission(i2c_addr);
  uint8_t buf[] = {0};
  if (b > 3) b = 0; // turn off if not sure
  
  buf[0] = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1);
  i2c_polling_write(i2c, buf, sizeof (buf), i2c_addr);
  // Wire.write(HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1)); 
  // Wire.endTransmission();
}

void bp_begin() {
  // i2c_addr = _addr;
  uint8_t buf[] = {0};

  // Wire.begin();
  position = 0;

  // Wire.beginTransmission(i2c_addr);
  // Wire.write(0x21);  // turn on oscillator
  // Wire.endTransmission();
  buf[0] = 0x21;
  i2c_polling_write(i2c, buf, sizeof (buf), i2c_addr);
  bp_blinkRate(HT16K33_BLINK_OFF);
  
  bp_setBrightness(15); // max brightness
}

void bp_writeDisplay(void) {
  uint8_t buf[17] = {0};
  uint8_t idx = 1;
  // Wire.beginTransmission(i2c_addr);
  // Wire.write((uint8_t)0x00); // start at address $00
  
  //for (uint8_t i=0; i<8; i++) {
  //   Wire.write(displaybuffer[i] & 0xFF);    
  //   Wire.write(displaybuffer[i] >> 8);    
  // Wire.endTransmission();
  
  for (uint8_t i=0; i<=8; i++) {
    buf[idx] = displaybuffer[i] & 0xFF;
    idx++;
    buf[idx] = displaybuffer[i] >> 8;
    idx++;
  }
  
  i2c_polling_write(i2c, buf, sizeof (buf), i2c_addr);
}

void bp_clear(void) {
  for (uint8_t i=0; i<8; i++) {
    displaybuffer[i] = 0;
  }
}

/******************************* 7 SEGMENT OBJECT */

// seg_Adafruit_7segment(void) {
//   position = 0;
// }

#ifdef ENABLE_LED_FLOAT
void seg_print(unsigned long n, int base)
{
  if (base == 0) seg_write(n);
  else seg_printNumber(n, base);
}


void  seg_println(unsigned int n, int base)
{
  seg_print(n, base);
  // seg_println();
  position = 0;
}

static size_t seg_write(uint8_t c) {

  uint8_t r = 0;

  if (c == '\n') position = 0;
  if (c == '\r') position = 0;

  if ((c >= '0') && (c <= '9')) {
    seg_writeDigitNum(position, c-'0', false);
    r = 1;
  }

  position++;
  if (position == 2) position++;

  return r;
}
#endif

static void seg_writeDigitRaw(uint8_t d, uint8_t bitmask) {
  if (d > 4) return;
  displaybuffer[d] = bitmask;
}

static void seg_drawColon(boolean state) {
  if (state)
    displaybuffer[2] = 0x2;
  else
    displaybuffer[2] = 0;
}

static void seg_writeColon(void) {
//     Wire.beginTransmission(i2c_addr);
//     Wire.write((uint8_t)0x04); // start at address $02
    
//     Wire.write(displaybuffer[2] & 0xFF);
//     Wire.write(displaybuffer[2] >> 8);

//     Wire.endTransmission();
  uint8_t buf[] = {0, 0, 0};
  buf[0] = 0x04;
  buf[1] = displaybuffer [2] & 0xFF;
  buf[2] = displaybuffer [2] >> 8;
  i2c_polling_write (i2c, buf, sizeof (buf), i2c_addr);
}

void seg_writeDigitNum(uint8_t d, uint8_t num, boolean dot) {
  if (d > 4) return;

  seg_writeDigitRaw(d, numbertable[num] | (dot << 7));
}

void seg_writeNumber(uint32_t n) {
  uint32_t i,pos;
  
	for (i=4,pos=4;i>0;i--,pos--) {
		if (2 == pos) pos--; // skip colon position
		seg_writeDigitNum(pos, n % 10, false);
		n /= 10;
	}
	bp_writeDisplay();
}

#ifdef ENABLE_LED_FLOAT

// void seg_print(long n, int base)
// {
//   seg_printNumber(n, base);
// }

void seg_printNumber(long n, uint8_t base)
{
    seg_printFloat(n, 0, base);
}

void seg_printFloat(double n, uint8_t fracDigits, uint8_t base) 
{ 
  uint8_t numericDigits = 4;   // available digits on display
  boolean isNegative = false;  // true if the number is negative
  
  // is the number negative?
  if(n < 0) {
    isNegative = true;  // need to draw sign later
    --numericDigits;    // the sign will take up one digit
    n *= -1;            // pretend the number is positive
  }
  
  // calculate the factor required to shift all fractional digits
  // into the integer part of the number
  double toIntFactor = 1.0;
  for(int i = 0; i < fracDigits; ++i) toIntFactor *= base;
  
  // create integer containing digits to display by applying
  // shifting factor and rounding adjustment
  uint32_t displayNumber = n * toIntFactor + 0.5;
  
  // calculate upper bound on displayNumber given
  // available digits on display
  uint32_t tooBig = 1;
  for(int i = 0; i < numericDigits; ++i) tooBig *= base;
  
  // if displayNumber is too large, try fewer fractional digits
  while(displayNumber >= tooBig) {
    --fracDigits;
    toIntFactor /= base;
    displayNumber = n * toIntFactor + 0.5;
  }
  
  // did toIntFactor shift the decimal off the display?
  if (toIntFactor < 1) {
    seg_printError();
  } else {
    // otherwise, display the number
    int8_t displayPos = 4;
    
    if (displayNumber)  //if displayNumber is not 0
    {
      for(uint8_t i = 0; displayNumber || i <= fracDigits; ++i) {
        boolean displayDecimal = (fracDigits != 0 && i == fracDigits);
        seg_writeDigitNum(displayPos--, displayNumber % base, displayDecimal);
        if(displayPos == 2) seg_writeDigitRaw(displayPos--, 0x00);
        displayNumber /= base;
      }
    }
    else {
      seg_writeDigitNum(displayPos--, 0, false);
    }
  
    // display negative sign if negative
    if(isNegative) seg_writeDigitRaw(displayPos--, 0x40);
  
    // clear remaining display positions
    while(displayPos >= 0) seg_writeDigitRaw(displayPos--, 0x00);
  }
}

void seg_printError(void) {
  for(uint8_t i = 0; i < SEVENSEG_DIGITS; ++i) {
    seg_writeDigitRaw(i, (i == 2 ? 0x00 : 0x40));
  }
}
#endif
