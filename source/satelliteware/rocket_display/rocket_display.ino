// rocket_display.ino: Rocket display Arduino instance

#include <Wire.h>
#include "TM1637.h"
#include "MicroAve.h"

// General Enbles
#define ENABLE_I2C      1
#define ENABLE_LED1     1
#define ENABLE_LED2     1
#define ENABLE_SOUND    1
#define ENABLE_PAN_TILT 0
#define ENABLE_NEOPIXEL 0
#define ENABLE_LEDRGB   0

#define ENABLE_TIMING   1   // track the I2C timing overhead

//
// which Arduino is this
//

#undef  IS_MICRO
#define IS_UNO

// ======== ARDUINO MICRO PORT ASSIGNMENTS =========
#ifdef IS_MICRO
// Height LED
#define LED1_CLK 4
#define LED1_DIO 6
TM1637 tm1637_1(LED1_CLK,LED1_DIO);

// Speed LED
#define LED2_CLK 5
#define LED2_DIO 6
TM1637 tm1637_2(LED2_CLK,LED2_DIO);

// Sound
#define SOUND_TRIGGER_1_PORT  7
#define SOUND_TRIGGER_2_PORT  8
#define SOUND_TRIGGER_3_PORT  9

// Pan and Tilt
#define PWM_PAN_PORT  10
#define PWM_TILT_PORT 11

// Neopixel
#define Neo_Pixel_PORT  13

// LED-RGB
#define LED_RGB_PORT  12
#endif


// ======== ARDUINO UNO PORT ASSIGNMENTS =========
#ifdef IS_UNO
// Height LED
#define LED1_CLK 5
#define LED1_DIO 6
TM1637 tm1637_1(LED1_CLK,LED1_DIO);

// Speed LED
#define LED2_CLK 7
#define LED2_DIO 8
TM1637 tm1637_2(LED2_CLK,LED2_DIO);

// Sound
#define SOUND_TRIGGER_1_PORT  9
#define SOUND_TRIGGER_2_PORT  10
#define SOUND_TRIGGER_3_PORT  11

// Pan and Tilt
#define PWM_PAN_PORT  3
#define PWM_TILT_PORT 4 // ############ TODO

// Neopixel
#define Neo_Pixel_PORT  13

// LED-RGB
#define LED_RGB_PORT  12

#endif

/* Sound Setup */
#define SOUND_QUIET  0
#define SOUND_READY  1
#define SOUND_PLAY   2
#define SOUND_DANGER 3
#define SOUND_LAND   4
#define SOUND_CRASH  5
#define SOUND_MAX    6

/* NeoPixel Setup */
#define NEOPIXEL_QUIET  0
#define NEOPIXEL_READY  1
#define NEOPIXEL_PLAY   2
#define NEOPIXEL_DANGER 3
#define NEOPIXEL_LAND   4
#define NEOPIXEL_CRASH  5
#define NEOPIXEL_MAX    6

/* I2C port Setup*/
#define SATELLITE_I2C_ADDRESS 18
#define I2C_READ_MAX 20

/* Timing analysis objects */
MicroAve ave_i2c;
MicroAve ave_loop;

//
// hardware asynchronous update handlers
//

boolean tigger_led1 = false;
byte led1_a;byte led1_b;byte led1_c;byte led1_d;
void write_led1() {
  if (ENABLE_LED1) {
    tm1637_1.display(0,led1_a);
    tm1637_1.display(1,led1_b);
    tm1637_1.display(2,led1_c);
    tm1637_1.display(3,led1_d);
  }
  tigger_led1 = false;
}

boolean tigger_led2 = false;
byte led2_a;byte led2_b;byte led2_c;byte led2_d;
void write_led2() {
  if (ENABLE_LED2) {
    tm1637_2.display(0,led2_a);
    tm1637_2.display(1,led2_b);
    tm1637_2.display(2,led2_c);
    tm1637_2.display(3,led2_d);
  }
  tigger_led2 = false;
}

boolean tigger_pan = false;
byte pantilt_pan;byte pantilt_tilt;
void write_pan_tilt() {
  if (ENABLE_PAN_TILT) {
    analogWrite(PWM_PAN_PORT , pantilt_pan);
    analogWrite(PWM_TILT_PORT, pantilt_tilt);
  }
  tigger_pan = false;
}

boolean tigger_ledrgb = false;
byte ledrgb_r;byte ledrgb_g;byte ledrgb_b;
void write_led_rgb() {
  if (ENABLE_LEDRGB) {
    // TODO ##################
  }
  tigger_ledrgb = false;
}

boolean tigger_neo = false;
byte neo_pattern;
void write_neopixel() {
  if (ENABLE_NEOPIXEL) {
    // TODO ##################
  }
  tigger_neo = false;
}

boolean tigger_sound = false;
byte sound_pattern;
void write_sound() {
  if (ENABLE_SOUND) {
      if (sound_pattern & 0x01)
        digitalWrite(SOUND_TRIGGER_1_PORT, HIGH);
      else
        digitalWrite(SOUND_TRIGGER_1_PORT, LOW);

      if (sound_pattern & 0x02)
        digitalWrite(SOUND_TRIGGER_2_PORT, HIGH);
      else
        digitalWrite(SOUND_TRIGGER_2_PORT, LOW);
      
      if (sound_pattern & 0x04)
        digitalWrite(SOUND_TRIGGER_3_PORT, HIGH);
      else
        digitalWrite(SOUND_TRIGGER_3_PORT, LOW);
  }
  tigger_sound = false;
}

//
// Setup()
//

boolean verbose = 2;  // verbose level: 0=off, 1=ping, 2=I2C, 3=LED fails

void setup() {

  Serial.begin(9600);           // start serial for output
  delay(1000);
  Serial.println("Init Start...");

  // init LED 1
  Serial.println("Init LED 1");
  tm1637_1.init();
  tm1637_1.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  
  // init LED 2
  Serial.println("Init LED 2");
  tm1637_2.init();
  tm1637_2.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;

  // init pan/tilt
  pantilt_pan  = 128;
  pantilt_tilt = 128;
  write_pan_tilt();

  // init led-rgb
  ledrgb_r = 0; ledrgb_g = 0; ledrgb_b = 0;
  write_led_rgb();

  // init sound 
  sound_pattern=SOUND_QUIET;
  write_sound();

  // init Neopixels
  neo_pattern=NEOPIXEL_QUIET;
  write_neopixel();

  // init the I2c slave
  if (ENABLE_I2C) {
    Serial.println("Init I2C");
    Wire.begin(SATELLITE_I2C_ADDRESS); // Set I2C slave address
    Wire.onReceive(receiveEvent); // register the I2C receive event
  }

  Serial.println("Setup Done!");

  // preset the outputs
  preset_outputs();

  Serial.println("");
  Serial.println("Unit Test Commands:");
  Serial.println("  d : display status, increment deviced");
  Serial.println("  r : reset the timing counters");
  Serial.println("  i : init the displays");
  Serial.println("  v : toggle the verbose level (0=off)");
  Serial.println("");

}

//
// preset_outputs()
//

void preset_outputs() {
  char str[20];

  // set the LEDs
  strcpy(str,"high"); tm1637_1.display(str);
  strcpy(str,"fast"); tm1637_2.display(str);

  // present the LED RGB
  // TODO ###############

  // present the sound
  // TODO ###############

  // present the NeoPixles
  // TODO ###############
}

//
// Loop()
//

int loop_count=0;
int ping_count=0;
void loop() {

  // how long were we gone?
  ave_loop.setStop();

  // Loop 10 times a second
  delay(100);

  // process any debugging commands
  if (Serial.available()) {
    char char_out=char(Serial.read());
 
    if ('d' == char_out) {
      display_debug();
    }

    if ('r' == char_out) {
      ave_i2c.reset();
    }

    if ('i' == char_out) {
      preset_outputs();
    }

    if ('v' == char_out) {
      if (++verbose > 2) verbose=0;
      Serial.print("** Verbose = ");
      Serial.println(verbose);
    }

  }

  // execute requested asynchronous updates
  if (tigger_led1) write_led1();
  if (tigger_led2) write_led2();
  if (tigger_pan) write_pan_tilt();
  if (tigger_ledrgb) write_led_rgb();
  if (tigger_neo) write_neopixel();
  if (tigger_sound) write_sound();

  // let us know that we are still live
  if (++loop_count>=30) {
    loop_count=0;
    if (verbose > 0) {
      Serial.print("ping:");
      Serial.println(ping_count);
    }
    ping_count++;
  }

  // capture the outside loop overhead
  ave_loop.setStart();
}

//
// I2C Handler
//

/* I2C verbose */
void display_cmnd(int howMany,  int read_count,  byte *buffer) {
  int i;
  
  if (verbose > 1) {
    Serial.print("I2C(");
    Serial.print(howMany);
    Serial.print(") Cmnd=");
    Serial.print((char) buffer[0]);
    for (i=1;i<read_count;i++) {
      Serial.print(",");
      Serial.print(buffer[i]);
    }
    Serial.println("");
  }
}

/* Receive event from Wire's I2C master */
void receiveEvent(int howMany) {

  byte buffer[I2C_READ_MAX];
  int read_count=0;

  if (ENABLE_TIMING) ave_i2c.setStart();
    
  while ((0 < Wire.available()) && (read_count < I2C_READ_MAX)) {
    buffer[read_count++] = Wire.read();
  }

  if (0 < read_count) {

    if ('1' == (char) buffer[0]) {
    led1_a = buffer[1];  led1_b = buffer[2]; led1_c = buffer[3]; led1_d = buffer[4];
    tigger_led1 = true;
    }

    if ('2' == (char) buffer[0]) {
    led2_a = buffer[1];  led2_b = buffer[2]; led2_c = buffer[3]; led2_d = buffer[4];
    tigger_led2 = true;
    }

    if ('p' == (char) buffer[0]) {
    pantilt_pan  = buffer[1];
    pantilt_tilt = buffer[2];
    tigger_pan = true;
    }
    
    if ('n' == (char) buffer[0]) {
      neo_pattern = buffer[1];
      tigger_neo = true;
    }
    
    if ('s' == (char) buffer[0]) {
      sound_pattern = buffer[1];
      tigger_sound = true;
    }
    
    if ('l' == (char) buffer[0]) {
      sound_pattern = buffer[1];
      ledrgb_r = buffer[1];
      ledrgb_g = buffer[2];
      ledrgb_b = buffer[3];
      tigger_ledrgb = true;
    }

  }

  if (ENABLE_TIMING) ave_i2c.setStop();
  display_cmnd(howMany,read_count,buffer);
}

//
// unit test increment and display
//

void display_debug() {
  static int led1=1234;
  static int led2=765;
  static int pan=1;
  static int sound=SOUND_QUIET;
  static int neo=NEOPIXEL_QUIET;
  
  Serial.println("Unit test features ...");

  led1_a = (led1/1000) % 10;
  led1_b = (led1/100 ) % 10;
  led1_c = (led1/10  ) % 10;
  led1_d =  led1       % 10;
  write_led1();
  led1++;

  led2_a = (led2/1000) % 10;
  led2_b = (led2/100 ) % 10;
  led2_c = (led2/10  ) % 10;
  led2_d =  led2       % 10;
  write_led2();
  led2++;

  if (0 == pan) {
    pantilt_pan  = 40;
    pantilt_tilt = 60;
    write_pan_tilt();
    pan = 1;
  } else {
    pantilt_pan  = 60;
    pantilt_tilt = 40;
    write_pan_tilt();
    pan = 0;
  }      
 
  // increment the sound pattern
  if (++sound >= SOUND_MAX) sound=SOUND_QUIET;
  sound_pattern = sound;
  write_sound();

  // increment the Neopixel pattern
  if (++neo >= NEOPIXEL_MAX) neo=NEOPIXEL_QUIET;
  neo_pattern = neo;
  write_neopixel();

  // display LED displays timeout/recovery counts
  Serial.print("LED_1 Timeouts:");
  Serial.println(tm1637_1.ackFailCnt);
  Serial.print("LED_2 Timeouts:");
  Serial.println(tm1637_2.ackFailCnt);

  // display timing summaries
  ave_loop.displayResults("loop",true);
  if (ENABLE_TIMING) ave_i2c.displayResults("i2c",true);
}


