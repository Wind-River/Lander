// rocket_satellite.ino: Rocket satellite Arduino instance

#include <Wire.h>
#include "TM1637.h"

// General Enbles
#define ENABLE_I2C      1
#define ENABLE_LED1     1   // setup will hang if device is not connected/working
#define ENABLE_LED2     1   // setup will hang if device is not connected/working
#define ENABLE_PAN_TILT 1
#define ENABLE_SOUND    0
#define ENABLE_NEOPIXEL 0
#define ENABLE_LEDRGB   0

// setup I2C port
#define SATELLITE_I2C_ADDRESS 18
#define I2C_READ_MAX 20

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

// Neopixel
#define Neo_Pixel_PORT  13

// Pan and Tilt
#define PWM_PAN_PORT  10
#define PWM_TILT_PORT 11

// LED-RGB
#define LED_RGB_PORT  12

void set_pan_tilt(byte pan,byte tilt);

void setup() {

  Serial.begin(9600);           // start serial for output
  delay(1000);
  Serial.println("Init Start...");

  if (ENABLE_LED1) {
    Serial.println("Init LED 1");
    tm1637_1.init();
    tm1637_1.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  }
  
  if (ENABLE_LED2) {
    Serial.println("Init LED 2");
    tm1637_2.init();
    tm1637_2.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  }

  if (ENABLE_PAN_TILT) {
    set_pan_tilt(128,128);
  }

  if (ENABLE_I2C) {
    Serial.println("Init I2C");
    Wire.begin(SATELLITE_I2C_ADDRESS); // Set I2C slave address
    Wire.onReceive(receiveEvent); // register the I2C receive event
  }


  Serial.println("Setup Done!");
}

void display_debug();

int loop_count=0;
int ping_count=0;
void loop() {
  delay(100);

  if (Serial.available()) {
    char char_out=char(Serial.read());
 
    if ('d' == char_out) {
      display_debug();
    }
  }
 
  if (++loop_count>=30) {
    loop_count=0;
    Serial.print("ping:");
    Serial.println(ping_count);
    ping_count++;
  }
}

void set_led1(byte a,byte b,byte c,byte d) {
  if (ENABLE_LED1) {
    tm1637_1.display(0,a);
    tm1637_1.display(1,b);
    tm1637_1.display(2,c);
    tm1637_1.display(3,d);
  }
}

void set_led2(byte a,byte b,byte c,byte d) {
  if (ENABLE_LED2) {
    tm1637_2.display(0,a);
    tm1637_2.display(1,b);
    tm1637_2.display(2,c);
    tm1637_2.display(3,d);
  }
}

void set_pan_tilt(byte pan,byte tilt) {
  if (ENABLE_PAN_TILT) {
    analogWrite(PWM_PAN_PORT , pan);
    analogWrite(PWM_TILT_PORT, tilt);
  }
}

void set_led_rgb(byte r,byte g,byte b) {
  if (ENABLE_LEDRGB) {
  }
}

void set_neopixel(byte pattern) {
  if (ENABLE_NEOPIXEL) {
  }
}

void set_sound(byte sound) {
  if (ENABLE_SOUND) {
      if (sound & 0x01)
        digitalWrite(SOUND_TRIGGER_1_PORT, HIGH);
      else
        digitalWrite(SOUND_TRIGGER_1_PORT, LOW);

      if (sound & 0x02)
        digitalWrite(SOUND_TRIGGER_2_PORT, HIGH);
      else
        digitalWrite(SOUND_TRIGGER_2_PORT, LOW);
      
      if (sound & 0x04)
        digitalWrite(SOUND_TRIGGER_3_PORT, HIGH);
      else
        digitalWrite(SOUND_TRIGGER_3_PORT, LOW);
  }
}

// verbose 
void display_cmnd(int howMany,  int read_count,  byte *buffer) {
  int i;
  
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

// Receive event from Wire's I2C master
void receiveEvent(int howMany) {

  byte buffer[I2C_READ_MAX];
  int read_count=0;

    
  while ((0 < Wire.available()) && (read_count < I2C_READ_MAX)) {
    buffer[read_count++] = Wire.read();
  }

  if (0 < read_count) {
    display_cmnd(howMany,read_count,buffer);

    if ('1' == (char) buffer[0]) {
      set_led1(buffer[1],buffer[2],buffer[3],buffer[4]);
    }
    if ('2' == (char) buffer[0]) {
      set_led2(buffer[1],buffer[2],buffer[3],buffer[4]);
    }
    if ('p' == (char) buffer[0]) {
      set_pan_tilt(buffer[1],buffer[2]);
    }
    if ('n' == (char) buffer[0]) {
      set_neopixel(buffer[1]);
    }
    if ('s' == (char) buffer[0]) {
      set_sound(buffer[1]);
    }
    if ('l' == (char) buffer[0]) {
      set_led_rgb(buffer[1],buffer[2],buffer[3]);
    }
  }
}

void display_debug() {
  static int led1=1234;
  static int led2=765;
  static int pan=1;
  static int sound=0;

  Serial.println("Debug features:");


  set_led1((led1/1000) % 10,(led1/100) % 10,(led1/10) % 10,led1 % 10);
  led1++;

  set_led2((led2/1000) % 10,(led2/100) % 10,(led2/10) % 10,led2 % 10);
  led2++;

  if (0 == pan) {
    set_pan_tilt(40,60);
    pan = 1;
  } else {
    set_pan_tilt(60,40);
    pan = 0;
  }      
 
  set_sound(sound);
  sound = (sound + 1) & 0x07;
}

