// rocket_motor.ino: Rocket motor mover Arduino instance

/* TO-DO
 * div zero
 * speed set
 * trigger for motor updates
 * verbose modes
 * test
 */

 
#include <Wire.h>
#include <inttypes.h>
#include "TM1637.h"
#include "MicroAve.h"

// General Enbles
#define ENABLE_I2C      1   // Enable the I2C slave mode
#define ENABLE_TIMING   1   // enable the debug timing overhead

// Timing objects
MicroAve ave_i2c;
MicroAve ave_loop;

// setup I2C port
#define SATELLITE_I2C_ADDRESS 19
#define I2C_READ_MAX 20

//
// which Arduino is this
//

#undef  IS_MICRO
#define IS_UNO

// ======== ARDUINO MICRO PORT ASSIGNMENTS =========
#ifdef IS_MICRO
#define TOWER_NW_A_PORT 3
#define TOWER_NW_B_PORT 4
#define TOWER_NE_A_PORT 5
#define TOWER_NE_B_PORT 6
#define TOWER_SW_A_PORT 7
#define TOWER_SW_B_PORT 8
#define TOWER_SE_A_PORT 9
#define TOWER_SE_B_PORT 10
#endif 


// ======== ARDUINO UNO PORT ASSIGNMENTS =========
#ifdef IS_UNO
#define TOWER_NW_A_PORT 3
#define TOWER_NW_B_PORT 4
#define TOWER_NE_A_PORT 5
#define TOWER_NE_B_PORT 6
#define TOWER_SW_A_PORT 7
#define TOWER_SW_B_PORT 8
#define TOWER_SE_A_PORT 9
#define TOWER_SE_B_PORT 10
#endif

//
// motor class
//

#define FRAMES_PER_SECOND   5  // 1/5 second, as per rocket control cpu
#define MOTOR_SPEED_MAX    50  // max steps per frame
#define MOTOR_SPEED_AUTO    0  // speed is auto-calculated per frame
#define MOTOR_SPEED_SLOW  300  // usec per step
#define MOTOR_SPEED_FAST  100  // usec per step


class Motor {
  public:
    // constructors:
    Motor(const char * name, uint16_t pin_a,uint16_t pin_b);

    void displayStatus();
    void reset();

    void set_location(uint8_t a,uint8_t b,uint8_t c,uint8_t d);
    void move_stop();
    
    void set_increment(uint32_t a,uint32_t speed);
    void step_loop(uint16_t u_sec_passed);

    const char * name;        // name of this motor's tower
    uint16_t pin_a;
    uint16_t pin_b;
    
    int32_t step_location;    // current location of stepper motor, in micro-steps
    int32_t step_destination; // goal location of stepper motor, in micro-steps
    
    uint16_t step_speed;       // speed loop total count
    uint16_t step_speed_count; // speed loop current countdown

    int32_t step_increment;   // pending step increment value
    boolean increment_ready;  // flag that a new increment is ready

  private:

};

Motor::Motor(const char * name, uint16_t pin_a,uint16_t pin_b) {
  this->name = name;
  this->pin_a = pin_a;
  this->pin_b = pin_b;
  reset();
}

void Motor::reset() {
  step_location = 0L;    
  step_destination = 0L; 
  increment_ready = false;
  step_increment = 0L;   
  step_speed = 0;       
  step_speed_count = 0;
}

/* set the motor's absolution location */
void Motor::set_location(uint8_t a,uint8_t b,uint8_t c,uint8_t d) {
  step_location = (((uint32_t) a) << 24) | (((uint32_t) b) << 16) | (((uint32_t) c) << 8) | ((uint32_t) d);
  step_location *= 4; // move to micro-steps
  step_destination = step_location;
}

/* cancel any pending motion */
void Motor::move_stop() {
  step_destination = step_location;
}


/* convert i2c bytes to 32-bit signed integer */
void Motor::set_increment(uint32_t a,uint32_t speed) {
  step_increment = 0;

  step_increment |= (((int32_t) a) << 8) | ((int32_t) b);
  if (0x80 & a) step_increment |= 0xffff0000;

  increment_ready = true;
}

#define MICRO_SECONDS_PER_FRAME  200000L  // micro-seconds per main loop frame time of 1/5 second

void Motor::step_loop(uint16_t u_sec_passed) {

  // is there a pending increment?
  if (increment_ready) {
    increment_ready = false;
    // new destimation
    step_destination += (step_increment * 4); // convert to micro-steps
    // new micro-seccond step timeout speed
    step_speed = (200000L / step_increment) / 4; // count in micro-steps
    // new countdown
    step_speed_count = 0L;
  }  

  // has enough time passed for a micro-step?
  step_speed_count += u_sec_passed;
  if (step_speed_count != step_speed) {
    step_speed_count = 0L;

    if (step_location < step_destination)
      step_location++;
    else
      step_location--;

    switch (step_location % 4) {
      case 0:  // 1010
        digitalWrite(pin_a, LOW);
        digitalWrite(pin_b, HIGH);
      break;
      case 1:  // 0110
        digitalWrite(pin_a, HIGH);
        digitalWrite(pin_b, HIGH);
      break;
      case 2:  //0101
        digitalWrite(pin_a, HIGH);
        digitalWrite(pin_b, LOW);
      break;
      case 3:  //1001
        digitalWrite(pin_a, LOW);
        digitalWrite(pin_b, LOW);
      break;
    }
  }
}

void Motor::displayStatus() {
  
}



//
// hardware asynchonous update handlers
//

boolean tigger_move = false;

//
// setup()
//

Motor motor_nw("NW",TOWER_NW_A_PORT, TOWER_NW_B_PORT);
Motor motor_ne("NE",TOWER_NW_A_PORT, TOWER_NW_B_PORT);
Motor motor_sw("SW",TOWER_NW_A_PORT, TOWER_NW_B_PORT);
Motor motor_se("SE",TOWER_NW_A_PORT, TOWER_NW_B_PORT);

void setup() {

  Serial.begin(9600);           // start serial for output
  delay(1000);
  Serial.println("Init Start...");

  if (ENABLE_I2C) {
    Serial.println("Init I2C");
    Wire.begin(SATELLITE_I2C_ADDRESS); // Set I2C slave address
    Wire.onReceive(receiveEvent); // register the I2C receive event
  }

  Serial.println("Setup Done!");

  Serial.println("");
  Serial.println("Unit Test Commands:");
  Serial.println("  d : display status, increment deviced");
  Serial.println("  r : reset the timing counters");
  Serial.println("  + : advance motors one micro-step");
  Serial.println("  - : advance motors one micro-step");
  Serial.println("  f : advance motors one mm");
  Serial.println("  b : advance motors one mm");
  Serial.println("");
  
}

// 
// loop()
//

int loop_count=0;
int ping_count=0;
unsigned long usec_last=0;

void loop() {
  unsigned long usec_now;
  unsigned long usec_diff;

  if (ENABLE_TIMING) ave_loop.setStop();

  usec_now = (unsigned long) micros();
  if (usec_now < usec_last) usec_now += 2L << 16; // micro count wrap-around
  usec_diff = usec_now - usec_last;
  usec_last = usec_now;

  // advance the motor states
  motor_nw.step_loop(usec_diff);
  motor_ne.step_loop(usec_diff);
  motor_sw.step_loop(usec_diff);
  motor_se.step_loop(usec_diff);

  // is there a debug request?
  if (Serial.available()) {
    char char_out=char(Serial.read());
 
    if ('d' == char_out) {
      display_debug();
    }

    if ('r' == char_out) {
      ave_i2c.reset();
    }

    // move the motors forward one micro step
    if ('+' == char_out) {
      motor_nw.step_destination++;
      motor_ne.step_destination++;
      motor_sw.step_destination++;
      motor_se.step_destination++;
    }
    
    // move the motors back one micro step
    if ('-' == char_out) {
      motor_nw.step_destination--;
      motor_ne.step_destination--;
      motor_sw.step_destination--;
      motor_se.step_destination--;
    }
    
    // move the motors forward one mm
    if ('f' == char_out) {
      motor_nw.step_destination += 32;
      motor_ne.step_destination += 32;
      motor_sw.step_destination += 32;
      motor_se.step_destination += 32;
    }
    
    // move the motors back one mm
    if ('b' == char_out) {
      motor_nw.step_destination -= 32;
      motor_ne.step_destination -= 32;
      motor_sw.step_destination -= 32;
      motor_se.step_destination -= 32;
    }
    
    if ('i' == char_out) {
//      init_displays();
    }

  }

  if (++loop_count>=30) {
    loop_count=0;
    Serial.print("ping:");
    Serial.println(ping_count);
    ping_count++;
//      display_debug();
  }

  if (ENABLE_TIMING) ave_loop.setStart();
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
void receiveEvent(int16_t howMany) {

  uint8_t buffer[I2C_READ_MAX];
  uint8_t read_count=0;

  if (ENABLE_TIMING) ave_i2c.setStart();
    
  while ((0 < Wire.available()) && (read_count < I2C_READ_MAX)) {
    buffer[read_count++] = Wire.read();
  }

  if (0 < read_count) {

    if ('i' == (char) buffer[0]) {
//      init_device();
    }

    if ('m' == (char) buffer[0]) {
      uint32_t new_step_increment;

      new_step_increment = (((int32_t) buffer[1]) << 8) | ((int32_t) buffer[2]);
      motor_nw.set_increment(new_step_increment,MOTOR_SPEED_AUTO);
      new_step_increment = (((int32_t) buffer[3]) << 8) | ((int32_t) buffer[4]);
      motor_ne.set_increment(new_step_increment,MOTOR_SPEED_AUTO);
      new_step_increment = (((int32_t) buffer[5]) << 8) | ((int32_t) buffer[6]);
      motor_sw.set_increment(new_step_increment,MOTOR_SPEED_AUTO);
      new_step_increment = (((int32_t) buffer[7]) << 8) | ((int32_t) buffer[8]);
      motor_se.set_increment(new_step_increment,MOTOR_SPEED_AUTO);
    }

    if ('s' == (char) buffer[0]) {
      motor_nw.move_stop();
      motor_ne.move_stop();
      motor_sw.move_stop();
      motor_se.move_stop();
    }

    // directly set the current motor locations
    if ('1' == (char) buffer[0]) {
      motor_nw.set_location(buffer[1],buffer[2],buffer[3],buffer[4]);
    }
    if ('2' == (char) buffer[0]) {
      motor_ne.set_location(buffer[1],buffer[2],buffer[3],buffer[4]);
    }
    if ('3' == (char) buffer[0]) {
      motor_sw.set_location(buffer[1],buffer[2],buffer[3],buffer[4]);
    }
    if ('4' == (char) buffer[0]) {
      motor_se.set_location(buffer[1],buffer[2],buffer[3],buffer[4]);
    }

  }

  if (ENABLE_TIMING) ave_i2c.setStop();
  display_cmnd(howMany,read_count,buffer);
}

void display_debug() {
  Serial.println("Debug: increment feature values ...");

  // display timing summaries
  ave_loop.displayResults("loop",true);
  if (ENABLE_TIMING) ave_i2c.displayResults("i2c",true);
}

