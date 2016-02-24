// rocket_motor.ino: Rocket motor mover Arduino instance

/* <legal-notice>
*
* Copyright (c) 2016 Wind River Systems, Inc.
*
* This software has been developed and/or maintained under the Wind River 
* CodeSwap program. The right to copy, distribute, modify, or otherwise 
* make use of this software may be licensed only pursuant to the terms
* of an applicable Wind River license agreement.
* 
* <credits>
*   { David Reyna,  david.reyna@windriver.com,  },
* </credits>
*
* </legal-notice>
*/

/* rocket_motor.ino: Theory of Operation
 * 
 * [Main Loop]
 *   - The main loop cycles as fast a possible, counting the micro seconds passed
 *   - The stepper movementment is divided across the 4 micro-steps, to provide the smoothest motion
 *   - Each motor has a micro second timeout, at which time the next micro-step is taken
 *   - The timeout is calclualted for the number of micro-steps needed for the coming timeframe
 *   - The timeframe is nominally 1/5 second, the time between updates from the main Rocket controller
 * 
 * [I2C Handler]
 *   - There I2C messages are received from the main Rocket controller to control motion:
 *     's' : Stop
 *     'g' : Go 
 *     'p' : Preset where the rocket is now (e.g. calibration initial position from master)
 *     'd' : go to a new Destination (e.g. goto to game start location)
 *     'n' : Next move increment (e.g. joystick change)
 *     '1'..'4': send 32-bit location value to respective motor (for 'p' and 'd')
 *     
 * [Serial Monitor:Debug]
 *   - Several commands are provided to do board bringup and test functionality. See 'show_help()'
 *   - The user's debug commands always run at the highest verbose level
 *   - Increase the verbose level while testing master Rocket board motor communicaton
 *   
*/


#include <Wire.h>
#include <inttypes.h>
#include "MicroAve.h"

// General Enbles
#define ENABLE_I2C      1   // Enable the I2C slave mode
#define ENABLE_TIMING   1   // enable the debug timing overhead

// setup I2C port
#define ROCKET_MOTOR_I2C_ADDRESS 19
#define I2C_READ_MAX 20

//
// which Arduino
//

#define IS_MICRO
#undef  IS_UNO

/* ======== ARDUINO MICRO PORT ASSIGNMENTS ========= */
#ifdef IS_MICRO
#define TOWER_NW_A_PORT 4
#define TOWER_NW_B_PORT 5
#define TOWER_NE_A_PORT 6
#define TOWER_NE_B_PORT 7
#define TOWER_SW_A_PORT 8
#define TOWER_SW_B_PORT 9
#define TOWER_SE_A_PORT 10
#define TOWER_SE_B_PORT 11
#endif 

/* ======== ARDUINO UNO PORT ASSIGNMENTS ========= */
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
// Global variables
//

/* Timing objects */
MicroAve ave_i2c;
MicroAve ave_loop;

/* Debugging */
#define VERBOSE_MAX 3
uint8_t verbose = 1;  // verbose level: 0=off, 1=ping, 2=I2C,motor updates
#define PING_LOOP_COUNT 10000000L // display ping every 10 seconds
#define PING_LOOP_INIT   2000000L // display first ping at 2 seconds

/* display motor positions after 3 seconds of non-activity */
#define MOVEMENT_DISPLAY_COUNT 3000000L
int32_t movement_display_counter = 0;  // coutdown pause delay

//
// motor class
//

#define FRAMES_PER_SECOND        5  // 1/5 second, as per rocket control cpu
#define USECONDS_PER_FRAME 200000L
#define MOTOR_SPEED_MAX         50  // max steps per frame
#define MOTOR_SPEED_AUTO         0  // speed is auto-calculated per frame
#define MOTOR_SPEED_SLOW       300  // usec per step
#define MOTOR_SPEED_FAST       100  // usec per step
#define MOTOR_DEST_MIN           0  // minimum (micro) step count
#define MOTOR_DEST_MAX     314120L  // maximum (micro) step count = ((1000000 um) / (12.734 um per step)) * (4 micro/step)

#define ROCKET_TOWER_STEPS_PER_UM10  785L  // 78.525 uM per step

class Motor {
  public:
    // constructors:
    Motor(const char * name, uint16_t pin_a,uint16_t pin_b);

    void displayStatus();
    void reset();

    void move_stop();
    void move_go();
    void move_preset();
    void move_destination();
    void set_location(uint32_t next_location);
   
    void move_next(int32_t a,uint32_t speed);
    void step_loop(uint16_t u_sec_passed);

    const char * name;        // name of this motor's tower
    uint16_t pin_a;
    uint16_t pin_b;
    
    int32_t step_location;    // current location of stepper motor, in micro-steps
    int32_t step_destination; // goal location of stepper motor, in micro-steps
    
    uint16_t step_speed_timecount;      // speed loop current useconds countdown
    uint16_t step_speed_timecount_max;  // speed loop total count, in countdown useconds

    boolean request_go;         // flag that a go is requested
    boolean request_stop;       // flag that a stop is requested
    boolean request_move;       // flag that a new move is ready
    boolean request_preset;     // flag that location is to be preset
    boolean request_destination;// flag that destination is to be set
    int32_t req_step_location;  // pending set location of stepper motor, in micro-steps
    int32_t req_step_increment; // pending step increment value
    int32_t req_step_speed_timecount_max;     // pending speed, in countdown useconds

    boolean enable_motion;       // stop or go

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
  step_speed_timecount = 0;       
  step_speed_timecount_max = 0;
  request_go = false;
  request_stop = false;
  request_move = false;
  request_preset = false;
  request_destination = false;
  req_step_increment = 0; 
  req_step_location = 0;
  req_step_speed_timecount_max = 0;
  // class variables
  enable_motion=true;
}

/* set the motor's absolution location */
void Motor::set_location(uint32_t next_location) {
  req_step_location = next_location << 2; // move to micro-steps
  request_move = true;

  if (verbose > 1) {
    Serial.print("Set_Location[");
    Serial.print(name);
    Serial.print("]=");
    Serial.print(next_location);
  }
}

/* cancel any pending motion */
void Motor::move_stop() {
  request_stop = true;
}

/* start/resume any pending motion */
void Motor::move_go() {
  request_go = true;
}

/* preset current location and destination to specific value */
void Motor::move_preset() {
  request_preset = true;
}

/* preset current destination to specific value */
void Motor::move_destination() {
  request_preset = true;
}


/* convert i2c bytes to 32-bit signed integer */
void Motor::move_next(int32_t request_step_increment,uint32_t request_speed) {
  req_step_increment = request_step_increment;
  req_step_speed_timecount_max = request_speed;
  request_move = true;

  if (verbose > 1) {
    Serial.print("Move_Next[");
    Serial.print(name);
    Serial.print("]=");
    Serial.print(request_step_increment);
    Serial.print(",Speed=");
    if (request_speed == MOTOR_SPEED_AUTO)
      Serial.println("auto");
    else
      Serial.println(request_speed);
  }
}

void Motor::step_loop(uint16_t u_sec_passed) {

  // Synchronously process pending motor commands here
  if (request_stop) {
    request_stop = false;
    enable_motion = false;
    // declare current location as the destination
    // step_destination =  step_location;
    movement_display_counter = MOVEMENT_DISPLAY_COUNT;
  } else if (request_go) {
    request_go = false;
    enable_motion = true;
    movement_display_counter = MOVEMENT_DISPLAY_COUNT;
  } else if (request_preset) {
    request_preset = false;
    step_location    = req_step_location;   
    step_destination = req_step_location;   
    movement_display_counter = MOVEMENT_DISPLAY_COUNT;
  } else if (request_destination) {
    request_destination = false;
    step_destination = req_step_location; 
    request_move = true;
    movement_display_counter = MOVEMENT_DISPLAY_COUNT;
  }
  
  if (request_move) {
    request_move = false;
    if (req_step_increment) {
      // update destination by increment
      step_destination += (req_step_increment * 4); // convert to micro-steps
      req_step_increment = 0;
      // (re)start the activity display coutdown
      movement_display_counter = MOVEMENT_DISPLAY_COUNT;
    }
    
    // test limits
    if (step_destination < MOTOR_DEST_MIN)
      step_destination = MOTOR_DEST_MIN;
    if (step_destination > MOTOR_DEST_MAX)
      step_destination = MOTOR_DEST_MAX;
    
    // calculate new loop speed
    step_speed_timecount_max = req_step_speed_timecount_max;
    req_step_speed_timecount_max = 0;
    if (step_speed_timecount_max == MOTOR_SPEED_AUTO) {
      int32_t frame_step_count = abs(step_location - step_destination);
      if (frame_step_count) {
        step_speed_timecount_max = (USECONDS_PER_FRAME/frame_step_count);
      }
      if (verbose > 1) {
        Serial.print(" Note: [");
        Serial.print(name);
        Serial.print("] step_speed_timecount_max is set to ");
        Serial.println(step_speed_timecount_max);
      }
    }
    // enforce the speed limit
    if (step_speed_timecount_max > MOTOR_SPEED_MAX)
      step_speed_timecount_max = MOTOR_SPEED_MAX;
  
    // reset countdown counter?
//    step_speed_timecount = 0L;
  }  

  // has enough time passed for a micro-step?
  step_speed_timecount += u_sec_passed;
  if ((enable_motion                                   ) &&
      (step_location        != step_destination        ) &&
      (step_speed_timecount >= step_speed_timecount_max) ) {
    step_speed_timecount = step_speed_timecount - step_speed_timecount_max;

    if (step_location < step_destination)
      step_location++;
    else
      step_location--;

    if (false) {
      Serial.print("Move step:");
      Serial.println(step_location & 0x03L);
    }
    
    switch (step_location & 0x03L) {
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
  Serial.print("Motor : ");
  Serial.print(name);
  Serial.print("(");
  Serial.print(pin_a);
  Serial.print(",");
  Serial.print(pin_b);
  Serial.print(") location=0x");
  Serial.print(step_location,HEX);
  Serial.print(",");
  Serial.print(step_location >> 2);
  Serial.print(" steps, dest=0x");
  Serial.print(step_destination,HEX);
  Serial.print(",");
  Serial.print(step_destination >> 2);
  Serial.print(" steps, speed:");
  Serial.print(step_speed_timecount_max);
  Serial.println(" mSec");
}

//
// setup()
//

Motor motor_nw("NW",TOWER_NW_A_PORT, TOWER_NW_B_PORT);
Motor motor_ne("NE",TOWER_NE_A_PORT, TOWER_NE_B_PORT);
Motor motor_sw("SW",TOWER_SW_A_PORT, TOWER_SW_B_PORT);
Motor motor_se("SE",TOWER_SE_A_PORT, TOWER_SE_B_PORT);

void setup() {

  Serial.begin(9600);           // start serial for output
  delay(1000);
  Serial.println("Init Start...");

  if (ENABLE_I2C) {
    Serial.println("Init I2C");
    Wire.begin(ROCKET_MOTOR_I2C_ADDRESS); // Set I2C slave address
    Wire.onReceive(receiveEvent); // register the I2C receive event
  }

  Serial.println("Setup Done!");
  show_help();
  
}

void show_help() {
  Serial.println("");
  Serial.println("Rocket Motor: Unit Test Commands:");
  Serial.println("  d : display status, increment deviced");
  Serial.println("  r : reset motors and timing counters");
  Serial.println("  + : advance motors one micro-step");
  Serial.println("  - : advance motors one micro-step");
  Serial.println("  f : advance motors one revolution");
  Serial.println("  b : advance motors one revolution");
  Serial.println("  h : goto high position");
  Serial.println("  l : goto low position");
  Serial.println("  v : toggle the verbose level (default: 0=off)");
  Serial.println("");
}

// 
// loop()
//

int32_t loop_count=PING_LOOP_INIT;
int32_t ping_count=0;
uint32_t usec_last=0;

void loop() {
  uint32_t usec_now;
  uint32_t usec_diff;
  uint8_t verbose_orig = verbose;
  boolean quick_display = false;
  
  if (ENABLE_TIMING) ave_loop.setStop();

  usec_now = (uint32_t) micros();
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
    char char_in=char(Serial.read());

    // always use maximum verbose for debug commands
    verbose=VERBOSE_MAX;
 
    if ('d' == char_in) {
      display_debug();
    }

    if ('r' == char_in) {
      motor_nw.reset();
      motor_ne.reset();
      motor_sw.reset();
      motor_se.reset();
      ave_i2c.reset();
    }

    // move the motors forward one micro step
    if ('+' == char_in) {
      motor_nw.step_destination++;
      motor_ne.step_destination++;
      motor_sw.step_destination++;
      motor_se.step_destination++;
      quick_display = true;
    }
    
    // move the motors back one micro step
    if ('-' == char_in) {
      motor_nw.step_destination--;
      motor_ne.step_destination--;
      motor_sw.step_destination--;
      motor_se.step_destination--;
      quick_display = true;
    }

    if (quick_display) {
      Serial.print("== Quick micro-step display: NW=");
      Serial.print(motor_nw.step_destination);
      Serial.print(", NE=");
      Serial.print(motor_ne.step_destination);
      Serial.print(", SW=");
      Serial.print(motor_sw.step_destination);
      Serial.print(", SE=");
      Serial.println(motor_se.step_destination);
    }
        
    // move the motors forward one revolution
    if ('f' == char_in) {
      motor_nw.move_next(200,MOTOR_SPEED_AUTO);
      motor_ne.move_next(200,MOTOR_SPEED_AUTO);
      motor_sw.move_next(200,MOTOR_SPEED_AUTO);
      motor_se.move_next(200,MOTOR_SPEED_AUTO);
    }
    
    // move the motors back one revolution
    if ('b' == char_in) {
      motor_nw.move_next(-200,MOTOR_SPEED_AUTO);
      motor_ne.move_next(-200,MOTOR_SPEED_AUTO);
      motor_sw.move_next(-200,MOTOR_SPEED_AUTO);
      motor_se.move_next(-200,MOTOR_SPEED_AUTO);
    }

    // goto high position
    if ('h' == char_in) {
      uint32_t destination = 11808;
      motor_nw.move_next(destination,MOTOR_SPEED_AUTO);
      motor_ne.move_next(destination,MOTOR_SPEED_AUTO);
      motor_sw.move_next(destination,MOTOR_SPEED_AUTO);
      motor_se.move_next(destination,MOTOR_SPEED_AUTO);
      motor_nw.move_destination();
      motor_ne.move_destination();
      motor_sw.move_destination();
      motor_se.move_destination();
    }

    // goto low position
    if ('l' == char_in) {
      uint32_t destination = 15592;
      motor_nw.move_next(destination,MOTOR_SPEED_AUTO);
      motor_ne.move_next(destination,MOTOR_SPEED_AUTO);
      motor_sw.move_next(destination,MOTOR_SPEED_AUTO);
      motor_se.move_next(destination,MOTOR_SPEED_AUTO);
      motor_nw.move_destination();
      motor_ne.move_destination();
      motor_sw.move_destination();
      motor_se.move_destination();
    }
    // test available integer types
    if ('y' == char_in) {
      uint8_t my_byte= 1;
      uint16_t my_long= 1;
      uint32_t my_dlong= 1;
      for (int i=0;i<32;i++) {
        Serial.print(i);
        Serial.print(":");
        Serial.print(my_byte,HEX);
        Serial.print(",");
        Serial.print(my_long,HEX);
        Serial.print(",");
        Serial.println(my_dlong,HEX);
        my_byte <<= 1;
        my_long <<= 1;
        my_dlong <<= 1;
      }
    }

    if ('?' == char_in) {
      show_help();
    }
 
    // restore verbose level
    verbose=verbose_orig;

    if ('v' == char_in) {
      if (++verbose > VERBOSE_MAX) verbose=0;
      Serial.print("** Verbose = ");
      Serial.println(verbose);
    }
  }
  
  // update every 10 seconds
  if (0 < verbose) {
    loop_count -= usec_diff;
    if (0L >= loop_count) {
      loop_count=PING_LOOP_COUNT;
      if (1L == ping_count) show_help();
      Serial.print("ping:");
      Serial.println(ping_count);
      ping_count++;
    }
  }

  // show status of recent activity
  if (movement_display_counter && (0 < verbose)) {
    movement_display_counter -= usec_diff;
    if (0L >= movement_display_counter) {
      // show steps
      Serial.print("\n== Current: NW=");
      Serial.print(motor_nw.step_location >> 2);
      Serial.print(", NE=");
      Serial.print(motor_ne.step_location >> 2);
      Serial.print(", SW=");
      Serial.print(motor_sw.step_location >> 2);
      Serial.print(", SE=");
      Serial.println(motor_se.step_location >> 2);
      // show millimeters
      Serial.print("         mm:NW=");
      Serial.print(((motor_nw.step_location >> 2)*ROCKET_TOWER_STEPS_PER_UM10)/10000L);
      Serial.print(", NE=");
      Serial.print(((motor_ne.step_location >> 2)*ROCKET_TOWER_STEPS_PER_UM10)/10000L);
      Serial.print(", SW=");
      Serial.print(((motor_sw.step_location >> 2)*ROCKET_TOWER_STEPS_PER_UM10)/10000L);
      Serial.print(", SE=");
      Serial.println(((motor_se.step_location >> 2)*ROCKET_TOWER_STEPS_PER_UM10)/10000L);
      movement_display_counter = 0L;
    }
  }

  if (ENABLE_TIMING) ave_loop.setStart();
}

/* user debug status dump display */
void display_debug() {
  Serial.println("\nBoard status...");

  // Motor status
  motor_nw.displayStatus();
  motor_ne.displayStatus();
  motor_sw.displayStatus();
  motor_se.displayStatus();

  // display timing summaries
  if (false && ENABLE_TIMING) {
    ave_loop.displayResults("loop",true);
    ave_i2c.displayResults("i2c",true);
  }
}

// 
// Receive event from Wire's I2C master
//

/* I2C verbose */
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

    // Stop motion
    if ('s' == (char) buffer[0]) {
      motor_nw.move_stop();
      motor_ne.move_stop();
      motor_sw.move_stop();
      motor_se.move_stop();
    }

    // Go (restart) motion
    if ('g' == (char) buffer[0]) {
      motor_nw.move_go();
      motor_ne.move_go();
      motor_sw.move_go();
      motor_se.move_go();
    }

    // Preset specific location and destination
    if ('p' == (char) buffer[0]) {
      motor_nw.move_preset();
      motor_ne.move_preset();
      motor_sw.move_preset();
      motor_se.move_preset();
    }

    // Set specific destination 
    if ('d' == (char) buffer[0]) {
      motor_nw.move_destination();
      motor_ne.move_destination();
      motor_sw.move_destination();
      motor_se.move_destination();
    }

    // Next move increment
    if ('n' == (char) buffer[0]) {
      int32_t req_step_increment = (((int32_t) buffer[1]) << 8) | ((int32_t) buffer[2]);
      if (buffer[1] & 0x80) req_step_increment |= 0xffff0000L;
      motor_nw.move_next(req_step_increment,MOTOR_SPEED_AUTO);
      req_step_increment = (((int32_t) buffer[3]) << 8) | ((int32_t) buffer[4]);
      if (buffer[3] & 0x80) req_step_increment |= 0xffff0000L;
      motor_ne.move_next(req_step_increment,MOTOR_SPEED_AUTO);
      req_step_increment = (((int32_t) buffer[5]) << 8) | ((int32_t) buffer[6]);
      if (buffer[5] & 0x80) req_step_increment |= 0xffff0000L;
      motor_sw.move_next(req_step_increment,MOTOR_SPEED_AUTO);
      req_step_increment = (((int32_t) buffer[7]) << 8) | ((int32_t) buffer[8]);
      if (buffer[7] & 0x80) req_step_increment |= 0xffff0000L;
      motor_se.move_next(req_step_increment,MOTOR_SPEED_AUTO);
    }

    // set up a specific motor location and/or destination
    if (('1' <= (char) buffer[0]) && ('4' >= (char) buffer[0])) {
      uint32_t req_step_location = 
        (((uint32_t) buffer[1]) << 24) | (((uint32_t) buffer[2]) << 16) | 
        (((uint32_t) buffer[3]) <<  8) |  ((uint32_t) buffer[4]);
      if        ('1' == (char) buffer[0]) {
        motor_nw.set_location(req_step_location);
      } else if ('2' == (char) buffer[0]) {
        motor_ne.set_location(req_step_location);
      } else if ('3' == (char) buffer[0]) {
        motor_sw.set_location(req_step_location);
      } else if ('4' == (char) buffer[0]) {
        motor_se.set_location(req_step_location);
      }
    }  
  }

  if (ENABLE_TIMING) ave_i2c.setStop();
  if (verbose > 1) display_cmnd(howMany,read_count,buffer);
}

