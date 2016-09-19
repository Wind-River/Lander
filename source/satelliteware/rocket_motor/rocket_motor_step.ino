// rocket_motor.ino: Rocket motor mover Arduino instance

/*
 *  Copyright (c) 2016 Wind River Systems, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

/*
 * <credits>
 *   { David Reyna,  david.reyna@windriver.com,  },
 * </credits>
 *
 */

/* rocket_motor.ino: Theory of Operation
 *
 * 1) There is a tack of dir/step stepper controllers (e.g. Pololu DRV8834)
 *        4 are for the rocket tower (e.g. Pololu Bipolar, 200 Steps/Rev, 7.2v)
 *    9..12 are for the surface (e.g. S20M024S03-M7, 24 step 5 V)
 *    
 * 2) Here are the data controls
 *   (a) One control for all motors DIR input
 *   (b) Four bit select to 2x74LS138 muxes for controller STEP input
 *   (c) Same four bit select to 2x74LS151 demuxes for motor boundary switch 
 *   (d) One enable bit for the 74LS138 muxes, when the selects are stable
 *   (e) One control for all motors SLEEP input
 *   (f) One input control for all motors SLEEP input
 *   
 * 3) Main Loop
 *   - The main loop cycles as fast a possible, counting the micro seconds passed
 *   - Each motor has a micro second countdown timeout, at which time the next micro-step is taken
 *   - The timeout is calculalted for the number of steps needed, proportional to the longest motor move
 *   - The timeframe is nominally 1/5 second (the time between updates from the main Rocket controller)
 *   - A speed limit is enforced as per the motors, so a move may cross many timeframes
 *   - The motors are moved to full power when any move command is received
 *   - The motors are moved to SLEEP/STBY after 2 seconds when there is no activity
 *   
 * 4) Stepping for a given motor
 *   (a) the DIR motor bit is set
 *   (b) the 4 select bits are set
 *   (c) motor boundary input tested, if triggered motion is stopped
 *   (d) the select enable set is set, triggering the respective STEP motor bit
 *   (e) the select enable set is unset after 2 mSec
 *   
 * 5) I2C Handler
 *   - There I2C messages are received from the main Rocket controller to control motion:
 *     'S' : Stop
 *     'P' : Preset where the rocket is now (e.g. calibration initial position from master)
 *     'D' : go to a new Destination (e.g. goto to game start location)
 *     'I' : Next move increment (e.g. joystick change)
 *     '0'..'c': send 32-bit location value to respective motor (for 'p' and 'd')
 *
 * 6) Serial Monitor:Debug
 *   - Several commands are provided to do board bringup and test functionality. See 'show_help()'
 *   - The user's debug commands always run at the highest verbose level
 *   - Increase the verbose level while testing master Rocket board motor communicaton
 *
 * 7) Dimensions
 *   - All stepper dimentions are in 16 bits (to conserve memory resources)
 *   - All time dimensions are in 32 bits (in micro seconds)
 *   - No long moves are done in this driver directly because of need for continual
 *     mapping of linear motion to the tower cable spherical coordinate system
 */

#include <Wire.h>
#include <inttypes.h>
#include "MicroAve.h"

#include <Stepper.h>

// General Enables
#define ENABLE_I2C      1   // Enable the I2C slave mode

#define ENABLE_MEASURE_LOOP     1 // measure main loop time
#define ENABLE_MEASURE_I2C      0 // measure I2C processing time
#define ENABLE_MEASURE_UPDATE   0 // measure incremental movement update time
#define ENABLE_MEASURE_LATENCY  0 // measure latency for motor incement timeout versus goal

// setup I2C port
#define ROCKET_MOTOR_I2C_ADDRESS 19
#define I2C_READ_MAX 20

/* request write commands */
#define REQUEST_STOP         's'  // send stop
#define REQUEST_PRESET       'p'  // send preset
#define REQUEST_DESTINATION  'd'  // send destination
#define REQUEST_INCREMENT    'n'  // set the request mode
#define REQUEST_ROCKET_LOC   'l'  // set the request mode

#define REQUEST_COMMAND      '?'  // set the request mode

/* request read commands */
#define REQUEST_COMMAND      '?'  // set the request mode
#define REQUEST_MOVE_STATUS  'S'  // return current move status as percentage completed
#define REQUEST_TIME_STATUS  'T'  // return current move's remaining microseconds
#define REQUEST_SPEED_LIMIT  'L'  // return minimum micro seconds per step
#define REQUEST_POSITION     'P'  // return position

//
// which Arduino
//

#undef  IS_MICRO
#undef  IS_UNO
#define IS_TRINKET

/* ======== ARDUINO MICRO PORT ASSIGNMENTS ========= */
#ifdef IS_TRINKET
#define MOTOR_SELECT_0  3
#define MOTOR_SELECT_1  4
#define MOTOR_SELECT_2  5
#define MOTOR_SELECT_3N 6
#define MOTOR_SELECT_3  8
#define MOTOR_SELECT_EN 9
#define MOTOR_DIR       10
#define MOTOR_POWER_A_PIN 11   // power PWM output pin, set A (rocket)
#define MOTOR_POWER_B_PIN 12   // power PWM output pin, set B (surface)

#define MOTOR_LIMIT_0_PIN 14   // Limit input pin, set 0..7
#define MOTOR_LIMIT_8_PIN 15   // Limit input pin, set 8..15
#define MOTOR_POWER_A_LED 16   // power PWM output LED pin, set A (rocket)
#define MOTOR_POWER_B_LED 17   // power PWM output LED pin, set B (surface)
#define MOTOR_LIMIT_NO_PIN 0   // This motor does not have limit switch implemented

#endif

/* ======== ARDUINO MICRO PORT ASSIGNMENTS ========= */
#ifdef IS_MICRO
#endif

/* ======== ARDUINO UNO PORT ASSIGNMENTS ========= */
#ifdef IS_UNO
#endif

//
// Motor Configuration
//

/* Time */
#define FRAMES_PER_SECOND        5  // 1/5 second, as per rocket control cpu
#define USECONDS_PER_FRAME 200000L  // 1/5 second
#define MOTOR_SPEED_A_MAX     1250  // minimum microseconds per step => maximum speed (mSec) = 240 rpm (NOTE:1000 mSec too fast for NEMA-17)
#define MOTOR_SPEED_B_MAX     2048  // minimum microseconds per step => maximum speed (mSec) = 240 rpm (NOTE:1000 mSec too fast for NEMA-17)
#define MOTOR_SPEED_AUTO        0L  // internal message - speed is auto-calculated per frame

/* Space */
#define MOTOR_DIR_A_INVERT    true  // Clockwise turn pulls the string - invert dir so that +steps => +cable length
#define MOTOR_DIR_B_INVERT   false  // Clockwise turn raises the ground - do not invert dir so that +steps => +height

/* Motor assignments */
#define MOTOR_NW   0
#define MOTOR_NE   1
#define MOTOR_SW   2
#define MOTOR_SE   3
#define MOTOR_00   4
#define MOTOR_01   5
#define MOTOR_02   6
#define MOTOR_10   7
#define MOTOR_11   8
#define MOTOR_12   9
#define MOTOR_20  10
#define MOTOR_21  11
#define MOTOR_22  12
#define MOTOR_MAX 13

// Motor set actions
#define ACTION_NONE       0
#define ACTION_PRESET     1
#define ACTION_INCREMENT  2
#define ACTION_MOVE       3
#define ACTION_HOME       4
#define ACTION_STOP       5

#define MOTOR_FORWARD_REVOLUTION   9999L  //  internal message for testing
#define MOTOR_BACKWARD_REVOLUTION -9999L


/* display motor positions after 3 seconds of non-activity */
#define ACTIVITY_POWERDOWN_COUNT 3000000L

/* Step max: top of tower to opposite tower base
  * a = 580 mm
  * b = sqrt(480^2 + 340^2) = 588.2 mm
  * c = sqrt(580^2 + 588.2^2) = 826 mm
  * s = (826 mm * 1000 um/mm) / 125.64 uM/step = 6575 steps = 33 turns = 8 seconds
 */
#define MOTOR_DEST_MAX      6575L  // maximum step count
#define MOTOR_DEST_MIN          0  // minimum step count

/* Spindle: diameter = 8 mm, circumference = 3.141 * 8 mm = 25.128, uM/step = (25.128 * 1000)/200 = 125.64 uM/step */
#define ROCKET_TOWER_STEPS_PER_UM10  1256L  // 125.6 * 10 uMx10 per step (grab one more digit of integer math precision)
#define UM10_PER_MILLIMETER          10000L // 1000  * 10 uMx10 per millimeter


//
// Global variables
//

/* Optional Timing objects */
MicroAve *ave_i2c;
MicroAve *ave_loop;
MicroAve *ave_update;
MicroAve *ave_latency;

/* Debugging */
#define VERBOSE_MAX 3
uint8_t verbose = 1;  // verbose level: 0=off, 1=errors,ping, 2=moves, 3=I2C
#define PING_LOOP_COUNT 10000000L // display ping every 10 seconds
#define PING_LOOP_INIT   2000000L // display first ping at 2 seconds
uint8_t test_motor_min=0;
uint8_t test_motor_max=MOTOR_MAX;
/* Request command mode */
uint8_t request_command = REQUEST_MOVE_STATUS;


//
// motor controller group class
//

class MotorControllerGroup {
  public:
    // constructors:
    MotorControllerGroup(const char * motor_name, int8_t select_min, int8_t select_max, uint16_t rev_steps, 
      uint16_t micro_steps, uint8_t power_port, uint16_t maximum_speed, boolean dir_invert);

    // member functions
    void activity_trigger();
    boolean activity_loop(int32_t u_sec_passed);
    void request_action(uint8_t action);
    void request_speed(uint32_t speed);
    void power(boolean on);
    void displayStatus();

    /* member variables */
    const char * name;            // name of this motor's tower
    int8_t motor_min;             // first group motor's select
    int8_t motor_max;             // last  group motor's select
    uint16_t step_count;          // steps per revolution
    uint16_t step_micro_count;    // micro steps set in the controller
    uint16_t max_speed;           // max speed, minimum uSec per step
    boolean direction_invert;     // invert direction control
    uint8_t power_pin;            // pin for power control
    uint8_t power_led_pin;        // pin for power on LED
    boolean power_on;             // is the power on
    uint8_t pending_action;       // flag that an action is requested
    uint32_t req_microseconds_per_step;  // requested action speed, in countdown useconds
    int32_t activity_countdown;   // timoutout for activity power on
};

MotorControllerGroup::MotorControllerGroup(const char * motor_name, int8_t select_min, int8_t select_max, uint16_t rev_steps, 
  uint16_t micro_steps, uint8_t power_port, uint16_t maximum_speed, boolean dir_invert) {
  name = motor_name;
  motor_min = select_min;
  motor_max = select_max;
  step_count = rev_steps;
  step_micro_count = micro_steps;
  power_pin = power_port;
  max_speed = maximum_speed;
  power_on = false;
  direction_invert = dir_invert;
  
  activity_countdown = 0L;
  pending_action = ACTION_NONE;

  power_led_pin = (MOTOR_POWER_A_PIN == power_pin) ? MOTOR_POWER_A_LED:MOTOR_POWER_B_LED;
    

  pinMode(power_pin,OUTPUT);
  power(false);
}

void MotorControllerGroup::request_action(uint8_t action) {
  pending_action = action;
}

void MotorControllerGroup::request_speed(uint32_t speed) {
  req_microseconds_per_step = speed;
}

// Power enable is active low
void MotorControllerGroup::power(boolean on) {
  power_on = on;
  digitalWrite(power_pin    , (power_on) ? LOW:HIGH); // driver enable low
  digitalWrite(power_led_pin, (power_on) ? HIGH:LOW); // LED    enable high
  delayMicroseconds(100); 
}

void MotorControllerGroup::activity_trigger() {
    // set the motors to high power
    if (!power_on) 
      power(true);

    activity_countdown = ACTIVITY_POWERDOWN_COUNT;
}
    
boolean MotorControllerGroup::activity_loop(int32_t u_sec_passed) {
  boolean ret = false;
  // activity timeout?
  if (activity_countdown) {
    activity_countdown -= (int32_t) u_sec_passed;
    if (0L >= activity_countdown) {
      activity_countdown=0L;
      
      // set the motors to low power
      power(false);
      ret = true;
    }
  }
  return ret;
}

void MotorControllerGroup::displayStatus() {
  Serial.print("Controller(");
  Serial.print(name);
  Serial.print(") range[");
  Serial.print(motor_min);
  Serial.print(",");
  Serial.print(motor_max);
  Serial.print(") Steps(");
  Serial.print(step_count);
  Serial.print(",");
  Serial.print(step_micro_count);
  Serial.print(") MaxSpeed=");
  Serial.println(max_speed);
}


//
// motor class
//

class Motor {
  public:
    // constructors:
    Motor();

    // member functions
    void init(const char * motor_name, int8_t motor_select, MotorControllerGroup *motor_controller, int8_t motor_home_pin, int16_t motor_home_offset);
    void step_loop(uint32_t u_sec_passed);
    void displayStatus();
    void reset();
    uint32_t remaining_steps();

    /* member variables */
    const char * name;         // name of this motor's tower
    uint8_t  select;           // motor select value 0..15
    int8_t   home_pin;         // home position input pin (MOTOR_LIMIT_NO_PIN if not implemented for this motor)
    int16_t  home_offset;      // home offset from home switch trigger
    int16_t  step_location;    // current location of stepper motor, in steps
    int16_t  step_destination; // goal location of stepper motor, in steps
    uint32_t microseconds_step_count; // speed loop current useconds countdown
    uint32_t microseconds_per_step;   // speed loop total count, in countdown useconds
    MotorControllerGroup *controller; // motor controller definition for this moter

    /* pending actions */
    int16_t request_value;      // value for action

    /* error testing analytics */
    uint32_t time_step_last;    // number of microseconds since last step
    uint32_t time_step_loops;   // number of time loops since last step
    uint16_t time_step_errors;  // number of under-time step errors
    
  private:
};


Motor::Motor() {
  reset();
}

void Motor::init(const char * motor_name, int8_t motor_select, MotorControllerGroup *motor_controller, int8_t motor_home_pin, int16_t motor_home_offset) {
  name = motor_name;
  select = motor_select;
  controller = motor_controller;
  home_pin = motor_home_pin;
  home_offset = motor_home_offset;
}

void Motor::reset() {
  step_location = 0L;
  step_destination = 0L;
  request_value = 0L;
  microseconds_step_count = 0L;
  microseconds_per_step = 0L;

  time_step_last=0L;
  time_step_errors=0; 
  time_step_loops=0;
}

uint32_t Motor::remaining_steps() {
  return (uint32_t) abs(step_destination - step_location);
}

// Synchronously advance the motor positions here
void Motor::step_loop(uint32_t u_sec_passed) {
  uint32_t time_step_now;
  
  // has enough time passed for a step?
  microseconds_step_count += u_sec_passed;
  time_step_loops++;
  if ((step_location           != step_destination     ) &&
      (microseconds_step_count >= microseconds_per_step) ) {

    // capture the latency
    if (ENABLE_MEASURE_LATENCY) {
      ave_latency->addValue(microseconds_step_count - microseconds_per_step);
    }

    // undertime protection code
    time_step_now = micros();
    if (time_step_last) {
      if ((verbose > 0) && (time_step_now == time_step_last)) {
        // NOTE: verbose console sends will overlap with stepper counting, causing loop
        // delays, so there will be some loops that reach their countdown in one
        // pass, and in this test it can appear that no time passed, especially with
        // the >=4 uSec reolution. This is normal and is not an error.
      } else if (controller->max_speed > (time_step_now - time_step_last)) {
        /* not enough time - wait again */
        time_step_errors++;
        time_step_last = time_step_now;

        if (verbose > 0) {
          Serial.println("");
          Serial.print("###ERROR_TIMING:[");
          Serial.print(name);
          Serial.print("]loc=");
          Serial.print(step_location);
          Serial.print(",dest=");
          Serial.print(step_destination);
          Serial.print(",last=");
          Serial.print(time_step_last);
          Serial.print(",now=");
          Serial.print(time_step_now);
          Serial.print(",inc=");
          Serial.print(u_sec_passed);
          Serial.print(",count=");
          Serial.print(microseconds_step_count);
          Serial.print(",max=");
          Serial.print(microseconds_per_step);
          Serial.print(",loops=");
          Serial.print(time_step_loops);
          Serial.println("");     
        }

        return;
      }
    }
    time_step_last = time_step_now;
    time_step_loops=0L;

    // reset the step count to 0 to insure full time for next step
    microseconds_step_count = 0L;

    /* which motor */
    digitalWrite(MOTOR_SELECT_0, (select & 0x01) ? HIGH:LOW);
    digitalWrite(MOTOR_SELECT_1, (select & 0x02) ? HIGH:LOW);
    digitalWrite(MOTOR_SELECT_2, (select & 0x04) ? HIGH:LOW);
    digitalWrite(MOTOR_SELECT_3, (select & 0x08) ? HIGH:LOW);
    digitalWrite(MOTOR_SELECT_3N,(select & 0x08) ? LOW:HIGH);

    /* what direction */
    if (step_location < step_destination) {
      step_location++;
      digitalWrite(MOTOR_DIR, (controller->direction_invert) ? LOW:HIGH);
      delayMicroseconds(4); //wait 4 microSec
    } else {
      step_location--;
      digitalWrite(MOTOR_DIR, (controller->direction_invert) ? HIGH:LOW);
      delayMicroseconds(4); //wait 4 microSec
    }

    /* execute the step (MOTOR_SELECT_EN is active low) */
    digitalWrite(MOTOR_SELECT_EN, LOW);
    delayMicroseconds(4); //wait 4 microSec
    digitalWrite(MOTOR_SELECT_EN, HIGH);
    delayMicroseconds(4); //wait 4 microSec

    /* reset the power-off countdown */
    controller->activity_trigger();
  }
}

void Motor::displayStatus() {
  Serial.print("Motor (");
  Serial.print(name);
  Serial.print(") location=");
  Serial.print(step_location);
  Serial.print(" steps, dest=");
  Serial.print(step_destination);
  Serial.print(" steps, speed:");
  Serial.print(microseconds_per_step);
  Serial.print(" mSec, Controller=");
  Serial.print(controller->name);  
  if (time_step_errors) {
    Serial.print(", ERROR_TIMING=");
    Serial.print(time_step_errors);
  }
  Serial.println("");
}


//
// Create the motor objects
//

MotorControllerGroup rocket_group("NEMA-14", MOTOR_NW, MOTOR_SE,200, 1, MOTOR_POWER_A_PIN, MOTOR_SPEED_A_MAX, MOTOR_DIR_A_INVERT);
MotorControllerGroup ground_group("SPIRAL" , MOTOR_00, MOTOR_22, 24, 1, MOTOR_POWER_B_PIN, MOTOR_SPEED_B_MAX, MOTOR_DIR_B_INVERT);
Motor motors[MOTOR_MAX];


//
// setup()
//

void setup() {
  uint8_t i,j;

  Serial.begin(9600);           // start serial for output
  delay(1000);
  Serial.println("Init Start...");

  // set up the motor control output pins
  pinMode(MOTOR_SELECT_0,OUTPUT);
  pinMode(MOTOR_SELECT_1,OUTPUT);
  pinMode(MOTOR_SELECT_2,OUTPUT);
  pinMode(MOTOR_SELECT_3,OUTPUT);
  pinMode(MOTOR_SELECT_3N,OUTPUT);
  pinMode(MOTOR_SELECT_EN,OUTPUT);
  digitalWrite(MOTOR_SELECT_EN, HIGH); /* turn off motor step selects */
  pinMode(MOTOR_DIR,OUTPUT);
  pinMode(MOTOR_POWER_A_PIN,OUTPUT);
  pinMode(MOTOR_POWER_B_PIN,OUTPUT);
  pinMode(MOTOR_LIMIT_0_PIN,INPUT);
  pinMode(MOTOR_LIMIT_8_PIN,INPUT);
  pinMode(MOTOR_POWER_A_LED,OUTPUT);
  pinMode(MOTOR_POWER_B_LED,OUTPUT);

  // setup rocket motors
  motors[ 0].init("NW", MOTOR_NW, &rocket_group,MOTOR_LIMIT_NO_PIN,0);
  motors[ 1].init("NE", MOTOR_NE, &rocket_group,MOTOR_LIMIT_NO_PIN,0);
  motors[ 2].init("SW", MOTOR_SW, &rocket_group,MOTOR_LIMIT_NO_PIN,0);
  motors[ 3].init("SE", MOTOR_SE, &rocket_group,MOTOR_LIMIT_NO_PIN,0);

  // setup ground motors
  motors[ 4].init("00", MOTOR_00, &ground_group,MOTOR_LIMIT_0_PIN,24);
  motors[ 5].init("01", MOTOR_01, &ground_group,MOTOR_LIMIT_0_PIN,24);
  motors[ 6].init("02", MOTOR_02, &ground_group,MOTOR_LIMIT_0_PIN,24);
  motors[ 7].init("10", MOTOR_10, &ground_group,MOTOR_LIMIT_0_PIN,24);
  motors[ 8].init("11", MOTOR_11, &ground_group,MOTOR_LIMIT_8_PIN,24);
  motors[ 9].init("12", MOTOR_12, &ground_group,MOTOR_LIMIT_8_PIN,24);
  motors[10].init("20", MOTOR_20, &ground_group,MOTOR_LIMIT_8_PIN,24);
  motors[11].init("21", MOTOR_21, &ground_group,MOTOR_LIMIT_8_PIN,24);
  motors[12].init("22", MOTOR_22, &ground_group,MOTOR_LIMIT_8_PIN,24);

  test_motor_min=0;
  test_motor_max=MOTOR_MAX-1;

  if (ENABLE_I2C) {
    Serial.println("Init I2C");
    Wire.begin(ROCKET_MOTOR_I2C_ADDRESS); // Set I2C slave address
    Wire.onReceive(receiveEvent); // register the I2C receive event
    Wire.onRequest(requestEvent); // register the I2C request event
  }

  // set up measure objects
  if (ENABLE_MEASURE_I2C)     ave_i2c = new MicroAve();
  if (ENABLE_MEASURE_LOOP)    ave_loop = new MicroAve();
  if (ENABLE_MEASURE_UPDATE)  ave_update = new MicroAve();
  if (ENABLE_MEASURE_LATENCY) ave_latency = new MicroAve();

  // set the motor update refresh time thresholds
  // normal refresh is 200uS, assume 300uS means the user has paused
  if (ENABLE_MEASURE_UPDATE) ave_update->setThreshold(0,300);

  // set the step uppdate latency buckets
  if (ENABLE_MEASURE_LATENCY) {
    for (i=0;i < HISTOGRAM_MAX; i++) {
      ave_latency->setHistBucket(i,i*(256/HISTOGRAM_MAX));
    }
  }

  Serial.println("Setup Done!");
  show_help();

}


//
// Action Dispatch
//

#define ACTION_PRESET     1
#define ACTION_INCREMENT  2
#define ACTION_MOVE       3
#define ACTION_HOME       4
#define ACTION_STOP       5

/* Synchronously process pending motor commands here */
boolean action_dispatcher(MotorControllerGroup *motor_group) {
  int8_t i;
  uint32_t move_time,longest_move,diff;
  uint32_t microseconds_per_step;
  boolean request_change=false;
  boolean request_move=false;

  if (ACTION_STOP == motor_group->pending_action) {
    motor_group->pending_action = ACTION_NONE;
    for (i=motor_group->motor_min;i<=motor_group->motor_max;i++) {
      // declare current location as the destination
      motors[i].step_destination = motors[i].step_location;
    }
    motor_group->activity_trigger();
    request_change = true;
  }

  if (ACTION_PRESET == motor_group->pending_action) {
    motor_group->pending_action = ACTION_NONE;
    for (i=motor_group->motor_min;i<=motor_group->motor_max;i++) {
      motors[i].step_location    = motors[i].request_value;
      motors[i].step_destination = motors[i].request_value;
    }
    motor_group->activity_trigger();
    request_change = true;
  }

  if (ACTION_MOVE == motor_group->pending_action) {
    motor_group->pending_action = ACTION_NONE;
    for (i=motor_group->motor_min;i<=motor_group->motor_max;i++) {
      motors[i].step_destination = motors[i].request_value;
    }
    request_change = true;
    request_move = true;
  }

  if (ACTION_INCREMENT == motor_group->pending_action) {
    motor_group->pending_action = ACTION_NONE;
    for (i=motor_group->motor_min;i<=motor_group->motor_max;i++) {
      motors[i].step_destination += motors[i].request_value;
      motors[i].request_value = 0;
    }
    request_change = true;
    request_move = true;
  }

   if (request_change) {
   /* reset countdown counters for any request */
    for (i=motor_group->motor_min;i<=motor_group->motor_max;i++) {
      motors[i].microseconds_step_count = 0L;
    }
  }

  /* if no pending move request we are done here */
  if (!request_move) {
    return(request_change);
  }

  /*
   * optimize the move request across the motors
   *
   */

  /* compute the longest move and critical path */
  longest_move = 0L;
  for (i=motor_group->motor_min;i<=motor_group->motor_max;i++) {
    diff = motors[i].remaining_steps();
    if (longest_move < diff) longest_move = diff;
  }

  /* if there is no actual movement, we are done (avoid division by zero) */
  if (0 == longest_move) return(true);

  /* compute time for longest move */
  microseconds_per_step = motor_group->req_microseconds_per_step;
  if (microseconds_per_step == MOTOR_SPEED_AUTO) {
    microseconds_per_step = USECONDS_PER_FRAME/longest_move;
  }

  /* enforce the speed limit */
  if (microseconds_per_step < motor_group->max_speed)
    microseconds_per_step =  motor_group->max_speed;

 
  /* compute countdown time for each motor */
  move_time = microseconds_per_step * longest_move;
  if (verbose > 1) {
    Serial.print("MOVE[");
    Serial.print(move_time);
    Serial.print("]:");
  }
  for (i=motor_group->motor_min;i<=motor_group->motor_max;i++) {
    motors[i].microseconds_per_step = move_time / motors[i].remaining_steps();

    if (verbose > 1) {
      Serial.print(motors[i].remaining_steps());
      Serial.print(",");
      Serial.print(motors[i].microseconds_per_step);
      Serial.print(":");
    }
  }
  if (verbose > 1) {
    Serial.println("");
  }

  return(true);
}

//
// unit test rountines
//

void test_motor_move(int16_t count, int32_t speed) {
  uint8_t i;
  
  for (i=test_motor_min;i<=test_motor_max;i++) {
Serial.println("test_motor_move:");
Serial.println(i);
    if (MOTOR_FORWARD_REVOLUTION == count) {
      motors[i].request_value = motors[i].controller->step_count;
    } else if (MOTOR_BACKWARD_REVOLUTION == count) {
      motors[i].request_value = -motors[i].controller->step_count;
    } else {
      motors[i].request_value=count;
    }
    motors[i].controller->request_speed(speed);
    motors[i].controller->request_action(ACTION_INCREMENT);
  } 
}

void show_help() {
  Serial.println("");
  Serial.println("Rocket Motor: Unit Test Commands:");
  Serial.println("  d : display status, increment deviced");
  Serial.println("  r : reset motors and timing counters");
  Serial.println("  + : advance motors one step");
  Serial.println("  - : advance motors one step");
  Serial.println("  f : advance motors one revolution");
  Serial.println("  b : advance motors one revolution");
  Serial.println("  h : goto high position");
  Serial.println("  l : goto low position (home)");
  Serial.println("  v : toggle the verbose level (default: 0=off)");
  Serial.println("");
}

//
// loop()
//

int32_t loop_count=PING_LOOP_INIT;
int32_t ping_count=0;
uint32_t usec_last=0;
char pending_char=' ';

void loop() {
  uint8_t i,disp_min,disp_max;
  uint32_t usec_now;
  uint32_t usec_diff;
  uint8_t verbose_orig = verbose;
  boolean quick_display = false;

  if (ENABLE_MEASURE_LOOP) ave_loop->setStop();

  // Handle any pending change requests
  if (ACTION_NONE != rocket_group.pending_action) {
    action_dispatcher(&rocket_group);
    return;
  }
  if (ACTION_NONE != ground_group.pending_action) {
    action_dispatcher(&ground_group);
    return;
  }
    
  // get the current time
  usec_now = micros();
  /* if there was a roll over, restart timer and wait this turn out */
  if (usec_now < usec_last) {
    usec_last = usec_now;
    return;
  }
  usec_diff = usec_now - usec_last;
  usec_last = usec_now;

  // advance the individual motor states
  for (i=0;i<MOTOR_MAX;i++) {
    motors[i].step_loop(usec_diff);
  }
  
  // is there a debug request?
  if (Serial.available()) {
    char char_in=char(Serial.read());

    // always use maximum verbose for debug commands
    verbose=VERBOSE_MAX;

    // any pending conpound chars?
    if ('m' == pending_char) {
      if ('a' == char_in) {
        test_motor_min=0;
        test_motor_max=MOTOR_MAX-1;
      } else if ('r' == char_in) {
        test_motor_min=MOTOR_NW;
        test_motor_max=MOTOR_SE;
      } else if ('g' == char_in) {
        test_motor_min=MOTOR_00;
        test_motor_max=MOTOR_22;
      } else if ('0' == char_in) {
        test_motor_min=MOTOR_NW;
        test_motor_max=MOTOR_NW;
      } else if ('4' == char_in) {
        test_motor_min=MOTOR_00;
        test_motor_max=MOTOR_00;
      } else if ('c' == char_in) {
        test_motor_min=MOTOR_22;
        test_motor_max=MOTOR_22;
      }
      Serial.print("** MotorTestRange = ");
      Serial.print(test_motor_min);
      Serial.print(" to ");
      Serial.println(test_motor_max);

      char_in = ' ';
    }
    // no more pending char cases
    pending_char = ' ';

    // set the motor range (entended char command)
    if ('m' == char_in) {
      pending_char=char_in;
    }

    if ('d' == char_in) {
      Serial.println("\nBoard status...");
      // Controller status
      rocket_group.displayStatus();
      ground_group.displayStatus();
      // Motor status
      for (int8_t i=test_motor_min; i<= test_motor_max; i++) {
        motors[i].displayStatus();
      }
    }

    if ('H' == char_in) {
      Serial.println("");
      Serial.println("=== HISTOGRAMS ============");
      // display timing summaries
      if (ENABLE_MEASURE_I2C)  ave_i2c->displayResults("i2c",true);
      if (ENABLE_MEASURE_LOOP) ave_loop->displayResults("loop",true);
      if (ENABLE_MEASURE_UPDATE) ave_update->displayResults("Inc_loop",false);
      if (ENABLE_MEASURE_LATENCY) ave_latency->displayResults("Latency",true);
    }
    
    if ('r' == char_in) {
      for (i=0;i<MOTOR_MAX;i++) {
        motors[i].reset();
      }
      if (ENABLE_MEASURE_I2C) ave_i2c->reset();
      if (ENABLE_MEASURE_LOOP) ave_loop->reset();
      if (ENABLE_MEASURE_UPDATE) ave_update->reset();
      if (ENABLE_MEASURE_LATENCY) ave_latency->reset();
    }

    // move the motors forward one step
    if ('+' == char_in) {
      test_motor_move(1,MOTOR_SPEED_AUTO);
    }

    // move the motors back one step
    if ('-' == char_in) {
      test_motor_move(-1,MOTOR_SPEED_AUTO);
    }

    // move the motors forward one revolution
    if ('f' == char_in) {
      test_motor_move(MOTOR_FORWARD_REVOLUTION,MOTOR_SPEED_AUTO);
    }

    // move the motors back one revolution
    if ('b' == char_in) {
      test_motor_move(MOTOR_BACKWARD_REVOLUTION,MOTOR_SPEED_AUTO);
    }

    // move the motors forward one revolution
    if ('1' == char_in) {
      rocket_group.power(true);
      for (i=0;i<24;i++) {
        digitalWrite(MOTOR_SELECT_EN, LOW);
        delayMicroseconds(4); //wait 4 microSec
        digitalWrite(MOTOR_SELECT_EN, HIGH);
        delayMicroseconds(2048);
      }
    }
    // move the motors forward one revolution
    if ('2' == char_in) {
      test_motor_move(MOTOR_FORWARD_REVOLUTION,20000L);
    }

    // move the motors forward one revolution
    if ('3' == char_in) {
      test_motor_move(MOTOR_FORWARD_REVOLUTION,5000L);
    }

    // move the motors forward one revolution
    if ('4' == char_in) {
      test_motor_move(MOTOR_FORWARD_REVOLUTION,2500L);
    }

    // move the motors forward one revolution
    if ('5' == char_in) {
      test_motor_move(MOTOR_FORWARD_REVOLUTION,1250L);
    }

    // move the motors forward one revolution
    if ('6' == char_in) {
      test_motor_move(MOTOR_FORWARD_REVOLUTION,1000L); // 1000 too fast!
    }


    // goto high position
//    if ('h' == char_in) {
//      // ((240,170,580) to (0,0,480) (mm) = 310.6 mm = (310644 um / 125.6 um_per_step) = 2473.3 steps
//      uint32_t destination = 2473;
//      Motor::set_next(destination,destination,destination,destination,MOTOR_SPEED_AUTO);
//      Motor::move_destination();
//    }

    // goto low position
//    if ('l' == char_in) {
//      // ((240,170,580) to (0,0,0) (mm) = 554.7 mm = (554700 um / 125.6 um_per_step) = 4416.4 steps
//      uint32_t destination = 4416;
//      Motor::set_next(destination,destination,destination,destination,MOTOR_SPEED_AUTO);
//      Motor::move_destination();
//    }

    /* NOTE: 4416 - 2473 = 1943 steps @ 800 steps/second = 2.5 seconds move = 9.7 turns */


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
    loop_count -= (int32_t) usec_diff;
    if (0L >= loop_count) {
      loop_count=PING_LOOP_COUNT;
      if (1L == ping_count) show_help();
      Serial.print("ping:");
      Serial.println(ping_count);
      ping_count++;
    }
  }

  // show status of recent activity
  disp_min = MOTOR_MAX;
  disp_max = MOTOR_MAX;
  if (rocket_group.activity_loop(usec_diff)) {
    disp_min = MOTOR_NW;
    disp_max = MOTOR_SE;
  }
  if (ground_group.activity_loop(usec_diff)) {
    if (MOTOR_MAX == disp_min) disp_min = MOTOR_00;
    disp_max = MOTOR_22;
  }
  if ((disp_min != disp_max) && (verbose > 0)) {
    char buffer[20];    
    Serial.print("== OFF [ ");
    Serial.print(disp_min);
    Serial.print("]:");
    for (i=disp_min; i<= disp_max; i++) {
      sprintf(buffer,"%04d ",motors[i].step_location);
      Serial.print(buffer);
    }
    Serial.println("");
  }

  if (ENABLE_MEASURE_LOOP) ave_loop->setStart();
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

  if (ENABLE_MEASURE_I2C) ave_i2c->setStart();

  while ((0 < Wire.available()) && (read_count < I2C_READ_MAX)) {
    buffer[read_count++] = Wire.read();
  }

  if (0 < read_count) {

    // Stop motion
    if (REQUEST_STOP == (char) buffer[0]) {
      rocket_group.request_action(ACTION_STOP);
      ground_group.request_action(ACTION_STOP);
    }

    // Preset specific location and destination
    if (REQUEST_PRESET == (char) buffer[0]) {
      switch ((char) buffer[1]) {
        case 'r':  
          rocket_group.request_action(ACTION_PRESET); 
          break;
        case 'g':  
          ground_group.request_action(ACTION_PRESET); 
          break;
        case 'a':  
          rocket_group.request_action(ACTION_PRESET); 
          ground_group.request_action(ACTION_PRESET); 
          break;
      }
    }

    // Set specific destination
    if (REQUEST_DESTINATION == (char) buffer[0]) {
      switch ((char) buffer[1]) {
        case 'r':  
          rocket_group.request_action(ACTION_MOVE); 
          break;
        case 'g':  
          ground_group.request_action(ACTION_MOVE); 
          break;
        case 'a':  
          rocket_group.request_action(ACTION_MOVE); 
          ground_group.request_action(ACTION_MOVE); 
          break;
      }
    }

    // Next move increment
    if (REQUEST_INCREMENT == (char) buffer[0]) {
      int16_t req_step_increment_nw,req_step_increment_ne,req_step_increment_sw,req_step_increment_se;
      motors[MOTOR_NW].request_value = (((int16_t) buffer[1]) << 8) | ((int16_t) buffer[2]);
      motors[MOTOR_NE].request_value = (((int16_t) buffer[3]) << 8) | ((int16_t) buffer[4]);
      motors[MOTOR_SW].request_value = (((int16_t) buffer[5]) << 8) | ((int16_t) buffer[6]);
      motors[MOTOR_SE].request_value = (((int16_t) buffer[7]) << 8) | ((int16_t) buffer[8]);
      rocket_group.request_speed(MOTOR_SPEED_AUTO);
      rocket_group.request_action(ACTION_INCREMENT);

      if (ENABLE_MEASURE_UPDATE) {
        ave_update->setStop();
        ave_update->setStart();
      }
    }

    // set up a specific motor location and/or destination
    if (('0' <= (char) buffer[0]) && ('9' >= (char) buffer[0])) {
      int16_t req_step_location = (((int16_t) buffer[2]) << 8) | ((int16_t) buffer[3]);
      int16_t motor = ((buffer[0] - '0') * 10) + (buffer[1] - '0');
      motors[motor].request_value = req_step_location;
    }

    // set up a specific motor location and/or destination
    if (REQUEST_ROCKET_LOC == (char) buffer[0]){
      motors[MOTOR_NW].request_value = (((uint16_t) buffer[1]) << 8) | ((uint16_t) buffer[2]);
      motors[MOTOR_NE].request_value = (((uint16_t) buffer[3]) << 8) | ((uint16_t) buffer[4]);
      motors[MOTOR_SW].request_value = (((uint16_t) buffer[5]) << 8) | ((uint16_t) buffer[6]);
      motors[MOTOR_SE].request_value = (((uint16_t) buffer[7]) << 8) | ((uint16_t) buffer[8]);
    }

    // Set next request mode
    if ('?' == (char) buffer[0]) {
      request_command = buffer[1];
      if (verbose > 2) {
        Serial.print("*** I2C Next Request Mode=");
        Serial.println((char) request_command);
      }
    }

  }

  if (ENABLE_MEASURE_I2C) ave_i2c->setStop();
  if (verbose > 2) display_cmnd(howMany,read_count,buffer);
}

void requestEvent() {
  uint8_t i;
  uint8_t buffer[20];
  int16_t step_compare = MOTOR_DEST_MAX / 2;
  int16_t step_diff;
  int16_t step_diff_max = 0L;
  int32_t time_remain = 0L;

  if (REQUEST_MOVE_STATUS == request_command) {
    // compute average percent of location differences
    for (i=0;i<MOTOR_MAX;i++) {
      step_diff = abs(motors[i].step_location - motors[i].step_destination);
      if (step_diff_max < step_diff) step_diff_max = step_diff;
    }
    //compare this value to half of MOTOR_DEST_MAX, get percent close
    if (step_diff > step_compare) {
      step_compare=0;
    } else {
       step_compare = ((step_compare - step_diff) * 100L)/step_compare;
    }
    buffer[0] = (uint8_t) step_compare;
    Wire.write(buffer[0]); // respond with message of 1 byte
  } else if (REQUEST_POSITION == request_command) {
    buffer[ 0]= (motors[MOTOR_NW].step_location >>  8) & 0x00ffL;
    buffer[ 1]= (motors[MOTOR_NW].step_location      ) & 0x00ffL;

    buffer[ 2]= (motors[MOTOR_NE].step_location >>  8) & 0x00ffL;
    buffer[ 3]= (motors[MOTOR_NE].step_location      ) & 0x00ffL;

    buffer[ 4]= (motors[MOTOR_SW].step_location >>  8) & 0x00ffL;
    buffer[ 5]= (motors[MOTOR_SW].step_location      ) & 0x00ffL;

    buffer[ 6]= (motors[MOTOR_SE].step_location >>  8) & 0x00ffL;
    buffer[ 7]= (motors[MOTOR_SE].step_location      ) & 0x00ffL;

    Wire.write(buffer,8);
  } else if (REQUEST_TIME_STATUS == request_command) {
    // current move's remaining microseconds
    int32_t steps_max = 0L;
    int32_t steps_speed = 0L;
    for (i=0;i<MOTOR_MAX;i++) {
      if (steps_max < motors[i].remaining_steps()) {
        steps_max = motors[i].remaining_steps();
        steps_speed = motors[i].microseconds_per_step;
      }
    }
    if (0L == steps_max) {
      time_remain = 0L;
    } else {
      time_remain = steps_max;
      time_remain *= (int32_t) steps_speed;
      time_remain /= 1000L; /* convert to milliseconds */
      if (time_remain > 32000) time_remain = 32000; /* keep it to 16-bits */
    }
    buffer[ 0]= (time_remain >>  8) & 0x00ffL;
    buffer[ 1]= (time_remain      ) & 0x00ffL;
    Wire.write(buffer,2);
  } else if (REQUEST_SPEED_LIMIT == request_command) {
    // minimum micro seconds per step
    buffer[ 0]= (rocket_group.max_speed >>  8) & 0x00ffL;
    buffer[ 1]= (rocket_group.max_speed      ) & 0x00ffL;
    buffer[ 2]= (ground_group.max_speed >>  8) & 0x00ffL;
    buffer[ 3]= (ground_group.max_speed      ) & 0x00ffL;
    Wire.write(buffer,4);
  } else {
    // unknown request state
    buffer[0] = (uint8_t) '?';
    Wire.write(1); // respond with message of 1 byte
  }

}

