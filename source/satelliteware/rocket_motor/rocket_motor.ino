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
 *   - Each motor has a micro second countdown timeout, at which time the next micro-step is taken
 *   - The timeout is calculalted for the number of steps needed, proportional to the longest motor move
 *   - The timeframe is nominally 1/5 second (the time between updates from the main Rocket controller)
 *   - A speed limit is enforced as per the motors, so a move may cross many timeframes
 *   - The motor is moved to low power when there is no activity via the STBY input
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
 * [Dimensions]
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
#define ENABLE_TIMING   1   // enable the debug timing overhead

// setup I2C port
#define ROCKET_MOTOR_I2C_ADDRESS 19
#define I2C_READ_MAX 20

/* request read commands */
#define REQUEST_COMMAND      '?'  // set the request mode
#define REQUEST_MOVE_STATUS  's'  // return current move status as percentage completed
#define REQUEST_TIME_STATUS  't'  // return current move's remaining microseconds
#define REQUEST_SPEED_LIMIT  'l'  // return minimum micro seconds per step
#define REQUEST_POSITION     'p'  // return position

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
#define TOWER_PORT_MIN  TOWER_NW_A_PORT
#define TOWER_PORT_MAX  TOWER_SE_B_PORT
#define MOTOR_POWER_PIN 12   // power PWM output pin
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
#define TOWER_PORT_MIN  TOWER_NW_A_PORT
#define TOWER_PORT_MAX  TOWER_SE_B_PORT
#define MOTOR_POWER_PIN 12   // power PWM output pin
#endif

//
// Global variables
//

/* Timing objects */
MicroAve ave_i2c;
MicroAve ave_loop;
MicroAve ave_update;
MicroAve ave_latency;

/* Debugging */
#define VERBOSE_MAX 3
uint8_t verbose = 1;  // verbose level: 0=off, 1=ping, 2=moves, 3=I2C
#define PING_LOOP_COUNT 10000000L // display ping every 10 seconds
#define PING_LOOP_INIT   2000000L // display first ping at 2 seconds

/* display motor positions after 3 seconds of non-activity */
#define MOVEMENT_DISPLAY_COUNT 3000000L
int32_t movement_display_counter = 0;  // coutdown pause delay

/* Request command mode */
uint8_t request_command = REQUEST_MOVE_STATUS;

//
// motor class
//

#define FRAMES_PER_SECOND        5  // 1/5 second, as per rocket control cpu
#define USECONDS_PER_FRAME 200000L  // 1/5 second
#define MOTOR_SPEED_MAX       1250  // minimum microseconds per step => maximum speed (mSec) = 240 rpm (NOTE:1000 mSec too fast for NEMA-17)
#define MOTOR_SPEED_AUTO         0  // speed is auto-calculated per frame

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

/* motor controller power */
#define MOTOR_POWER_OFF       0   // value of PWM signal to motor STBY control pins
#define MOTOR_POWER_ON      255   // STBY is full on
#define MOTOR_POWER_LOW     128   // STBY is half on
#define MOTOR_POWER_PIN_NULL  0   // power PWM output pin is undefined

/* motor mode flags */
#define MOTOR_CALIBRATE_MODE  0x01  // calibrate mode, disable step limits

class Motor {
  public:
    // constructors:
    Motor(const char * name, uint16_t pin_a,uint16_t pin_b);

    // member functions
    void step_loop(int32_t u_sec_passed);
    void displayStatus();
    void reset();

    // static member functions
    static void move_stop();
    static void move_go();
    static void move_preset();
    static void move_destination();
    static void set_power(uint8_t request_power);
    static boolean moveRequestPending();
    static void move_next(int16_t request_inc_nw,int16_t request_inc_ne,int16_t request_inc_sw,int16_t request_inc_se,int32_t request_speed);
    static void set_next(int16_t request_set_nw,int16_t request_set_ne,int16_t request_set_sw,int16_t request_set_se,int32_t request_speed);
    static void motor_dispatcher(Motor *motor_nw,Motor *motor_ne,Motor *motor_sw,Motor *motor_se);

    // member variables
    const char * name;        // name of this motor's tower
    uint8_t pin_a;            // notor control A
    uint8_t pin_b;            // notor control B
    int16_t step_location;    // current location of stepper motor, in steps
    int16_t step_destination; // goal location of stepper motor, in steps
    int32_t microseconds_step_count; // speed loop current useconds countdown
    int32_t microseconds_per_step;   // speed loop total count, in countdown useconds

    // static member variables
    static uint8_t power_pwm;   // power value 100=full on, 0=full off
    static uint8_t power_pin;   // pin for power control
    static uint8_t mode;        // global motor mode
    static boolean enable_motion;      // stop or go
    static boolean request_go;         // flag that a go is requested
    static boolean request_stop;       // flag that a stop is requested
    static boolean request_preset;     // flag that location is to be preset
    static boolean request_destination;// flag that destination is to be set
    static boolean request_increment;  // flag that a new incremental move is ready

    static int16_t request_increment_nw; // pending increment of stepper motor, in steps
    static int16_t request_increment_ne;
    static int16_t request_increment_sw;
    static int16_t request_increment_se;
    static int16_t request_location_nw; // pending set location of stepper motor, in steps
    static int16_t request_location_ne;
    static int16_t request_location_sw;
    static int16_t request_location_se;

    static int32_t req_microseconds_per_step; // pending speed, in countdown useconds

    static Motor *motor_critical_path;        // which motor is the timing critical path

  private:
};

// initialize Motor static members
uint8_t Motor::power_pwm=MOTOR_POWER_OFF;
uint8_t Motor::power_pin=MOTOR_POWER_PIN_NULL;
uint8_t Motor::mode=0;
boolean Motor::request_increment;
boolean Motor::request_go;
boolean Motor::request_stop;
boolean Motor::request_preset;
boolean Motor::request_destination;
boolean Motor::enable_motion;
int16_t Motor::request_increment_nw;
int16_t Motor::request_increment_ne;
int16_t Motor::request_increment_sw;
int16_t Motor::request_increment_se;
int16_t Motor::request_location_nw;
int16_t Motor::request_location_ne;
int16_t Motor::request_location_sw;
int16_t Motor::request_location_se;
int32_t Motor::req_microseconds_per_step;
Motor*  Motor::motor_critical_path=NULL;


Motor::Motor(const char * name, uint16_t pin_a,uint16_t pin_b) {
  this->name = name;
  this->pin_a = pin_a;
  this->pin_b = pin_b;

  // set up the motor control output pins
  pinMode(pin_a,OUTPUT);
  pinMode(pin_b,OUTPUT);

  // set the motor power pin (and preset the power off)
  if (MOTOR_POWER_PIN_NULL == power_pin) {
    power_pin = MOTOR_POWER_PIN;
    power_pwm = MOTOR_POWER_OFF;
    analogWrite(power_pin, power_pwm);
  }

  reset();
}

void Motor::reset() {
  step_location = 0L;
  step_destination = 0L;
  microseconds_step_count = 0;
  microseconds_per_step = 0;
  request_go = false;
  request_stop = false;
  request_increment = false;
  request_preset = false;
  request_destination = false;
  req_microseconds_per_step = 0;
  enable_motion=true;
  Motor::set_power(MOTOR_POWER_OFF);
}

/* set the global motor power */
void Motor::set_power(uint8_t request_power) {
  if ((power_pwm != request_power) && (MOTOR_POWER_PIN_NULL != power_pin)) {
    power_pwm=request_power;
    analogWrite(power_pin, power_pwm);
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

/* move current destination to specific value */
void Motor::move_destination() {
  request_destination = true;
}


void Motor::move_next(int16_t request_inc_nw,int16_t request_inc_ne,int16_t request_inc_sw,int16_t request_inc_se,int32_t request_speed) {

  request_increment_nw = request_inc_nw;
  request_increment_ne = request_inc_ne;
  request_increment_sw = request_inc_sw;
  request_increment_se = request_inc_se;
  req_microseconds_per_step = request_speed;
  request_increment = true;

  if (verbose > 1) {
    Serial.print("Move_Next=");
    Serial.print(request_inc_nw);
    Serial.print(",");
    Serial.print(request_inc_ne);
    Serial.print(",");
    Serial.print(request_inc_sw);
    Serial.print(",");
    Serial.print(request_inc_se);
    Serial.print(",Speed=");
    if (request_speed == MOTOR_SPEED_AUTO)
      Serial.println("auto");
    else
      Serial.println(request_speed);
  }
}

void Motor::set_next(int16_t request_set_nw,int16_t request_set_ne,int16_t request_set_sw,int16_t request_set_se,int32_t request_speed) {

  request_location_nw = request_set_nw;
  request_location_ne = request_set_ne;
  request_location_sw = request_set_sw;
  request_location_se = request_set_se;

  /* the respective 'preset' or 'move' command request will come is a susequent call */

  if (verbose > 1) {
    Serial.print("Set_Next=");
    Serial.print(request_set_nw);
    Serial.print(",");
    Serial.print(request_set_ne);
    Serial.print(",");
    Serial.print(request_set_sw);
    Serial.print(",");
    Serial.print(request_set_se);
    Serial.print(",Speed=");
    if (request_speed == MOTOR_SPEED_AUTO)
      Serial.println("auto");
    else
      Serial.println(request_speed);
  }
}

// Synchronously process pending motor commands here
void Motor::motor_dispatcher(Motor *motor_nw,Motor *motor_ne,Motor *motor_sw,Motor *motor_se) {
  int32_t move_time,longest_move;
  int32_t microseconds_per_step;
  int16_t diff_nw,diff_ne,diff_sw,diff_se;
  boolean request_move=false;

  if (request_stop) {
    request_stop = false;
    enable_motion = false;
    // declare current location as the destination
    motor_nw->step_destination = motor_nw->step_location;
    motor_ne->step_destination = motor_ne->step_location;
    motor_sw->step_destination = motor_sw->step_location;
    motor_se->step_destination = motor_se->step_location;
    movement_display_counter = MOVEMENT_DISPLAY_COUNT;
    return;
  }

  if (request_go) {
    request_go = false;
    enable_motion = true;
    movement_display_counter = MOVEMENT_DISPLAY_COUNT;
    return;
  }

  if (request_preset) {
    request_preset = false;
    motor_nw->step_location    = request_location_nw;
    motor_nw->step_destination = motor_nw->step_location;
    motor_ne->step_location    = request_location_ne;
    motor_ne->step_destination = motor_ne->step_location;
    motor_sw->step_location    = request_location_sw;
    motor_sw->step_destination = motor_sw->step_location;
    motor_se->step_location    = request_location_se;
    motor_se->step_destination = motor_se->step_location;
    movement_display_counter = MOVEMENT_DISPLAY_COUNT;
    return;
  }

  if (request_destination) {
    request_destination = false;
    motor_nw->step_destination = request_location_nw;
    motor_ne->step_destination = request_location_ne;
    motor_sw->step_destination = request_location_sw;
    motor_se->step_destination = request_location_se;
    request_move = true;
  }

  if (request_increment) {
    request_increment = false;
    motor_nw->step_destination += request_increment_nw;
    motor_ne->step_destination += request_increment_ne;
    motor_sw->step_destination += request_increment_sw;
    motor_se->step_destination += request_increment_se;
    request_move = true;
  }

  /* if not pending move request we are done here */
  if (!request_move) {
    return;
  }

  /*
   * optimize the move request across the motors
   *
   */

  /* enforce the step limits  (if not in calibrate mode) */
  if (false && (0x00 == (Motor::mode | MOTOR_CALIBRATE_MODE))) {
    motor_nw->step_destination = min(MOTOR_DEST_MAX,motor_nw->step_destination);
    motor_nw->step_destination = max(MOTOR_DEST_MIN,motor_nw->step_destination);
    motor_ne->step_destination = min(MOTOR_DEST_MAX,motor_ne->step_destination);
    motor_ne->step_destination = max(MOTOR_DEST_MIN,motor_ne->step_destination);
    motor_sw->step_destination = min(MOTOR_DEST_MAX,motor_sw->step_destination);
    motor_sw->step_destination = max(MOTOR_DEST_MIN,motor_sw->step_destination);
    motor_se->step_destination = min(MOTOR_DEST_MAX,motor_se->step_destination);
    motor_se->step_destination = max(MOTOR_DEST_MIN,motor_se->step_destination);
  }

  /* compute the longest move and critical path */
  diff_nw = motor_nw->step_destination - motor_nw->step_location;
  diff_ne = motor_ne->step_destination - motor_ne->step_location;
  diff_sw = motor_sw->step_destination - motor_sw->step_location;
  diff_se = motor_se->step_destination - motor_se->step_location;
  longest_move = abs(diff_nw);
  Motor::motor_critical_path = motor_nw;
  if (longest_move < abs(diff_ne)) {
    longest_move = abs(diff_ne);
    Motor::motor_critical_path = motor_ne;
  }
  if (longest_move < abs(diff_sw)) {
    longest_move = abs(diff_sw);
    Motor::motor_critical_path = motor_sw;
  }
  if (longest_move < abs(diff_se)) {
    longest_move = abs(diff_se);
    Motor::motor_critical_path = motor_se;
  }

  /* if there is no actual movement, we are done (avoid division by zero) */
  if (0 == longest_move) return;

  /* compute time for longest move */
  microseconds_per_step = req_microseconds_per_step;
  if (microseconds_per_step == MOTOR_SPEED_AUTO) {
    microseconds_per_step = USECONDS_PER_FRAME/((int32_t) longest_move);
  }

  /* enforce the speed limit */
  if (microseconds_per_step < MOTOR_SPEED_MAX)
    microseconds_per_step = MOTOR_SPEED_MAX;

  /* compute countdown time for each motor */
  move_time = microseconds_per_step * ((int32_t) longest_move);
  motor_nw->microseconds_per_step = move_time / abs((int32_t) diff_nw);
  motor_ne->microseconds_per_step = move_time / abs((int32_t) diff_ne);
  motor_sw->microseconds_per_step = move_time / abs((int32_t) diff_sw);
  motor_se->microseconds_per_step = move_time / abs((int32_t) diff_se);

  if (verbose > 0) {
    Serial.print("MOVE:(");
    Serial.print(diff_nw);
    Serial.print(",");
    Serial.print(motor_nw->microseconds_per_step);
    Serial.print("),(");
    Serial.print(diff_ne);
    Serial.print(",");
    Serial.print(motor_ne->microseconds_per_step);
    Serial.print("),(");
    Serial.print(diff_sw);
    Serial.print(",");
    Serial.print(motor_sw->microseconds_per_step);
    Serial.print("),(");
    Serial.print(diff_se);
    Serial.print(",");
    Serial.print(motor_se->microseconds_per_step);
    Serial.println(")");
  }

  /* reset countdown counters */
  motor_nw->microseconds_step_count = 0L;
  motor_ne->microseconds_step_count = 0L;
  motor_sw->microseconds_step_count = 0L;
  motor_se->microseconds_step_count = 0L;

  // set the motors to full power
  Motor::set_power(MOTOR_POWER_ON);
  movement_display_counter = MOVEMENT_DISPLAY_COUNT;
}

// Synchronously advance the motor positions here
void Motor::step_loop(int32_t u_sec_passed) {

  // has enough time passed for a step?
  microseconds_step_count += u_sec_passed;
  if ((enable_motion                                   ) &&
      (step_location           != step_destination     ) &&
      (microseconds_step_count >= microseconds_per_step) ) {

    // capture the latency
    if (ENABLE_TIMING) {
      ave_latency.addValue(microseconds_step_count - microseconds_per_step);
    }

    // microseconds_step_count = microseconds_step_count - microseconds_per_step;
    microseconds_step_count = 0L;

    if (step_location < step_destination)
      step_location++;
    else
      step_location--;

    switch ((step_location & 0x03L) /* ^ 0x03 */ ) {
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

boolean Motor::moveRequestPending() {
    return (request_increment || request_preset || request_destination);
}

void Motor::displayStatus() {
  Serial.print("Motor : ");
  Serial.print(name);
  Serial.print("(");
  Serial.print(pin_a);
  Serial.print(",");
  Serial.print(pin_b);
  Serial.print(") location=");
  Serial.print(step_location);
  Serial.print(" steps, dest=");
  Serial.print(step_destination);
  Serial.print(" steps, speed:");
  Serial.print(microseconds_per_step);
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
  uint8_t i,j;

  Serial.begin(9600);           // start serial for output
  delay(1000);
  Serial.println("Init Start...");

  if (ENABLE_I2C) {
    Serial.println("Init I2C");
    Wire.begin(ROCKET_MOTOR_I2C_ADDRESS); // Set I2C slave address
    Wire.onReceive(receiveEvent); // register the I2C receive event
    Wire.onRequest(requestEvent); // register the I2C request event
  }

  // set the motor update refresh time thresholds
  // normal refresh is 200uS, assume 300uS means the user has paused
  ave_update.setThreshold(0,300);

  // set the step uppdate latency buckets
  for (i=0;i < HISTOGRAM_MAX; i++) {
    ave_latency.setHistBucket(i,i*(256/HISTOGRAM_MAX));
  }

  Serial.println("Setup Done!");
  show_help();

}

//
// dist(); cable distances, mm to steps, for self test
//

int16_t dist2steps(int16_t tx,int16_t ty,int16_t tz,int16_t x,int16_t y,int16_t z) {
  int32_t dist;
  dist = sqrt(sq(tx - x) + sq(ty - y) + sq(tz - z));
  dist = (dist * 1000L)/126;
  return (dist);
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
int32_t usec_last=0;

void loop() {
  int32_t usec_now;
  int32_t usec_diff;
  uint8_t verbose_orig = verbose;
  boolean quick_display = false;

  if (ENABLE_TIMING) ave_loop.setStop();

  // Handle any pending move change requests
  Motor::motor_dispatcher(&motor_nw,&motor_ne,&motor_sw,&motor_se);

  // carefully extend uint_16 micro count to common base int_32 for time
  usec_now = micros();
  if (usec_now < usec_last) {
    // 32-bit roll over
    usec_diff = (0xffffffff - usec_last) + 1 + usec_now;
  } else {
    usec_diff = usec_now - usec_last;
  }
  usec_last = usec_now;

  // advance the individual motor states
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

    if ('H' == char_in) {
      display_debug_histograms();
    }
    
    if ('r' == char_in) {
      motor_nw.reset();
      motor_ne.reset();
      motor_sw.reset();
      motor_se.reset();
      ave_i2c.reset();
      ave_loop.reset();
      ave_update.reset();
      ave_latency.reset();
    }

    // move the motors forward one step
    if ('+' == char_in) {
      motor_nw.step_destination++;
      motor_ne.step_destination++;
      motor_sw.step_destination++;
      motor_se.step_destination++;
      quick_display = true;
    }

    // move the motors back one step
    if ('-' == char_in) {
      motor_nw.step_destination--;
      motor_ne.step_destination--;
      motor_sw.step_destination--;
      motor_se.step_destination--;
      quick_display = true;
    }

    if (quick_display) {
      Serial.print("== Quick step display: NW=");
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
      Motor::move_next(200,200,200,200,MOTOR_SPEED_AUTO);
    }

    // move the motors back one revolution
    if ('b' == char_in) {
      Motor::move_next(-200,-200,-200,-200,MOTOR_SPEED_AUTO);
    }

    // move the motors forward one revolution
    if ('1' == char_in) {
      Motor::move_next(100,100,100,100,10000);
    }

    // move the motors forward one revolution
    if ('2' == char_in) {
      Motor::move_next(100,100,100,100,20000);
    }

    // move the motors forward one revolution
    if ('3' == char_in) {
      Motor::move_next(200,200,200,200,5000);
    }

    // move the motors forward one revolution
    if ('4' == char_in) {
      Motor::move_next(200,200,200,200,2500);
    }

    // move the motors forward one revolution
    if ('5' == char_in) {
      Motor::move_next(200,200,200,200,1250);
    }

    // move the motors forward one revolution
    if ('6' == char_in) {
      Motor::move_next(200,200,200,200,1000); // 1000 too fast!
    }

    // toggle the move limit mode
    if ('m' == char_in) {
      Motor::mode ^= MOTOR_CALIBRATE_MODE;
      Serial.print("** Mode = ");
      Serial.println(Motor::mode);
    }

    // increase the standby power
    if ('p' == char_in) {
      if (Motor::power_pwm > 255-16) Motor::power_pwm = 255-16;
      Motor::set_power(Motor::power_pwm + 16);
      if (0 < verbose) {
        Serial.print("** Power = ");
        Serial.println(Motor::power_pwm);
      }
    }
    // decrease the standby power
    if ('P' == char_in) {
      if (Motor::power_pwm < 16) Motor::power_pwm = 16;
      Motor::set_power(Motor::power_pwm - 16);
      if (0 < verbose) {
        Serial.print("** Power = ");
        Serial.println(Motor::power_pwm);
      }
    }

    // goto high position
    if ('h' == char_in) {
      // ((240,170,580) to (0,0,480) (mm) = 310.6 mm = (310644 um / 125.6 um_per_step) = 2473.3 steps
      uint32_t destination = 2473;
      Motor::set_next(destination,destination,destination,destination,MOTOR_SPEED_AUTO);
      Motor::move_destination();
    }

    // goto low position
    if ('l' == char_in) {
      // ((240,170,580) to (0,0,0) (mm) = 554.7 mm = (554700 um / 125.6 um_per_step) = 4416.4 steps
      uint32_t destination = 4416;
      Motor::set_next(destination,destination,destination,destination,MOTOR_SPEED_AUTO);
      Motor::move_destination();
    }

    /* NOTE: 4416 - 2473 = 1943 steps @ 800 steps/second = 2.5 seconds move = 9.7 turns */

      // goto high NW corner position
    if ('6' == char_in) {
      Motor::set_next(
        dist2steps(-240, 170,580, -200,100,200),
        dist2steps( 240, 170,580, -200,100,200),
        dist2steps(-240,-170,580, -200,100,200),
        dist2steps( 240,-170,580, -200,100,200),
        MOTOR_SPEED_AUTO);
      Motor::move_destination();
    }
    // goto low SE corner position
    if ('7' == char_in) {
      Motor::set_next(
        dist2steps(-240, 170,580, 200,-100,0),
        dist2steps( 240, 170,580, 200,-100,0),
        dist2steps(-240,-170,580, 200,-100,0),
        dist2steps( 240,-170,580, 200,-100,0),
        MOTOR_SPEED_AUTO);
      Motor::move_destination();
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
  if (movement_display_counter) {
    movement_display_counter -= usec_diff;
    if (0L >= movement_display_counter) {

      // set the motors to low power
      Motor::set_power(MOTOR_POWER_OFF /* MOTOR_POWER_LOW */);

      if (0 < verbose) {
        // show steps
        Serial.print("\n== Current: NW=");
        Serial.print(motor_nw.step_location);
        Serial.print(", NE=");
        Serial.print(motor_ne.step_location);
        Serial.print(", SW=");
        Serial.print(motor_sw.step_location);
        Serial.print(", SE=");
        Serial.println(motor_se.step_location);
        // show millimeters
        Serial.print("         mm:NW=");
        Serial.print((motor_nw.step_location * ROCKET_TOWER_STEPS_PER_UM10)/UM10_PER_MILLIMETER);
        Serial.print(", NE=");
        Serial.print((motor_ne.step_location * ROCKET_TOWER_STEPS_PER_UM10)/UM10_PER_MILLIMETER);
        Serial.print(", SW=");
        Serial.print((motor_sw.step_location * ROCKET_TOWER_STEPS_PER_UM10)/UM10_PER_MILLIMETER);
        Serial.print(", SE=");
        Serial.println((motor_se.step_location * ROCKET_TOWER_STEPS_PER_UM10)/UM10_PER_MILLIMETER);
        movement_display_counter = 0L;
      }
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

  // power status and mode
  Serial.print("Power = ");
  Serial.print(motor_nw.power_pwm);
  Serial.print(", Mode = ");
  Serial.println(Motor::mode);
}

void display_debug_histograms() {
  // display timing summaries
  if (ENABLE_TIMING) {
    ave_loop.displayResults("loop",true);
    //ave_i2c.displayResults("i2c",true);
  }
  ave_update.displayResults("Inc_loop",false);
  ave_latency.displayResults("Latency",true);
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
      Motor::move_stop();
    }

    // Go (restart) motion
    if ('g' == (char) buffer[0]) {
      Motor::move_go();
    }

    // Preset specific location and destination
    if ('p' == (char) buffer[0]) {
      Motor::move_preset();
    }

    // Set specific destination
    if ('d' == (char) buffer[0]) {
      Motor::move_destination();
    }

    // Next move increment
    if ('n' == (char) buffer[0]) {
      int16_t req_step_increment_nw,req_step_increment_ne,req_step_increment_sw,req_step_increment_se;

      req_step_increment_nw = (((int16_t) buffer[1]) << 8) | ((int16_t) buffer[2]);
      req_step_increment_ne = (((int16_t) buffer[3]) << 8) | ((int16_t) buffer[4]);
      req_step_increment_sw = (((int16_t) buffer[5]) << 8) | ((int16_t) buffer[6]);
      req_step_increment_se = (((int16_t) buffer[7]) << 8) | ((int16_t) buffer[8]);
      Motor::move_next(req_step_increment_nw,req_step_increment_ne,req_step_increment_sw,req_step_increment_se,MOTOR_SPEED_AUTO);

      if (ENABLE_TIMING) {
        ave_update.setStop();
        ave_update.setStart();
      }
    }

    // set up a specific motor location and/or destination
    if (('1' <= (char) buffer[0]) && ('4' >= (char) buffer[0])) {
      int16_t req_step_location = (((int16_t) buffer[3]) << 8) | ((int16_t) buffer[4]);
      if        ('1' == (char) buffer[0]) {
        Motor::request_location_nw = req_step_location;
      } else if ('2' == (char) buffer[0]) {
        Motor::request_location_ne = req_step_location;
      } else if ('3' == (char) buffer[0]) {
        Motor::request_location_sw = req_step_location;
      } else if ('4' == (char) buffer[0]) {
        Motor::request_location_se = req_step_location;
      }
    }

    // set up a specific motor location and/or destination
    if ('l' == (char) buffer[0]){
      Motor::request_location_nw = (((uint16_t) buffer[1]) << 8) | ((uint16_t) buffer[2]);
      Motor::request_location_ne = (((uint16_t) buffer[3]) << 8) | ((uint16_t) buffer[4]);
      Motor::request_location_sw = (((uint16_t) buffer[5]) << 8) | ((uint16_t) buffer[6]);
      Motor::request_location_se = (((uint16_t) buffer[7]) << 8) | ((uint16_t) buffer[8]);
    }

    // Set calbration mode
    if ('C' == (char) buffer[0]) {
      Motor::mode |= MOTOR_CALIBRATE_MODE;
    }

    // Set normal mode
    if ('N' == (char) buffer[0]) {
      Motor::mode &= ~MOTOR_CALIBRATE_MODE;
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

  if (ENABLE_TIMING) ave_i2c.setStop();
  if (verbose > 2) display_cmnd(howMany,read_count,buffer);
}

void requestEvent() {
  uint8_t buffer[20];
  int16_t step_compare = MOTOR_DEST_MAX / 2;
  int16_t step_diff;
  int16_t step_diff_max = 0L;
  int32_t time_remain = 0L;

  if (REQUEST_MOVE_STATUS == request_command) {
    if (Motor::moveRequestPending()) {
        /* there is an update pending - assume zero progress */
        step_compare=0;
    } else {
      // compute average percent of location differences
      step_diff = abs(motor_nw.step_location - motor_nw.step_destination);
      if (step_diff_max < step_diff) step_diff_max = step_diff;
      step_diff = abs(motor_ne.step_location - motor_ne.step_destination);
      if (step_diff_max < step_diff) step_diff_max = step_diff;
      step_diff = abs(motor_sw.step_location - motor_sw.step_destination);
      if (step_diff_max < step_diff) step_diff_max = step_diff;
      step_diff = abs(motor_se.step_location - motor_se.step_destination);
      if (step_diff_max < step_diff) step_diff_max = step_diff;

      //compare this value to half of MOTOR_DEST_MAX, get percent close
      if (step_diff > step_compare) {
        step_compare=0;
      } else {
         step_compare = ((step_compare - step_diff) * 100L)/step_compare;
      }
    }

    buffer[0] = (uint8_t) step_compare;
    Wire.write(buffer[0]); // respond with message of 1 byte
  } else if (REQUEST_POSITION == request_command) {
    buffer[ 0]= (motor_nw.step_location >>  8) & 0x00ffL;
    buffer[ 1]= (motor_nw.step_location      ) & 0x00ffL;

    buffer[ 2]= (motor_ne.step_location >>  8) & 0x00ffL;
    buffer[ 3]= (motor_ne.step_location      ) & 0x00ffL;

    buffer[ 4]= (motor_sw.step_location >>  8) & 0x00ffL;
    buffer[ 5]= (motor_sw.step_location      ) & 0x00ffL;

    buffer[ 6]= (motor_se.step_location >>  8) & 0x00ffL;
    buffer[ 7]= (motor_se.step_location      ) & 0x00ffL;

    Wire.write(buffer,8);
  } else if (REQUEST_TIME_STATUS == request_command) {
    // current move's remaining microseconds
    if (NULL == Motor::motor_critical_path) {
      time_remain = 0L;
    } else {
      time_remain = abs(Motor::motor_critical_path->step_destination - Motor::motor_critical_path->step_location);
      time_remain *= Motor::motor_critical_path->microseconds_step_count;
      time_remain /= 1000L; /* convert to milliseconds */
      if (time_remain > 32000) time_remain = 32000; /* keep it to 16-bits */
    }
    buffer[ 0]= (time_remain >>  8) & 0x00ffL;
    buffer[ 1]= (time_remain      ) & 0x00ffL;
    Wire.write(buffer,2);
  } else if (REQUEST_SPEED_LIMIT == request_command) {
    // minimum micro seconds per step
    buffer[ 0]= (MOTOR_SPEED_MAX >>  8) & 0x00ffL;
    buffer[ 1]= (MOTOR_SPEED_MAX      ) & 0x00ffL;
    Wire.write(buffer,2);
  } else {
    // unknown request state
    buffer[0] = (uint8_t) '?';
    Wire.write(buffer[0]); // respond with message of 1 byte
  }

}

