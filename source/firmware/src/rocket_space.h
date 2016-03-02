/* rocket_space.h - Rocket Lander Game */

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
 

// Dimensions of game mechanical space, in uMeters, referenced from game center
// tower dimensions: 580 mm high, 480 mm wide, 340 mm deep
#define X_POS_MIN -240000	// in uMeters ...
#define X_POS_MAX  240000
#define Y_POS_MIN -170000
#define Y_POS_MAX  170000
#define Z_POS_MIN       0
#define Z_POS_MAX  580000

// Dimensions of game play space, in uMeters, referenced from game center
// Assume 10mm from top, 50 mm from sides
#define GAME_X_POS_MIN -190000	// in uMeters ...
#define GAME_X_POS_MAX   90000
#define GAME_Y_POS_MIN -120000
#define GAME_Y_POS_MAX  120000
#define GAME_Z_POS_MIN       0
#define GAME_Z_POS_MAX  480000

// Dimensions of game rocket, in uMeters, referenced from rocket center
// rocket mount is 22 mm x 37 mm x 15 mm high
#define ROCKET_MOUNT_X_POS_MIN -11000	// in uMeters ...
#define ROCKET_MOUNT_X_POS_MAX  11000
#define ROCKET_MOUNT_Y_POS_MIN -17500
#define ROCKET_MOUNT_Y_POS_MAX  17500
#define ROCKET_MOUNT_Z_POS_MIN      0
#define ROCKET_MOUNT_Z_POS_MAX  15000
// Scale of game space to real space
// Assume one millimeter to 1 meter => moon space of ~1000 meters square
#define SCALE_GAME_UMETER_TO_MOON_METER 1000
#define SCALE_GAME_UMETER_TO_MOON_CMETER 10

// Moon Gravity = 1.622 m/s² => 1622 uM/s
#define GRAVITY_UMETER_PER_SECOND 1622
// Safe Lander <= 1.900 m/s => 1900 uM/s
#define SAFE_UMETER_PER_SECOND 1900


// Thrust power, in mMeters per second
// Assume XY full thrust provides  5 millimeter/second
// Assume  Z full thrust provides 10 millimeter/second
#define THRUST_UMETER_INC_X  5000/TIME_LOOPS_PER_SECOND
#define THRUST_UMETER_INC_Y  5000/TIME_LOOPS_PER_SECOND
#define THRUST_UMETER_INC_Z 10000/(JOYSTICK_Z_MAX-JOYSTICK_Z_MID)

// Fuel Units
// Assume 1 millimeter/second = 1 unit
#define FUEL_SUPPLY_INIT 1000
#define FUEL_X_INC         10	// fuel unit to mMeters per second
#define FUEL_Y_INC         10	// fuel unit to mMeters per second
#define FUEL_Z_INC         20	// fuel unit to mMeters per second


// Game Space

#define ROCKET_TOWER_NW  0
#define ROCKET_TOWER_NE  1
#define ROCKET_TOWER_SW  2
#define ROCKET_TOWER_SE  3
#define ROCKET_TOWER_MAX 4

#define ROCKET_TOWER_NW_ADDR	0x80
#define ROCKET_TOWER_NE_ADDR	0x81
#define ROCKET_TOWER_SW_ADDR	0x82
#define ROCKET_TOWER_SE_ADDR	0x83

#define ROCKET_HOME_X         0 // this is the power off rocket step home position
#define ROCKET_HOME_Y         0
#define ROCKET_HOME_Z     -5000 // this is negative to provide cable slack (in um)
#define ROCKET_CALIBRATE_X	  0	// this is the calibration point rocket home position
#define ROCKET_CALIBRATE_Y	  0
#define ROCKET_CALIBRATE_Z	  0
								
// Assume  Z full movement provides 10 millimeter/second
#define ROCKET_CALIBRATE_INC_Z 10000/(JOYSTICK_Z_MAX-JOYSTICK_Z_MID)

// Tower spool lengths
// ASSUME spool circumference of 8 mM = 8000 uM
// ASSUME spool stepper count of 200
#define ROCKET_TOWER_SPOOL_CIRC   8000L
#define ROCKET_TOWER_SPOOL_STEPS  200L

// Spindle: diameter = 8 mm, circumference = 3.141 * 8 mm = 25.128, uM/step = (25.128 * 1000)/200 = 125.64 uM/step
#define ROCKET_TOWER_STEPS_PER_UM10  1256L  // 125.6 * 10 uMx10 per step (grab one more digit of integer math precision)
#define UM10_PER_MILLIMETER          10000L // 1000  * 10 uMx10 per millimeter

// Motor speed: assume auto speed
#define MOTOR_SPEED_AUTO         0  // speed is auto-calculated per frame


// Exported Structures and Funtions

struct ROCKET_TOWER_S {
	int32_t	i2c_address;	// address of the tower's stepper controller

	int32_t	pos_x;			// position of tower pulley point
	int32_t	pos_y;
	int32_t	pos_z;

	int32_t	mount_pos_x;	// offset of rouck mount point
	int32_t	mount_pos_y;
	int32_t	mount_pos_z;

	int32_t	length;			// string deploy length current
	int32_t	length_goal;	// string deploy length goal

	int32_t	step_count;		// calculated tower motor step count
	int32_t	step_diff;		// next tower motor step move

	int32_t	speed;			// stepper motor speed
};

struct ROCKET_SPACE_S {
	int32_t	rocket_x;			// current game-space rocket position, in uMeters
	int32_t	rocket_y;
	int32_t	rocket_z;

	int32_t	rocket_goal_x;		// goal game-space rocket position, in uMeters
	int32_t	rocket_goal_y;
	int32_t	rocket_goal_z;

	int32_t	rocket_delta_x;		// current game-space rocket speed, in uMeters
	int32_t	rocket_delta_y;
	int32_t	rocket_delta_z;

	int32_t	rocket_fuel;		// current fuel level, in fuel units

	int32_t	thrust_x;			// current thruster value, in fuel units
	int32_t	thrust_y;
	int32_t	thrust_z;

};

/* commands to Rocket Motor board */
#define ROCKET_MOTOR_CMD_GO     'g'
#define ROCKET_MOTOR_CMD_STOP   's'
#define ROCKET_MOTOR_CMD_PRESET 'p'
#define ROCKET_MOTOR_CMD_DEST   'd'
#define ROCKET_MOTOR_CMD_NEXT   'n'

extern struct ROCKET_SPACE_S r_space;
extern struct ROCKET_TOWER_S r_towers[ROCKET_TOWER_MAX];

extern int sqrt_cnt;

bool init_rocket_hardware();
void init_rocket_game (int32_t pos_x, int32_t pos_y, int32_t pos_z, int32_t fuel, int32_t gravity, int32_t mode);

void compute_rocket_next_position();
void compute_rocket_cable_lengths();
void move_rocket_next_position();
void simulate_move_rocket_next_position();
uint8_t query_rocket_progress();
void rocket_increment_send(int32_t increment_nw, int32_t increment_ne, int32_t increment_sw, int32_t increment_se);

void rocket_position_send();
void rocket_command_send(uint8_t command);

int32_t sqrt_with_accuracy(int32_t x, int32_t init_guess);

