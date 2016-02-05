/* rocket_space.h - Rocket Lander Game */

/*
 * Copyright (c) 2016, Wind River Systems, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Dimensions of game play space, in uMeters, referenced from game center
// Assume a 0.50 meter square game space = 500000 uMeters
#define X_POS_MIN -500000
#define X_POS_MAX  500000
#define Y_POS_MIN -500000
#define Y_POS_MAX  500000
#define Z_POS_MIN  0
#define Z_POS_MAX  1000000

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
#define ROCKET_TOWER_NW_X		X_POS_MIN
#define ROCKET_TOWER_NW_Y 		X_POS_MAX
#define ROCKET_TOWER_NW_Z 		Z_POS_MAX
#define ROCKET_TOWER_NW_SCALE   11

#define ROCKET_TOWER_NE_ADDR	0x81
#define ROCKET_TOWER_NE_X		X_POS_MIN
#define ROCKET_TOWER_NE_Y 		X_POS_MIN
#define ROCKET_TOWER_NE_Z 		Z_POS_MAX
#define ROCKET_TOWER_NE_SCALE   11

#define ROCKET_TOWER_SW_ADDR	0x82
#define ROCKET_TOWER_SW_X		X_POS_MAX
#define ROCKET_TOWER_SW_Y 		X_POS_MAX
#define ROCKET_TOWER_SW_Z 		Z_POS_MAX
#define ROCKET_TOWER_SW_SCALE   11

#define ROCKET_TOWER_SE_ADDR	0x83
#define ROCKET_TOWER_SE_X		X_POS_MAX
#define ROCKET_TOWER_SE_Y 		X_POS_MIN
#define ROCKET_TOWER_SE_Z 		Z_POS_MAX
#define ROCKET_TOWER_SE_SCALE   11

// Tower spool lengths
// ASSUME spool radius of 5 mM = 5000 uM
// ASSUME spool stepper count of 200
// Results in step size of ((5000 uM * 3141)/1000)/200 = 15705/200 =  78.525 uM
#define ROCKET_TOWER_SPOOL_RADIUS   5000
#define ROCKET_TOWER_SPOOL_STEPS    200

// Exported Structures and Funtions

struct ROCKET_TOWER_S {
	int32_t	i2c_address;	// address of the tower's stepper controller

	int32_t	pos_x;			// position of tower pulley point
	int32_t	pos_y;
	int32_t	pos_z;

	int32_t	length;			// string deploy length current
	int32_t	length_goal;	// string deploy length goal

	int32_t	spool_circ;		// stepper's spool circumference in uM, to be divided by step size
	int32_t	move_steps;		// step count to move stepper
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

extern struct ROCKET_SPACE_S r_space;
extern struct ROCKET_TOWER_S r_towers[ROCKET_TOWER_MAX];

bool init_rocket_hardware();
void init_rocket_game (int32_t pos_x, int32_t pos_y, int32_t pos_z, int32_t fuel, int32_t gravity);

void compute_rocket_next_position();
void compute_rocket_cable_lengths();
void move_rocket_next_position();
void simulate_move_rocket_next_position();
bool test_rocket_in_position();

int32_t sqrt_with_accuracy(int32_t x, int32_t init_guess);

