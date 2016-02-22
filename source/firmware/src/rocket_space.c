/* rocket_space.c - Rocket Lander Game */

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
 
#include <zephyr.h>

#include <i2c.h>

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "rocket.h"
#include "rocket_space.h"


/*
 * Rocket Control Structures
 *
 */

struct ROCKET_SPACE_S r_space;

struct ROCKET_TOWER_S r_towers[ROCKET_TOWER_MAX] = {
    {
        .pos_x       = ROCKET_TOWER_NW_X,
        .pos_y       = ROCKET_TOWER_NW_Y,
        .pos_z       = ROCKET_TOWER_NW_Z,
        .i2c_address = ROCKET_TOWER_NW_ADDR,
        .spool_circ = 1
    },
    {
        .pos_x       = ROCKET_TOWER_NE_X,
        .pos_y       = ROCKET_TOWER_NE_Y,
        .pos_z       = ROCKET_TOWER_NE_Z,
        .i2c_address = ROCKET_TOWER_NE_ADDR,
        .spool_circ = 1
    },
    {
        .pos_x       = ROCKET_TOWER_SW_X,
        .pos_y       = ROCKET_TOWER_SW_Y,
        .pos_z       = ROCKET_TOWER_SW_Z,
        .i2c_address = ROCKET_TOWER_SW_ADDR,
        .spool_circ = 1
    },
    {
        .pos_x       = ROCKET_TOWER_SE_X,
        .pos_y       = ROCKET_TOWER_SE_Y,
        .pos_z       = ROCKET_TOWER_SE_Z,
        .i2c_address = ROCKET_TOWER_SE_ADDR,
        .spool_circ = 1
    },
};



/*
 * Initialize Rocket Hardware
 *
 */

bool init_rocket_hardware () {

	// TODO ################ Calibrate true rocket power-up position
	r_space.rocket_x = 0;			// current game-space rocket position, in uMeters
	r_space.rocket_y = 0;
	r_space.rocket_z = 0;

	// Compute tower spool circumference
	// ASSUME PI as (3.141 * 1000)/1000
	r_towers[ROCKET_TOWER_NW].spool_circ = (ROCKET_TOWER_SPOOL_RADIUS * 3141)/1000;
	r_towers[ROCKET_TOWER_NE].spool_circ = (ROCKET_TOWER_SPOOL_RADIUS * 3141)/1000;
	r_towers[ROCKET_TOWER_SW].spool_circ = (ROCKET_TOWER_SPOOL_RADIUS * 3141)/1000;
	r_towers[ROCKET_TOWER_SE].spool_circ = (ROCKET_TOWER_SPOOL_RADIUS * 3141)/1000;

	// Initialize XYZ motor controls
	if (IO_MOTOR_ENABLE) {
		// TODO ################
	}

	return true;
}


/*
 * Initialize Rocket Settings
 *
 */

void init_rocket_game (int32_t pos_x, int32_t pos_y, int32_t pos_z, int32_t fuel, int32_t gravity, int32_t mode)
 {

	// set initial rocket conditions

	// TODO ################### Adjust
	r_game.fuel_option = fuel;
	if        (r_game.fuel_option == GAME_FUEL_LOW) {
		r_space.rocket_fuel=FUEL_SUPPLY_INIT/2;
	} else if (r_game.fuel_option == GAME_FUEL_NOLIMIT) {
		r_space.rocket_fuel=FUEL_SUPPLY_INIT*10;
	} else {
		r_space.rocket_fuel=FUEL_SUPPLY_INIT;
	}

	// TODO ###################
	r_game.gravity_option=gravity;
	if        (r_game.gravity_option == GAME_GRAVITY_NORMAL) {
	} else if (r_game.gravity_option == GAME_GRAVITY_HIGH) {
	} else if (r_game.gravity_option == GAME_GRAVITY_NONE) {
	} else if (r_game.gravity_option == GAME_GRAVITY_NEGATIVE) {
	}

	r_space.rocket_goal_x = pos_x;		// goal game-space rocket position, in uMeters
	r_space.rocket_goal_y = pos_y;
	r_space.rocket_goal_z = pos_z;

	r_space.rocket_delta_x = 0;		// current game-space rocket speed, in uMeters
	r_space.rocket_delta_y = 0;
	r_space.rocket_delta_z = 0;

	r_space.thrust_x = 0;			// current thruster value, in fuel units
	r_space.thrust_y = 0;
	r_space.thrust_z = 0;

	r_game.game_mode = mode;

	// move to the rocket start position
	compute_rocket_next_position();
	compute_rocket_cable_lengths();
	move_rocket_next_position();

}


/*
 * compute_next_position : use vectors to compute next incremental position for rocket
 *
 */

void compute_rocket_next_position ()
 {
	int32_t	rocket_fuel_used=0;
	int32_t	rocket_thrust_inc_x=THRUST_UMETER_INC_X;
	int32_t	rocket_thrust_inc_y=THRUST_UMETER_INC_Y;
	int32_t	rocket_thrust_inc_z=THRUST_UMETER_INC_Z;

	r_space.thrust_x=0;
	r_space.thrust_y=0;
	r_space.thrust_z=0;

	if (GAME_XYZ_MOVE == r_game.game) {
		// fast absolute xy changes in 'move' mode
		rocket_thrust_inc_x = 1000;
		rocket_thrust_inc_y = 1000;
	}

	// Convert joystick to thrust values
	if (r_space.rocket_fuel > 0) {
		// Thruster X is 'on-left or 'on-right' or 'off'
		r_space.thrust_x = r_control.analog_x - JOYSTICK_X_MID;
		if (r_space.thrust_x < -JOYSTICK_DELTA_XY_MIN) {
			r_space.rocket_delta_x = -rocket_thrust_inc_x;
			rocket_fuel_used += FUEL_X_INC;
		}
		if (r_space.thrust_x > JOYSTICK_DELTA_XY_MIN ) {
			r_space.rocket_delta_x = rocket_thrust_inc_x;
			rocket_fuel_used += FUEL_X_INC;
		}

		// Thruster Y is 'on-forward or 'on-backward' or 'off'
		r_space.thrust_y = r_control.analog_y - JOYSTICK_Y_MID;
		if (r_space.thrust_y < -JOYSTICK_DELTA_XY_MIN) {
			r_space.rocket_delta_y = -rocket_thrust_inc_y;
			rocket_fuel_used += FUEL_Y_INC;
		}
		if (r_space.thrust_y > JOYSTICK_DELTA_XY_MIN ) {
			r_space.rocket_delta_y = rocket_thrust_inc_y;
			rocket_fuel_used += FUEL_Y_INC;
		}

		// Thruster Z is 'proportion-up or 'proportion-down' or 'off'
		r_space.thrust_z = r_control.analog_z - JOYSTICK_Z_MID;
		if (r_space.thrust_z < -JOYSTICK_DELTA_Z_MIN) {
			r_space.rocket_delta_z += (r_space.thrust_z+JOYSTICK_DELTA_Z_MIN)*rocket_thrust_inc_z;
			rocket_fuel_used += FUEL_Z_INC;
		}
		if (r_space.thrust_z > JOYSTICK_DELTA_Z_MIN) {
			r_space.rocket_delta_z += (r_space.thrust_z-JOYSTICK_DELTA_Z_MIN)*rocket_thrust_inc_z;
			rocket_fuel_used += FUEL_Z_INC;
		}
	}

	r_space.rocket_goal_x += r_space.rocket_delta_x;
	r_space.rocket_goal_y += r_space.rocket_delta_y;
	r_space.rocket_goal_z += r_space.rocket_delta_z;

	if (GAME_XYZ_MOVE != r_game.game) {
		// Acceleration due to gravity
		if (GAME_GRAVITY_NONE != r_game.gravity_option) {
			r_space.rocket_delta_z -= GRAVITY_UMETER_PER_SECOND;
		}
	} else {
		// Cancel any inertial and gravity motion, also any fuel usage
		r_space.rocket_delta_x = 0;
		r_space.rocket_delta_y = 0;
		r_space.rocket_delta_z = 0;
	}

	// Burn that fuel
	r_space.rocket_fuel -= rocket_fuel_used;
	if ((GAME_FUEL_NOLIMIT == r_game.fuel_option) || (GAME_XYZ_MOVE == r_game.game)) {
		if (r_space.rocket_fuel < 100) r_space.rocket_fuel = FUEL_SUPPLY_INIT;
	}

	// Assert Limits
	if (GAME_SIMULATE == r_game.game_mode) {
		if (r_space.rocket_goal_x < X_POS_MIN)  r_space.rocket_goal_x = X_POS_MIN;
		if (r_space.rocket_goal_x > X_POS_MAX)  r_space.rocket_goal_x = X_POS_MAX;
		if (r_space.rocket_goal_y < Y_POS_MIN)  r_space.rocket_goal_y = Y_POS_MIN;
		if (r_space.rocket_goal_y > Y_POS_MAX)  r_space.rocket_goal_y = Y_POS_MAX;
		if (r_space.rocket_goal_z < Z_POS_MIN)  r_space.rocket_goal_z = Z_POS_MIN;
		if (r_space.rocket_goal_z > Z_POS_MAX)  r_space.rocket_goal_z = Z_POS_MAX;
	} else {
		if (r_space.rocket_goal_x < GAME_X_POS_MIN)  r_space.rocket_goal_x = GAME_X_POS_MIN;
		if (r_space.rocket_goal_x > GAME_X_POS_MAX)  r_space.rocket_goal_x = GAME_X_POS_MAX;
		if (r_space.rocket_goal_y < GAME_Y_POS_MIN)  r_space.rocket_goal_y = GAME_Y_POS_MIN;
		if (r_space.rocket_goal_y > GAME_Y_POS_MAX)  r_space.rocket_goal_y = GAME_Y_POS_MAX;
		if (r_space.rocket_goal_z < GAME_Z_POS_MIN)  r_space.rocket_goal_z = GAME_Z_POS_MIN;
		if (r_space.rocket_goal_z > GAME_Z_POS_MAX)  r_space.rocket_goal_z = GAME_Z_POS_MAX;
	}
 }

/*
 * square_root_binary_search : approximation for square root function
 *
 */

// Newton-Raphson method
int32_t sqrt_with_accuracy(int32_t x, int32_t init_guess)
{	int32_t next_guess;

	// if we reach 0=init_guess, then the number and answer is zero
	if (0 == init_guess) return 0;

	next_guess = (init_guess + (x/init_guess))/2;
	if (abs(init_guess - next_guess) < 1)
		return(next_guess);
	else
		return(sqrt_with_accuracy(x, next_guess));
};

/*
 * compute_rocket_cable_lengths : compute the rocket position to cable lengths
 *
 */

static void do_compute_cable_length(int32_t tower) {
	int32_t x,y,z,length;

	// we will use millimeters for the intermedate calculation to avoid overflow
	x=(r_space.rocket_goal_x-r_towers[tower].pos_x)/1000;
	y=(r_space.rocket_goal_y-r_towers[tower].pos_y)/1000;
	z=(r_space.rocket_goal_z-r_towers[tower].pos_z)/1000;
	r_towers[tower].length_goal = sqrt_with_accuracy((x*x)+(y*y)+(z*z),500)*1000;

	// Calculate needed cable deployment change
	length = r_towers[tower].length_goal - r_towers[tower].length;
	// steps = l / (circ / 200) = (length * 200)/circ
	r_towers[tower].move_steps = (length * ROCKET_TOWER_SPOOL_STEPS) / r_towers[ROCKET_TOWER_NW].spool_circ;

	// Calculate needed cable deployment speed
	// This is the speed to do the steps within the loop time of 200 mSec
	// TODO ################
	r_towers[tower].speed = 1;
}

void compute_rocket_cable_lengths ()
 {
 	do_compute_cable_length(ROCKET_TOWER_NW);
 	do_compute_cable_length(ROCKET_TOWER_NE);
 	do_compute_cable_length(ROCKET_TOWER_SW);
 	do_compute_cable_length(ROCKET_TOWER_SE);
 }

/*
 * move_rocket_next_position : incrementally move the Rocket position
 *
 */

void do_move_rocket_next_position (bool simulate)
 {
	if (IO_MOTOR_ENABLE && !simulate) {
		// TODO ################ Add motor control

	} else {
		r_space.rocket_x = r_space.rocket_goal_x;
		r_space.rocket_y = r_space.rocket_goal_y;
		r_space.rocket_z = r_space.rocket_goal_z;

		r_towers[ROCKET_TOWER_NW].length = r_towers[ROCKET_TOWER_NW].length_goal;
		r_towers[ROCKET_TOWER_NE].length = r_towers[ROCKET_TOWER_NE].length_goal;
		r_towers[ROCKET_TOWER_SW].length = r_towers[ROCKET_TOWER_SW].length_goal;
		r_towers[ROCKET_TOWER_SE].length = r_towers[ROCKET_TOWER_SE].length_goal;
	}
 }

void move_rocket_next_position () {
	do_move_rocket_next_position(false);
}
void simulate_move_rocket_next_position () {
	do_move_rocket_next_position(true);
}

/*
 * test_rocket_in_position : inform if rocket is in position
 *                           (e.g. at start position)
 *
 */

bool test_rocket_in_position ()
 {
	 // TODO ################ Refine

	if ((r_space.rocket_x == r_space.rocket_goal_x) &&
	    (r_space.rocket_y == r_space.rocket_goal_y) &&
	    (r_space.rocket_z == r_space.rocket_goal_z) ) {
		return true;
	} else {
		return false;
	}
 }

/*
 * rocket_increment_send : increment a rocket motor
 *
 */

void rocket_increment_send (int32_t increment_nw, int32_t increment_ne, int32_t increment_sw, int32_t increment_se)
 {
	uint8_t buf[10];

	buf[0]=(uint8_t) 'm';
	buf[1]=(uint8_t) ((increment_nw & 0x00ff00L) >> 8);
	buf[2]=(uint8_t) ((increment_nw & 0x0000ffL)     );
	buf[3]=(uint8_t) ((increment_ne & 0x00ff00L) >> 8);
	buf[4]=(uint8_t) ((increment_ne & 0x0000ffL)     );
	buf[5]=(uint8_t) ((increment_sw & 0x00ff00L) >> 8);
	buf[6]=(uint8_t) ((increment_sw & 0x0000ffL)     );
	buf[7]=(uint8_t) ((increment_se & 0x00ff00L) >> 8);
	buf[8]=(uint8_t) ((increment_se & 0x0000ffL)     );

	i2c_polling_write (i2c, buf, 9, ROCKET_MOTOR_I2C_ADDRESS);

 }
