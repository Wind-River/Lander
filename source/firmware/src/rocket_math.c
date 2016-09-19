/* rocket_math.c - Rocket Lander Game */

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

 /*
 * Theory of Implementation
 *
 * This file contains the advanced math computation routines
 * It provised integer routines for game play
 * It provides flouting point routines for calibration and curved rocket motions
 *
 * This file features:
 *  - Interget version of the square root function (game-time distances)
 *  - Least-square approximation of motor steps to tower cable micro-meters
 *  - Curve routines for the come-hither and self-play modes
 *
 */

#include <zephyr.h>

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "rocket.h"
#include "rocket_state.h"
#include "rocket_space.h"
#include "rocket_math.h"

/*
 * square_root_binary_search : approximation for square root function
 *
 */

int sqrt_cnt=0;	// loop protection counter

// Newton-Raphson method
//   NOTE: allowed accuracy of "< 2" avoids infinite bounce between 1 and -1
static int32_t do_sqrt_rocket(int32_t x, int32_t init_guess)
{	int32_t next_guess;

	if (++sqrt_cnt > 10) {
		PRINT("#############SQRT_TOOMANY(%ld,%ld\n",x,init_guess);
		return init_guess;
	}

	// if we reach 0=init_guess, then the number and answer is zero
	if ((0 == x) || (0 == init_guess)) return 0;

	next_guess = (init_guess + (x/init_guess))/2;
	if (abs(init_guess - next_guess) < 2)
		return(next_guess);
	else
		return(do_sqrt_rocket(x, next_guess));
};

int32_t sqrt_rocket(int32_t x) {
	sqrt_cnt=0;

	// scale the initial guess to reduce iteration loops
	if      (    6824L > x) return(do_sqrt_rocket(x, 50));
	else if (  682400L > x) return(do_sqrt_rocket(x, 500));
	else                    return(do_sqrt_rocket(x, 5000));
}

/*
 * explicitly convert between micrometers and millimeters
 *
 */

int32_t micro2millimeter(int32_t value) {
	if (0 <= value)
		return((value+500L)/1000L);
	else
		return((value-500L)/1000L);
}

int32_t milli2micrometer(int32_t value) {
	return(value*1000L);
}

/*
 * tower spool length calibrate : extrapolate tower uMeters to steps from measurement table
 *
 */

struct TOWER_SPOOL_SAMPLES {
	uint32_t length;	// measured length (mm)
	uint32_t steps;		// measured steps
};

#define TOWER_SPOOL_SAMPLES_MAX 22

struct TOWER_SPOOL_SAMPLES spool_samples[4][TOWER_SPOOL_SAMPLES_MAX] = {
	// NW
	{
		{	100,1919},
		{	132,2119},
		{	163,2319},
		{	195,2519},
		{	227,2719},
		{	259,2919},
		{	291,3119},
		{	323,3319},
		{	355,3519},
		{	385,3719},
		{	418,3919},
		{	448,4119},
		{	477,4319},
		{	508,4519},
		{	538,4719},
		{	569,4919},
		{	599,5119},
		{	628,5319},
		{	658,5519},
		{	687,5719},
		{	716,5919},
		{	745,9119}
	},
	// NE
	{
		{	100,1919},
		{	132,2119},
		{	163,2319},
		{	195,2519},
		{	227,2719},
		{	259,2919},
		{	291,3119},
		{	323,3319},
		{	355,3519},
		{	385,3719},
		{	418,3919},
		{	448,4119},
		{	477,4319},
		{	508,4519},
		{	538,4719},
		{	569,4919},
		{	599,5119},
		{	628,5319},
		{	658,5519},
		{	687,5719},
		{	716,5919},
		{	745,9119}
		},
	// SW
	{
		{	100,1919},
		{	132,2119},
		{	163,2319},
		{	195,2519},
		{	227,2719},
		{	259,2919},
		{	291,3119},
		{	323,3319},
		{	355,3519},
		{	385,3719},
		{	418,3919},
		{	448,4119},
		{	477,4319},
		{	508,4519},
		{	538,4719},
		{	569,4919},
		{	599,5119},
		{	628,5319},
		{	658,5519},
		{	687,5719},
		{	716,5919},
		{	745,9119}
		},
	// SE
	{
		{	100,1919},
		{	132,2119},
		{	163,2319},
		{	195,2519},
		{	227,2719},
		{	259,2919},
		{	291,3119},
		{	323,3319},
		{	355,3519},
		{	385,3719},
		{	418,3919},
		{	448,4119},
		{	477,4319},
		{	508,4519},
		{	538,4719},
		{	569,4919},
		{	599,5119},
		{	628,5319},
		{	658,5519},
		{	687,5719},
		{	716,5919},
		{	745,9119}
	}
};

int32_t micrometers2steps(int32_t tower,int32_t um) {
	int32_t millimeters,steps;
	uint8_t i;

	millimeters = micro2millimeter(um);
	if (millimeters < spool_samples[tower][0].length)
		return(spool_samples[tower][0].steps);
	if (millimeters > spool_samples[tower][TOWER_SPOOL_SAMPLES_MAX-1].length)
		return(spool_samples[tower][TOWER_SPOOL_SAMPLES_MAX-1].steps);

	for (i=0;i<TOWER_SPOOL_SAMPLES_MAX;) {
		if (spool_samples[tower][i+1].length > millimeters) {
			break;
		} else {
			i++;
		}
	}

	steps  = millimeters - spool_samples[tower][i].length;
	steps *= spool_samples[tower][i+1].steps  - spool_samples[tower][i].steps;
	steps /= spool_samples[tower][i+1].length - spool_samples[tower][i].length;
	steps += spool_samples[tower][i].steps;

	// HACK FOR NEW SPINDLES ######s
	steps /= 2;

	return (steps);
}

int32_t steps2micrometers(int32_t tower,int32_t steps) {
	int32_t millimeters;
	uint8_t i;

	if (steps < spool_samples[tower][0].steps)
		return(milli2micrometer(spool_samples[tower][0].length));
	if (steps > spool_samples[tower][TOWER_SPOOL_SAMPLES_MAX-1].steps)
		return(milli2micrometer(spool_samples[tower][TOWER_SPOOL_SAMPLES_MAX-1].length));

	for (i=0;i<TOWER_SPOOL_SAMPLES_MAX;) {
		if (spool_samples[tower][i+1].steps > steps) {
			break;
		} else {
			i++;
		}
	}

	millimeters  = steps - spool_samples[tower][i].steps;
	millimeters *= spool_samples[tower][i+1].length - spool_samples[tower][i].length;
	millimeters /= spool_samples[tower][i+1].steps  - spool_samples[tower][i].steps;
	millimeters += spool_samples[tower][i].length;

	return (milli2micrometer(millimeters));
}

/*
 * compass_select : tool to select the (initial) position of the rocket or menu selection
 *			calculate joystick position to compass direction/selection, slider to speed/scale
 *
 */

void compass_select(uint8_t command, struct CompassRec *compass) {
    int32_t x_delta,y_delta,z_delta;

    if (COMPASS_INIT == command) {
        compass->calibration_lock_x=0;
        compass->calibration_lock_y=0;
        compass->nw_inc=0;
        compass->ne_inc=0;
        compass->sw_inc=0;
        compass->se_inc=0;
        compass->x=ROCKET_HOME_X;
        compass->y=ROCKET_HOME_Y;
        compass->z=ROCKET_HOME_Z;
        compass->name = "";
        compass->lock = false;
        return;
    }

	// Convert joystick direction to compass selection
	x_delta = r_control.analog_x - JOYSTICK_X_MID;
	if (x_delta < -JOYSTICK_DELTA_XY_MIN) {
		x_delta = -1;
	} else if (x_delta > JOYSTICK_DELTA_XY_MIN ) {
		x_delta = 1;
	} else {
		x_delta = 0;
	}
	y_delta = r_control.analog_y - JOYSTICK_Y_MID;
	if (y_delta < -JOYSTICK_DELTA_XY_MIN) {
		y_delta = -1;
	} else if (y_delta > JOYSTICK_DELTA_XY_MIN ) {
		y_delta = 1;
	} else {
		y_delta = 0;
	}

    if (COMPASS_LOCK == command) {
        // lock the current compass (unlock if currently center)
    	if (!x_delta && !y_delta) {
    		compass->calibration_lock_x = 0;
    		compass->calibration_lock_y = 0;
	        compass->lock = false;
    	} else {
    		compass->calibration_lock_x = x_delta;
    		compass->calibration_lock_y = y_delta;
	        compass->lock = true;
    	}
    	return;
    } else {
        // assert lock
        if (compass->lock) {
    		x_delta = compass->calibration_lock_x;
    		y_delta = compass->calibration_lock_y;
        }
    }

	// Thruster Z is 'proportion-up or 'proportion-down' or 'off'
	z_delta = r_control.analog_z - JOYSTICK_Z_MID;
	if (z_delta < -JOYSTICK_DELTA_Z_MIN) {
		z_delta = (z_delta+JOYSTICK_DELTA_Z_MIN)*ROCKET_CALIBRATE_INC_Z;
	} else if (z_delta > JOYSTICK_DELTA_Z_MIN) {
		z_delta = (z_delta-JOYSTICK_DELTA_Z_MIN)*ROCKET_CALIBRATE_INC_Z;
	} else {
		z_delta = 0;
	}

	// scale Z from nm to steps
	if (COMPASS_CALC_HOME == command) {
    	z_delta = (z_delta * 10L) / ROCKET_TOWER_STEP_PER_UM10;
    } else if (COMPASS_CALC_TILT == command) {
        // TODO ############## scale to fine tilt control
    	z_delta = (z_delta * 10L) / ROCKET_TOWER_STEP_PER_UM10;
    } else if (COMPASS_CALC_POS == command) {
    	// ignore Z for now, use later for position scaling
    } else if (COMPASS_CALC_CIRC == command) {
    	// ignore Z for now, use later for circle scaling
    } else {
        // unknown command
        return;
    }

	// compute compass increment(s)
	if (COMPASS_CALC_POS ==  command) {
		if        ((x_delta < 0) && (y_delta > 0)) {
			compass->name = "NW+10";
			compass->x=ROCKET_HOME_X-100000L;
			compass->y=ROCKET_HOME_Y+100000L;
			compass->z=ROCKET_HOME_Z+100000L;
		} else if ((x_delta == 0) && (y_delta > 0)) {
			compass->name = "HM+05";
			compass->x=ROCKET_HOME_X+     0L;
			compass->y=ROCKET_HOME_Y+     0L;
			compass->z=ROCKET_HOME_Z+ 50000L;
		} else if ((x_delta > 0) && (y_delta > 0)) {
			compass->name = "NE+10";
			compass->x=ROCKET_HOME_X+100000L;
			compass->y=ROCKET_HOME_Y+100000L;
			compass->z=ROCKET_HOME_Z+100000L;
		} else if ((x_delta > 0) && (y_delta == 0)) {
			compass->name = "HM+10";
			compass->x=ROCKET_HOME_X+     0L;
			compass->y=ROCKET_HOME_Y+     0L;
			compass->z=ROCKET_HOME_Z+100000L;
		} else if ((x_delta > 0) && (y_delta < 0)) {
			compass->name = "SE+10";
			compass->x=ROCKET_HOME_X+100000L;
			compass->y=ROCKET_HOME_Y-100000L;
			compass->z=ROCKET_HOME_Z+100000L;
		} else if ((x_delta == 0) && (y_delta < 0)) {
			compass->name = "HM+15";
			compass->x=ROCKET_HOME_X+     0L;
			compass->y=ROCKET_HOME_Y+     0L;
			compass->z=ROCKET_HOME_Z+150000L;
		} else if ((x_delta < 0) && (y_delta < 0)) {
			compass->name = "SW+10";
			compass->x=ROCKET_HOME_X-100000L;
			compass->y=ROCKET_HOME_Y-100000L;
			compass->z=ROCKET_HOME_Z+100000L;
		} else if ((x_delta < 0) && (y_delta == 0)) {
			compass->name = "HM+20";
			compass->x=ROCKET_HOME_X+     0L;
			compass->y=ROCKET_HOME_Y+     0L;
			compass->z=ROCKET_HOME_Z+200000L;
		} else if ((x_delta == 0) && (y_delta == 0)) {
			// if center, then true home
			compass->name = "HM+00";
			compass->x=ROCKET_HOME_X+0L;
			compass->y=ROCKET_HOME_Y+0L;
			compass->z=ROCKET_HOME_Z+0L;
		}
	} else if (COMPASS_CALC_CIRC ==  command) {
		if        ((x_delta == 0) && (y_delta == 0)) {
			compass->name = "Home  ";
		} else if ((x_delta < 0) && (y_delta > 0)) {
			compass->name = "AllHgh";
		} else if ((x_delta == 0) && (y_delta > 0)) {
			compass->name = "Z_Circ";
		} else if ((x_delta > 0) && (y_delta > 0)) {
			compass->name = "Bumble";
		} else if ((x_delta > 0) && (y_delta == 0)) {
			compass->name = "Y_Circ";
		} else if ((x_delta > 0) && (y_delta < 0)) {
			compass->name = "      ";
		} else if ((x_delta == 0) && (y_delta < 0)) {
			compass->name = "X_Circ";
		} else if ((x_delta < 0) && (y_delta < 0)) {
			compass->name = "AllMed";
		} else if ((x_delta < 0) && (y_delta == 0)) {
			compass->name = "AllLow";
		}
	} else if (COMPASS_CALC_GROUND ==  command) {
		if        ((x_delta == 0) && (y_delta == 0)) {
			compass->name = "Home";
		} else if ((x_delta < 0) && (y_delta > 0)) {
			compass->name = "      ";
		} else if ((x_delta == 0) && (y_delta > 0)) {
			compass->name = "+05";
		} else if ((x_delta > 0) && (y_delta > 0)) {
			compass->name = "      ";
		} else if ((x_delta > 0) && (y_delta == 0)) {
			compass->name = "+10";
		} else if ((x_delta > 0) && (y_delta < 0)) {
			compass->name = "      ";
		} else if ((x_delta == 0) && (y_delta < 0)) {
			compass->name = "+20";
		} else if ((x_delta < 0) && (y_delta < 0)) {
			compass->name = "      ";
		} else if ((x_delta < 0) && (y_delta == 0)) {
			compass->name = "      ";
		}
	} else {
		if        ((x_delta < 0) && (y_delta > 0)) {
			compass->name = "NW";
			compass->nw_inc=z_delta;
		} else if ((x_delta == 0) && (y_delta > 0)) {
			compass->name = "N ";
			compass->nw_inc=z_delta;
			compass->ne_inc=z_delta;
		} else if ((x_delta > 0) && (y_delta > 0)) {
			compass->name = "NE";
			compass->ne_inc=z_delta;
		} else if ((x_delta > 0) && (y_delta == 0)) {
			compass->name = "E ";
			compass->ne_inc=z_delta;
			compass->se_inc=z_delta;
		} else if ((x_delta > 0) && (y_delta < 0)) {
			compass->name = "SE";
			compass->se_inc=z_delta;
		} else if ((x_delta == 0) && (y_delta < 0)) {
			compass->name = "S ";
			compass->se_inc=z_delta;
			compass->sw_inc=z_delta;
		} else if ((x_delta < 0) && (y_delta < 0)) {
			compass->name = "SW";
			compass->sw_inc=z_delta;
		} else if ((x_delta < 0) && (y_delta == 0)) {
			compass->name = "W ";
			compass->sw_inc=z_delta;
			compass->nw_inc=z_delta;
		} else if ((x_delta == 0) && (y_delta == 0)) {
			// if center, then affect all towers
			compass->nw_inc=z_delta;
			compass->ne_inc=z_delta;
			compass->sw_inc=z_delta;
			compass->se_inc=z_delta;
			compass->name = "AL";
		}
	}
}


/*
 * trigonomety tables, using linear interpolation
 *
 */

struct MATH_TABLE {
	double degrees;
	double value;
};

#define MATH_TABLE_MAX 37

struct MATH_TABLE cosine_table[MATH_TABLE_MAX] = {
  {  0,  1.000000 },
  { 10,  0.984808 },
  { 20,  0.939693 },
  { 30,  0.866025 },
  { 40,  0.766044 },
  { 50,  0.642788 },
  { 60,  0.500000 },
  { 70,  0.342020 },
  { 80,  0.173648 },
  { 90,  0.000000 },
  { 100, -0.173648 },
  { 110, -0.342020 },
  { 120, -0.500000 },
  { 130, -0.642788 },
  { 140, -0.766044 },
  { 150, -0.866025 },
  { 160, -0.939693 },
  { 170, -0.984808 },
  { 180, -1.000000 },
  { 190, -0.984808 },
  { 200, -0.939693 },
  { 210, -0.866025 },
  { 220, -0.766044 },
  { 230, -0.642788 },
  { 240, -0.500000 },
  { 250, -0.342020 },
  { 260, -0.173648 },
  { 270, -0.000000 },
  { 280,  0.173648 },
  { 290,  0.342020 },
  { 300,  0.500000 },
  { 310,  0.642788 },
  { 320,  0.766044 },
  { 330,  0.866025 },
  { 340,  0.939693 },
  { 350,  0.984808 },
  { 360,  1.000000 },
};

struct MATH_TABLE sine_table[MATH_TABLE_MAX] = {
  {  0,  0.000000 },
  { 10,  0.173648 },
  { 20,  0.342020 },
  { 30,  0.500000 },
  { 40,  0.642788 },
  { 50,  0.766044 },
  { 60,  0.866025 },
  { 70,  0.939693 },
  { 80,  0.984808 },
  { 90,  1.000000 },
  { 100,  0.984808 },
  { 110,  0.939693 },
  { 120,  0.866025 },
  { 130,  0.766044 },
  { 140,  0.642788 },
  { 150,  0.500000 },
  { 160,  0.342020 },
  { 170,  0.173648 },
  { 180,  0.000000 },
  { 190, -0.173648 },
  { 200, -0.342020 },
  { 210, -0.500000 },
  { 220, -0.642788 },
  { 230, -0.766044 },
  { 240, -0.866025 },
  { 250, -0.939693 },
  { 260, -0.984808 },
  { 270, -1.000000 },
  { 280, -0.984808 },
  { 290, -0.939693 },
  { 300, -0.866025 },
  { 310, -0.766044 },
  { 320, -0.642788 },
  { 330, -0.500000 },
  { 340, -0.342020 },
  { 350, -0.173648 },
  { 360, -0.000000 },
};

#define MATH_ATAN_MAX 44

struct MATH_TABLE tan_table[MATH_ATAN_MAX] = {
  { -86,-14.300666 },
  { -82, -7.115370 },
  { -78, -4.704630 },
  { -74, -3.487414 },
  { -70, -2.747477 },
  { -66, -2.246037 },
  { -62, -1.880726 },
  { -58, -1.600335 },
  { -54, -1.376382 },
  { -50, -1.191754 },
  { -46, -1.035530 },
  { -42, -0.900404 },
  { -38, -0.781286 },
  { -34, -0.674509 },
  { -30, -0.577350 },
  { -26, -0.487733 },
  { -22, -0.404026 },
  { -18, -0.324920 },
  { -14, -0.249328 },
  { -10, -0.176327 },
  { -6, -0.105104 },
  { -2, -0.034921 },
  {  2,  0.034921 },
  {  6,  0.105104 },
  { 10,  0.176327 },
  { 14,  0.249328 },
  { 18,  0.324920 },
  { 22,  0.404026 },
  { 26,  0.487733 },
  { 30,  0.577350 },
  { 34,  0.674509 },
  { 38,  0.781286 },
  { 42,  0.900404 },
  { 46,  1.035530 },
  { 50,  1.191754 },
  { 54,  1.376382 },
  { 58,  1.600335 },
  { 62,  1.880726 },
  { 66,  2.246037 },
  { 70,  2.747477 },
  { 74,  3.487414 },
  { 78,  4.704630 },
  { 82,  7.115370 },
  { 86, 14.300666 },
};

double degrees2cosine(int16_t degrees) {
	for (int i=0;i<MATH_TABLE_MAX;i++) {
		if (degrees < cosine_table[i+1].degrees) {
			double degree_part = (degrees - cosine_table[i].degrees)/(cosine_table[i+1].degrees-cosine_table[i].degrees);
			return ((((cosine_table[i+1].value-cosine_table[i].value)) * degree_part) + cosine_table[i].value);
		}
	}
	return(1.0);
};

double degrees2sine(int16_t degrees) {
	for (int i=0;i<MATH_TABLE_MAX;i++) {
		if (degrees < sine_table[i+1].degrees) {
			double degree_part = (degrees - sine_table[i].degrees)/(sine_table[i+1].degrees-sine_table[i].degrees);
			return ((((sine_table[i+1].value-sine_table[i].value)) * degree_part) + sine_table[i].value);
		}
	}
	return(0.0);
};

int16_t atan2degrees(double x, double y) {
	double value;

	// protect against division by zero
	if (y < 0.0001) y = 0.0001;
	value = x/y;

	if (value < tan_table[0].value) return(-89);

	for (int i=0;i<MATH_ATAN_MAX;i++) {
		if (value < tan_table[i+1].value) {
			double value_scale = (value - tan_table[i].value)/(tan_table[i+1].value-tan_table[i].value);
			double degree_part = (double) (tan_table[i+1].degrees-tan_table[i].degrees);
			return ((int16_t) ((degree_part * value_scale) + tan_table[i].degrees+ 0.5));
		}
	}

	return (89);
};


/*
 * flight_length
 *
 */

#define LENGTH_SQRT_SCALER   6	/* 2^6 = 64, Scale at 100 uM closely matches stepper 125nM step size */

static int32_t get_length(int32_t pos_x, int32_t pos_y, int32_t pos_z, int32_t goal_x, int32_t goal_y, int32_t goal_z) {
	int32_t x,y,z;

	// we will use uMeter scaler for the intermedate calculation to avoid overflow
	// i.e. all numbers must be <= 65535 so that the square does not overflow
	x=(goal_x - pos_x) >> LENGTH_SQRT_SCALER;
	y=(goal_y - pos_y) >> LENGTH_SQRT_SCALER;
	z=(goal_z - pos_z) >> LENGTH_SQRT_SCALER;
	return(sqrt_rocket((x*x)+(y*y)+(z*z)) << LENGTH_SQRT_SCALER);
}

/*
 * flight_linear
 *
 */

struct ROCKET_FLIGHT_S r_flight;

void flight_init () {
	r_flight.dx=0; r_flight.dy=0; r_flight.dz=0;

	r_flight.ax=0; r_flight.ay=0; r_flight.az=0;
	r_flight.current_ax=0; r_flight.current_ay=0; r_flight.current_az=0;
	r_flight.center_x=0;
	r_flight.center_y=0;
	r_flight.center_z=0;
	r_flight.radius=0;

	r_flight.speed=DEFAULT_SPEED;
	r_flight.current_x=0; r_flight.current_y=0; r_flight.current_z=0;
	r_flight.final_x  =0; r_flight.final_y  =0; r_flight.final_z  =0;
	r_flight.frame_count=0;
	r_flight.frame_max  =0;
	r_flight.state_done=STATE_NOP;
 }

/* speed is millimeters per second
 * Given that the tower cable speed max is 4 rps * 200 steps/rpm * 125.6 uM/step = 100 mm/sec,
 * let us suggest a safe and sane speed of 80 mm/sec, which is 16 mm per frame
 */

//#define DEFAULT_LINEAR_MM_PER_SECOND 50
#define DEFAULT_LINEAR_MM_PER_SECOND 80

void flight_linear (int32_t dest_x,int32_t dest_y,int32_t dest_z, int32_t speed) {
	int32_t length;

	flight_init();

	r_flight.final_x = dest_x;
	r_flight.final_y = dest_y;
	r_flight.final_z = dest_z;

	r_flight.current_x=r_space.rocket_x;
	r_flight.current_y=r_space.rocket_y;
	r_flight.current_z=r_space.rocket_z;

	length = get_length(
		r_space.rocket_x,
		r_space.rocket_y,
		r_space.rocket_z,
		dest_x,
		dest_y,
		dest_z);

	if (MOTOR_SPEED_AUTO == speed) {
		speed = DEFAULT_LINEAR_MM_PER_SECOND;
	}
	if (0 == speed) {
		r_flight.frame_max = 0;
		return;
	}
	r_flight.frame_max = (length/1000L)/(speed/FRAMES_PER_SECOND);
	if (0 == r_flight.frame_max) {
		return;
	} else if (200 < r_flight.frame_max) {
		r_flight.frame_max = 200;
	}
	r_flight.dx=(dest_x-r_flight.current_x)/r_flight.frame_max;
	r_flight.dy=(dest_y-r_flight.current_y)/r_flight.frame_max;
	r_flight.dz=(dest_z-r_flight.current_z)/r_flight.frame_max;
}

void flight_linear_loop () {
	r_flight.frame_count++;

	if (r_flight.frame_count < r_flight.frame_max) {
		r_flight.current_x += r_flight.dx;
		r_flight.current_y += r_flight.dy;
		r_flight.current_z += r_flight.dz;
	} else {
		// if last step, jump to the end (and resolve roundoff errors)
		r_flight.current_x = r_flight.final_x;
		r_flight.current_y = r_flight.final_y;
		r_flight.current_z = r_flight.final_z;
	}

	r_space.rocket_goal_x = r_flight.current_x;
	r_space.rocket_goal_y = r_flight.current_y;
	r_space.rocket_goal_z = r_flight.current_z;
	compute_rocket_cable_lengths();
	move_rocket_next_position();
}

void flight_wait (int32_t frame_count) {
	flight_init();

	r_flight.final_x = r_space.rocket_x;
	r_flight.final_y = r_space.rocket_y;
	r_flight.final_z = r_space.rocket_z;

	r_flight.current_x=r_flight.final_x;
	r_flight.current_y=r_flight.final_y;
	r_flight.current_z=r_flight.final_z;

	r_flight.frame_max = frame_count;
}

void flight_wait_loop () {
	r_flight.frame_count++;
}

/*
 * rigid rotation math
 *
 */

void rigid_rotation_compute (int16_t x_degrees,int16_t y_degrees,int16_t z_degrees,int32_t start_x,int32_t start_y,int32_t start_z) {
	double rotation_matrix[3][3];

	// convert initial position to double in 1/10 mm
	double x = (double) ((start_x - r_flight.center_x)/100L);
	double y = (double) ((start_y - r_flight.center_y)/100L);
	double z = (double) ((start_z - r_flight.center_z)/100L);

	// tait_brian rigid rotation matrix computation, rotating z then y then x
	double s0 = degrees2sine(z_degrees), c0 = degrees2cosine(z_degrees);
	double s1 = degrees2sine(y_degrees), c1 = degrees2cosine(y_degrees);
	double s2 = degrees2sine(x_degrees), c2 = degrees2cosine(x_degrees);
	rotation_matrix[0][0] =  c0 * c1;
	rotation_matrix[0][1] =  s0 * c1;
	rotation_matrix[0][2] = -s1;
	rotation_matrix[1][0] = -s0 * c2 + c0 * s1 * s2;
	rotation_matrix[1][1] =  c0 * c2 + s0 * s1 * s2;
	rotation_matrix[1][2] =  c1 * s2;
	rotation_matrix[2][0] =  s0 * s2 + c0 * s1 * c2;
	rotation_matrix[2][1] = -c0 * s2 + s0 * s1 * c2;
	rotation_matrix[2][2] =  c1 * c2;

	// compute next location
	x = (rotation_matrix[0][0] * x) + (rotation_matrix[0][1] * y) + (rotation_matrix[0][2] * z);
	y = (rotation_matrix[1][0] * x) + (rotation_matrix[1][1] * y) + (rotation_matrix[1][2] * z);
	z = (rotation_matrix[2][0] * x) + (rotation_matrix[2][1] * y) + (rotation_matrix[2][2] * z);

	// convert to integer um coordinates around center
	r_flight.current_x = (((int32_t) x + 0.5) * 100L) + r_flight.center_x;
	r_flight.current_y = (((int32_t) y + 0.5) * 100L) + r_flight.center_y;
	r_flight.current_z = (((int32_t) z + 0.5) * 100L) + r_flight.center_z;
}

/*
 * flight_circular
 *
 */

void flight_circular (int32_t ax,int32_t ay,int32_t az, int32_t center_x, int32_t center_y, int32_t center_z, int32_t frame_count) {
	flight_init();

	r_flight.ax=ax;
	r_flight.ay=ay;
	r_flight.az=az;

	r_flight.center_x=center_x;
	r_flight.center_y=center_y;
	r_flight.center_z=center_z;

	r_flight.current_x = r_space.rocket_x;
	r_flight.current_y = r_space.rocket_y;
	r_flight.current_z = r_space.rocket_z;

	r_flight.radius=get_length(
		r_space.rocket_x,
		r_space.rocket_y,
		r_space.rocket_z,
		center_x,
		center_y,
		center_z);

	/* compute current degrees */
	// ### TODO : SHORT CUT FOR NOW
	if        ((10000L == r_space.rocket_x) && (0L == r_space.rocket_y) && (20000L==r_space.rocket_z)) {
		r_flight.current_ax=0; r_flight.current_ay=0; r_flight.current_az=0;
	} else if ((0L == r_space.rocket_x) && (10000L == r_space.rocket_y) && (20000L==r_space.rocket_z)) {
		r_flight.current_ax=0; r_flight.current_ay=0; r_flight.current_az=90;
	} else {
		r_flight.current_ax=0; r_flight.current_ay=0; r_flight.current_az=0;
	}

	r_flight.frame_max = frame_count;
}

void flight_circular_loop () {
	float x=1.0,y=1.0,z=1.0;
	r_flight.frame_count++;

	/* increment angles */
	r_flight.current_ax = (r_flight.current_ax + r_flight.ax) % 360;
	r_flight.current_ay = (r_flight.current_ay + r_flight.ay) % 360;
	r_flight.current_az = (r_flight.current_az + r_flight.az) % 360;

	/* find next rotation destination points */
	if        ((0 == r_flight.current_ay) && (0 == r_flight.current_ax)) {
		// simple rotate on z-axis
		x = degrees2cosine(r_flight.current_az);
		y = degrees2sine(  r_flight.current_az);
		r_flight.current_x = ((int32_t) (x * r_flight.radius)) + r_flight.center_x;
		r_flight.current_y = ((int32_t) (y * r_flight.radius)) + r_flight.center_y;
		// Z is unchanged
	} else if ((0 == r_flight.current_az) && (0 == r_flight.current_ax)) {
		// simple rotate on y-axis
		x = degrees2cosine(r_flight.current_ay);
		z = degrees2sine(  r_flight.current_ay);
		r_flight.current_x = ((int32_t) (x * r_flight.radius)) + r_flight.center_x;
		r_flight.current_z = ((int32_t) (z * r_flight.radius)) + r_flight.center_z;
		// Y is unchanged
	} else if ((0 == r_flight.current_az) && (0 == r_flight.current_ay)) {
		// simple rotate on x-axis
		y = degrees2cosine(r_flight.current_ax);
		z = degrees2sine(  r_flight.current_ax);
		r_flight.current_y = ((int32_t) (y * r_flight.radius)) + r_flight.center_y;
		r_flight.current_z = ((int32_t) (z * r_flight.radius)) + r_flight.center_z;
		// X is unchanged
	} else {
		rigid_rotation_compute (r_flight.current_ax,r_flight.current_ay,r_flight.current_az,ROCKET_HOME_X+100000L,ROCKET_HOME_Y+0,ROCKET_HOME_Z+150000L);
	}

	// send the coordinates to the rocket
	r_space.rocket_goal_x = r_flight.current_x;
	r_space.rocket_goal_y = r_flight.current_y;
	r_space.rocket_goal_z = r_flight.current_z;
	if (!self_test) {
		compute_rocket_cable_lengths();
		move_rocket_next_position();
	}
}

/*
 * Antenna control
 *
 */

struct PWM_DEGREES_TABLE {
	int16_t degrees;
	uint16_t value;
};

#define MATH_PAN_MAX 2

struct PWM_DEGREES_TABLE pan_table[MATH_PAN_MAX] = {
  { -90, (518/4)},
  {  90, (165/4)},
};

#define MATH_TILT_MAX 3

struct PWM_DEGREES_TABLE tilt_table[MATH_TILT_MAX] = {
  { -45, (497/4)},
  {   0, (401/4)},
  {  90, (215/4)},
};


uint16_t pan_degrees2pwm(int16_t degrees) {
	if (degrees < pan_table[0].degrees)
		return pan_table[0].value;
	for (int i=0;i<MATH_PAN_MAX;i++) {
		if (degrees < pan_table[i+1].degrees) {
			int16_t result;
			result  = (degrees - pan_table[i].degrees);
			result *= (pan_table[i+1].value-pan_table[i].value);
			result /= (pan_table[i+1].degrees-pan_table[i].degrees);
			return (result + pan_table[i].value);
		}
	}
	return pan_table[MATH_PAN_MAX-1].value;
};

uint16_t tilt_degrees2pwm(int16_t degrees) {
	if (degrees < tilt_table[0].degrees)
		return tilt_table[0].value;
	for (int i=0;i<MATH_PAN_MAX;i++) {
		if (degrees < tilt_table[i+1].degrees) {
			int16_t result;
			result  = (degrees - tilt_table[i].degrees);
			result *= (tilt_table[i+1].value-tilt_table[i].value);
			result /= (tilt_table[i+1].degrees-tilt_table[i].degrees);
			return (result + tilt_table[i].value);
		}
	}
	return tilt_table[MATH_PAN_MAX-1].value;
};


void antenna_update() {
	static uint16_t pan_current=0;
	static uint16_t tilt_current=0;
	uint16_t pan_now=0;
	uint16_t tilt_now=0;
	double degrees_x,degrees_z;

	degrees_x = atan2degrees((double) (r_space.rocket_goal_x - ANTENNA_X_POS), (double) (r_space.rocket_goal_y - ANTENNA_Y_POS));
	degrees_z = atan2degrees((double) (r_space.rocket_goal_z - ANTENNA_Z_POS), (double) (r_space.rocket_goal_y - ANTENNA_Y_POS));

	pan_now=pan_degrees2pwm(degrees_x);
	tilt_now=tilt_degrees2pwm(degrees_z);

	if (false) printf("Antennae(%ld,%ld,%ld)=(%f,%f)=(%d,%d)\n",
		r_space.rocket_goal_x,r_space.rocket_goal_y,r_space.rocket_goal_z,
		degrees_x,degrees_z,
		pan_now,tilt_now
		);

	if (!self_test && IO_TRACKER_FOLLOW_ENABLE && ((pan_current != pan_now) || (tilt_current != tilt_now))) {
		pan_current=pan_now;
		tilt_current=tilt_now;
		send_Pan_Tilt(pan_now,tilt_now);
	}
}
