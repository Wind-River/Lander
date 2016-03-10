/* rocket_measure.c - Rocket Lander Game */

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
#include "rocket_space.h"
#include "rocket_math.h" 

/*
 * square_root_binary_search : approximation for square root function
 *
 */

int sqrt_cnt=0;	// loop protection counter

// Newton-Raphson method
//   NOTE: allowed accuracy of (1..-1) avoids infinite bounce between 1 and -1
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

	if      (    6824L > x) return(do_sqrt_rocket(x, 50));
	else if (  682400L > x) return(do_sqrt_rocket(x, 500));
	else                    return(do_sqrt_rocket(x, 5000));
}

/*
 * convert micrometers to and from steps, using respective tower's linear equation
 *
 */

int32_t micrometers2steps(int32_t tower,int32_t value) {
	value <<= r_towers[tower].um2step_scaler;
	value  /= r_towers[tower].um2step_slope;
	value  += r_towers[tower].um2step_offset;
	return (value);
}

int32_t steps2micrometers(int32_t tower,int32_t value) {
	value  -= r_towers[tower].um2step_offset;
	value  *= r_towers[tower].um2step_slope;
	value >>= r_towers[tower].um2step_scaler;
	return (value);
}

int32_t n2m(int32_t value) {
	if (0 <= value)
		return((value+500L)/1000L);
	else
		return((value-500L)/1000L);
}

/*
 * compass_adjustment : tool to adjust the (initial) position of the rocket
 *			calculate joystick position to compass direction, slider to speed
 *
 */

void compass_adjustment(uint8_t command, struct CompassRec *compass) {
    int32_t x_delta,y_delta,z_delta;

    if (COMPASS_INIT == command) {
        compass->calibration_lock_x=0;
        compass->calibration_lock_y=0;
        compass->nw_inc=0;
        compass->ne_inc=0;
        compass->sw_inc=0;
        compass->se_inc=0;
        compass->x=0;
        compass->y=0;
        compass->z=0;
        compass->name = "";
        compass->lock = false;
        return;
    }
    
	// Convert joystick to compass selection
	x_delta = r_control.analog_x - JOYSTICK_X_MID;
	if (x_delta < -JOYSTICK_DELTA_XY_MIN) {
		x_delta = -1;
	} else if (x_delta > JOYSTICK_DELTA_XY_MIN ) {
		x_delta = 1;
	} else {
		x_delta = 0;
	}

	// Thruster Y is 'on-forward or 'on-backward' or 'off'
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
	}
	if (z_delta > JOYSTICK_DELTA_Z_MIN) {
		z_delta = (z_delta-JOYSTICK_DELTA_Z_MIN)*ROCKET_CALIBRATE_INC_Z;
	}

	// scale Z from nm to steps
	if (COMPASS_CALC_HOME == command) {
    	z_delta = micrometers2steps(ROCKET_TOWER_NW,z_delta);
    } else if (COMPASS_CALC_TILT == command) {
        // TODO ############## scale to fine tilt control
    	z_delta = micrometers2steps(ROCKET_TOWER_NW,z_delta);
    } else if (COMPASS_CALC_POS == command) {
    	// ignore Z for now, use later for position scaling
    } else {
        // unknown command
        return;
    }

	// compute compass increment(s)
	if (COMPASS_CALC_POS ==  command) {
		if        ((x_delta < 0) && (y_delta > 0)) {
			compass->name = "NW+10";
			compass->x=-100000L;
			compass->y= 100000L;
			compass->z= 100000L;
		} else if ((x_delta == 0) && (y_delta > 0)) {
			compass->name = "HM+05";
			compass->x=      0L;
			compass->y=      0L;
			compass->z=  50000L;
		} else if ((x_delta > 0) && (y_delta > 0)) {
			compass->name = "NE+10";
			compass->x= 100000L;
			compass->y= 100000L;
			compass->z= 100000L;
		} else if ((x_delta > 0) && (y_delta == 0)) {
			compass->name = "HM+10";
			compass->x=      0L;
			compass->y=      0L;
			compass->z= 100000L;
		} else if ((x_delta > 0) && (y_delta < 0)) {
			compass->name = "SE+10";
			compass->x=-100000L;
			compass->y=-100000L;
			compass->z= 100000L;
		} else if ((x_delta == 0) && (y_delta < 0)) {
			compass->name = "HM+15";
			compass->x=      0L;
			compass->y=      0L;
			compass->z= 150000L;
		} else if ((x_delta < 0) && (y_delta < 0)) {
			compass->name = "SW+10";
			compass->x=-100000L;
			compass->y=-100000L;
			compass->z= 100000L;
		} else if ((x_delta < 0) && (y_delta == 0)) {
			compass->name = "HM+20";
			compass->x=      0L;
			compass->y=      0L;
			compass->z= 200000L;
		} else if ((x_delta == 0) && (y_delta == 0)) {
			// if center, then true home
			compass->name = "HM+00";
			compass->x= 0L;
			compass->y= 0L;
			compass->z= 0L;
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

struct TOWER_SPOOL_SAMPLES {
	double steps;		// measured steps
	double length;		// measured length (um)
};


/*
 * least squares approximation 
 *   y = a + bx
 *   Sx, Sy, Sxx, Sxy, Syy
 *   b = ((n * Sxy) - (Sx * Sy))/((n * Sxx) - (Sx * Sx)
 *   a = ((1/n)*Sy) - (b*((1/n)*Sx))
 */

#define SAMPLE_SCALE 100

static void samples_least_squares(struct TOWER_SPOOL_SAMPLES samples[], uint8_t count,struct ROCKET_TOWER_S *tower) {
	double Sx=0;
	double Sy=0;
	double Sxx=0;
	double Sxy=0;
	double Syy=0;
	double b=0;
	double a=0;  
	double x,y;  
	uint8_t i;
	
	for (i=0;i<count;i++) {
		x=samples[i].steps;
		y=samples[i].length;
		
		Sx += x;
		Sy += y;
		Sxx += x*x;
		Sxy += x*y;
		Syy += y*y;
	}
	
	b = ((count * Sxy) - (Sx * Sy)) / ((count * Sxx) - (Sx * Sx));
	a = (Sy - (b*Sx)) / count;
	
	tower->um2step_slope  = (int32_t) (b * 8.0); /* 8 = 2^3 = um2step_scaler */
	tower->um2step_offset = (int32_t) (a * 1.0);
}


/*
 * tower spool calibrate : compute tower uMeters per step from measurement table 
 *			using least squares approximation
 *
 */

/* default length is to center = srqt(240000^2 + 170000^2 + 580000^2) = 650308
 * default steps is  650308 / 125.6 = 518
 */

struct TOWER_SPOOL_SAMPLES samples_test[] = {
    {	    1.47,    52.21},
    {	    1.50,    53.12},
    {	    1.52,    54.48},
    {	    1.55,    55.84},
    {	    1.57,    57.20},
    {	    1.60,    58.57},
    {	    1.63,    59.93},
    {	    1.65,    61.29},
    {	    1.68,    63.11},
    {	    1.70,    64.47},
    {	    1.73,    66.28},
    {	    1.75,    68.10},
    {	    1.78,    69.92},
    {	    1.80,    72.19},
    {	    1.83,    74.46},    
} ;

struct TOWER_SPOOL_SAMPLES samples_nw[] = {
    {	    0L,      0L},
    {    5178L, 650308L},
} ;

struct TOWER_SPOOL_SAMPLES samples_ne[] = {
    {	    0L,    0L},
    {    5178L, 650308L},
} ;

struct TOWER_SPOOL_SAMPLES samples_sw[] = {
    {	    0L,    0L},
    {    5178L, 650308L},
} ;

struct TOWER_SPOOL_SAMPLES samples_se[] = {
    {	    0L,    0L},
    {    5178L, 650308L},
} ;

void compute_tower_step_to_nm() {
	samples_least_squares(samples_test, 15, &r_towers[ROCKET_TOWER_NW]); /* unit test: b=61.27, a=-39.06 */
	samples_least_squares(samples_nw, 2, &r_towers[ROCKET_TOWER_NW]);
	samples_least_squares(samples_nw, 2, &r_towers[ROCKET_TOWER_NE]);
	samples_least_squares(samples_nw, 2, &r_towers[ROCKET_TOWER_SW]);
	samples_least_squares(samples_nw, 2, &r_towers[ROCKET_TOWER_SE]);
}
