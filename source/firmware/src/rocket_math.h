/* rocket_math.h - Rocket Lander Game */

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

#define COMPASS_INIT        0   // initialize the compass record
#define COMPASS_CALC_HOME   1   // calculate step offests for homing
#define COMPASS_CALC_TILT   2   // calculate step offsets for tilt asjust 
#define COMPASS_CALC_POS    3   // calculate compass for position moves
#define COMPASS_CALC_CIRC   4   // calculate compass for circle moves
#define COMPASS_LOCK        5   // lock/unlock the compass heading

struct CompassRec {
    int32_t calibration_lock_x;
    int32_t calibration_lock_y;

	int32_t nw_inc,ne_inc,sw_inc,se_inc;
	int32_t x,y,z;

    const char* name;
    boolean lock;
};

struct ROCKET_FLIGHT_S {
	int32_t	dx;			// change in X,Y,Z per frame in uM
	int32_t	dy;
	int32_t	dz;

	int32_t	ax;		// change in angle around X,Y,Z per frame, degrees
	int32_t	ay;
	int32_t	az;

	int32_t	speed;		// speed (microseconds per step)

	int32_t	current_ax;	// current angle on axis X,Y,Z in degrees
	int32_t	current_ay;
	int32_t	current_az;

	int32_t	center_x;	// center X,Y,Z for circles in uM
	int32_t	center_y;
	int32_t	center_z;
	int32_t	radius;

	int32_t	current_x;	// current X,Y,Z in uM
	int32_t	current_y;
	int32_t	current_z;

	int32_t	final_x;	// final X,Y,Z in uM
	int32_t	final_y;
	int32_t	final_z;

	int32_t	frame_count;
	int32_t	frame_max;

	const char* state_done;	// state to jump when flight  done
};

extern int sqrt_cnt;

int32_t sqrt_rocket(int32_t x);

int32_t micrometers2steps(int32_t tower,int32_t value);
int32_t steps2micrometers(int32_t tower,int32_t value);
int32_t micro2millimeter(int32_t value);
int32_t milli2micrometer(int32_t value);

extern struct ROCKET_FLIGHT_S r_flight;

void compass_select(uint8_t command, struct CompassRec *compass);
void compute_tower_step_to_nm();

double degrees2sine(int16_t degrees);
double degrees2cosine(int16_t degrees);
int16_t atan2degrees(double x, double y);

void flight_linear(int32_t dest_x,int32_t dest_y,int32_t dest_z, int32_t speed);
void flight_linear_loop();

void flight_circular(int32_t ax,int32_t ay,int32_t az, int32_t center_x, int32_t center_y, int32_t center_z, int32_t frame_count);
void flight_circular_loop();

void flight_wait(int32_t frame_count);
void flight_wait_loop();
