/* rocket_measure.h - Rocket Lander Game */

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
#define COMPASS_LOCK        4   // lock/unlock the compass heading

struct CompassRec {
    int32_t calibration_lock_x;
    int32_t calibration_lock_y;

	int32_t nw_inc,ne_inc,sw_inc,se_inc;
	int32_t x,y,z;

    const char* name;
    boolean lock;
};

extern int sqrt_cnt;

int32_t sqrt_rocket(int32_t x);

int32_t micrometers2steps(int32_t tower,int32_t value);
int32_t steps2micrometers(int32_t tower,int32_t value);
int32_t n2m(int32_t value);

extern void compass_adjustment(uint8_t command, struct CompassRec *compass);
extern void compute_tower_step_to_nm();

