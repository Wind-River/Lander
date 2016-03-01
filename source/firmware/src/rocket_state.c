/* rocket_state.c - Rocket Lander Game */

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
#include "groveLCDUtils.h"
#include "Adafruit_LEDBackpack.h"
#include "groveLCD.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "rocket.h"
#include "rocket_space.h"
#include "rocket_state.h"


/*
 * State Management
 *
 */

static char *ACTION_NOP=NULL;
static char *STATE_NOP=NULL;
static uint32_t StateGuiCount=0;
static char buffer[1000];

#define StateGuiMax 100
struct StateGuiRec state_array[StateGuiMax];
int32_t state_now;	// current state
static int32_t  state_prev;	// Previous state name (for pause/resume)


void state_callback(char *call_name);
static void display_state();
static int32_t find_state(char *select_state);

// Constructor
static void StateGuiAdd(char *state_name,uint32_t flags,char *display_1,char *display_2,char *k1,char *k2,char *state_enter,char *state_loop,char *state_exit) {
	if (StateGuiCount >= StateGuiMax) {
		log("ERROR: OUT OF STATE RECORD SPACE");
		return;
	}

	if (STATE_NOT_FOUND != find_state(state_name)) {
		log("\n");
		log_val("ERROR: Duplicate state %s\n",state_name);
		log("\n");
		return;
	}


	state_array[StateGuiCount].state_name = state_name;		// String name of state
	state_array[StateGuiCount].state_flags = flags;			// Optional state flags
	strcpy(state_array[StateGuiCount].display_1,display_1); 	// Display string Line 1 (16 chars) (empty string for no change)
	strcpy(state_array[StateGuiCount].display_2,display_2); 	// Display string Line 2 (16 chars)
	state_array[StateGuiCount].k1 = k1;						// Key1 goto state name (Use <STATE_NOP> for no action)
	state_array[StateGuiCount].k2 = k2;						// Key2 goto state name
	state_array[StateGuiCount].state_enter = state_enter;	// Callback on state entry (Use <ACTION_NOP> for no action)
	state_array[StateGuiCount].state_loop  = state_loop;	// Callback on state loop
	state_array[StateGuiCount].state_exit  = state_exit;	// Callback on state exit
	StateGuiCount++;
}

void set_lcd_display(int32_t line,char *buffer) {
	if (0 == line) {
		strncpy(state_array[state_now].display_1,buffer,LCD_DISPLAY_POS_MAX);
		state_array[state_now].display_1[LCD_DISPLAY_POS_MAX]='\0';
	}
	if (1 == line) {
		strncpy(state_array[state_now].display_2,buffer,LCD_DISPLAY_POS_MAX);
		state_array[state_now].display_2[LCD_DISPLAY_POS_MAX]='\0';
	}
}

static void display_state() {

	sprintf(r_control.lcd_line0,"%-16s",state_array[state_now].display_1);
	sprintf(r_control.lcd_line1,"%-16s",state_array[state_now].display_2);

	if (verbose && (0x0000 == (state_array[state_now].state_flags & STATE_NO_VERBOSE))) {
		log("\n");
		sprintf(buffer,"/----------------\\ State=%s\n",state_array[state_now].state_name);
		log(buffer);
		sprintf(buffer,"|%s|\n",r_control.lcd_line0);
		log(buffer);
		sprintf(buffer,"|%s|\n",r_control.lcd_line1);
		log(buffer);
		log("\\----------------/\n");
		sprintf(buffer,"1:=%s, 2=%s\n",
			(STATE_NOP == state_array[state_now].k1)?"None":state_array[state_now].k1,
			(STATE_NOP == state_array[state_now].k2)?"None":state_array[state_now].k2);
		log(buffer);
	}

	// Send text to screen
	if (IO_LCD_ENABLE) {
		groveLcdPrint(i2c, 0, 0, r_control.lcd_line0, strlen(r_control.lcd_line0));
		groveLcdPrint(i2c, 1, 0, r_control.lcd_line1, strlen(r_control.lcd_line1));
	}
}


static int32_t find_state(char *select_state) {
	int32_t i;
	if (STATE_NOP == select_state)
		return STATE_NOT_FOUND;
	for (i=0;i<StateGuiMax;i++) {
		if (0 == strcmp(select_state,state_array[i].state_name))
			return i;
	}
	return STATE_NOT_FOUND;
}

static void do_goto_state(char *select_state_name, bool skip_display) {
	int32_t display_this_state=true;

	// skip if next state is NOP
	if (STATE_NOP == select_state_name)
		return;

	// Find state
	int32_t state_next=find_state(select_state_name);
	if (STATE_NOT_FOUND == state_next) {
		log("\n");
		log_val("ERROR: Could not find state %s\n",select_state_name);
		log("\n");
		return;
	}

	// self-test mode?
	if (self_test) return;

	// execute any state epilog function
	if (ACTION_NOP != state_array[state_now].state_exit) {
		state_callback(state_array[state_now].state_exit);
	}

	// assert new state
	state_prev= state_now;
	state_now = state_next;

	log_val("NEW_STATE=%s\n",state_array[state_now].state_name);

	// execute any state prolog function
	if (ACTION_NOP != state_array[state_now].state_enter) {
		int32_t expected_state=state_now;
		state_callback(state_array[state_now].state_enter);
		// See if the callback changed the state
		if (state_now != expected_state) {
			display_this_state=false;
		}
	}

	// display the new state
	if ((false == skip_display) && (0x0000 == (state_array[state_now].state_flags & STATE_NO_DISPLAY)))
		display_state();
}

void goto_state(char *select_state_name) {
	do_goto_state(select_state_name, false);
}
void jump_state(char *select_state_name) {
	do_goto_state(select_state_name, true);
}


/*
 * State callbacks; Enter, Loop, and Exit
 *
 */


static void S_Start_Home_enter () {
	init_rocket_game(ROCKET_HOME_X, ROCKET_HOME_Y, ROCKET_HOME_Z, GAME_FUEL_NOLIMIT, GAME_GRAVITY_NONE,GAME_AT_HOME);
	jump_state("S_Main_Play");
}


/**** CALIBRATION ********************************************************/

static void S_Calibrate_enter () {
	// self-test mode?
	if (self_test) return;

	// tell the motors to pretend that they are game start position, to avoid step limits 
	init_rocket_game(0, 0, Z_POS_MAX/2, GAME_FUEL_NOLIMIT, GAME_GRAVITY_NONE,GAME_AT_HOME);
}

static void S_Calibrate_loop () {
	int32_t nw_inc=0,ne_inc=0,sw_inc=0,se_inc=0;
	int32_t x_delta,y_delta,z_delta;

	// self-test mode?
	if (self_test) return;

	// Thruster Z is 'proportion-up or 'proportion-down' or 'off'
	z_delta = r_control.analog_z - JOYSTICK_Z_MID;
	if (z_delta < -JOYSTICK_DELTA_Z_MIN) {
		z_delta = (z_delta+JOYSTICK_DELTA_Z_MIN)*ROCKET_CALIBRATE_INC_Z;
	}
	if (z_delta > JOYSTICK_DELTA_Z_MIN) {
		z_delta = (z_delta-JOYSTICK_DELTA_Z_MIN)*ROCKET_CALIBRATE_INC_Z;
	}

	// Convert joystick to tower selection
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
	
	// scale Z from nm to steps
	z_delta = (z_delta * 10L) / ROCKET_TOWER_STEPS_PER_UM10;

	// if quadarant, then only affect that tower
	if      ((x_delta < 0) && (y_delta > 0)) nw_inc=z_delta;
	else if ((x_delta > 0) && (y_delta > 0)) ne_inc=z_delta;
	else if ((x_delta < 0) && (y_delta < 0)) sw_inc=z_delta;
	else if ((x_delta > 0) && (y_delta < 0)) se_inc=z_delta;
	else if ((x_delta == 0) && (y_delta == 0)) {
		// if center, then affect all towers
		nw_inc=z_delta;
		ne_inc=z_delta;
		sw_inc=z_delta;
		se_inc=z_delta;
	}

	// send the increment
	rocket_increment_send (nw_inc, ne_inc, sw_inc, se_inc);

}

static void S_Calibrate_Done_enter () {
	// self-test mode?
	if (self_test) return;

	init_rocket_game(ROCKET_CALIBRATE_X, ROCKET_CALIBRATE_Y, ROCKET_CALIBRATE_Z, GAME_FUEL_NOLIMIT, GAME_GRAVITY_NONE,GAME_AT_HOME);
	jump_state("S_Main_Play");
}

static void S_Calibrate_Status_enter  () {
	PRINT("\nStep Status:NW=%ld, NE=%ld, SW=%ld, SE=%ld\n",
		r_towers[ROCKET_TOWER_NW].step_count,
		r_towers[ROCKET_TOWER_NE].step_count,
		r_towers[ROCKET_TOWER_SW].step_count,
		r_towers[ROCKET_TOWER_SE].step_count);
	PRINT("        nm :NW=%ld, NE=%ld, SW=%ld, SE=%ld\n\n",
		(r_towers[ROCKET_TOWER_NW].step_count*ROCKET_TOWER_STEPS_PER_UM10)/UM10_PER_MILLIMETER,
		(r_towers[ROCKET_TOWER_NE].step_count*ROCKET_TOWER_STEPS_PER_UM10)/UM10_PER_MILLIMETER,
		(r_towers[ROCKET_TOWER_SW].step_count*ROCKET_TOWER_STEPS_PER_UM10)/UM10_PER_MILLIMETER,
		(r_towers[ROCKET_TOWER_SE].step_count*ROCKET_TOWER_STEPS_PER_UM10)/UM10_PER_MILLIMETER);
	jump_state("S_Calibrate");
}

static void S_Test_Motor_Status_loop () {
	uint8_t buf[10];
	uint32_t len = 1;
	buf[0] = (uint8_t) '?';

	// self-test mode?
	if (self_test) return;

	i2c_read(i2c,buf,len,ROCKET_MOTOR_I2C_ADDRESS);
	sprintf(state_array[state_now].display_1,"Status=%4d",buf[0]);
	display_state();
}


/**** GAME PLAY ********************************************************/

static void updateLedDisplays () {
	// self-test mode?
	if (self_test) return;

	// show fuel
	send_LED_Backpack(r_space.rocket_fuel);
	// show height
	send_Led1(r_space.rocket_z/SCALE_GAME_UMETER_TO_MOON_METER);
	// show speed
	if (GAME_XYZ_MOVE != r_game.game) {
		send_Led2((r_space.rocket_delta_x+r_space.rocket_delta_y+r_space.rocket_delta_z)/SCALE_GAME_UMETER_TO_MOON_METER);
	} else {
		int32_t psuedo_speed=0;
		if (JOYSTICK_DELTA_XY_MIN < abs(r_space.thrust_x))
			psuedo_speed += abs(r_space.thrust_x) - JOYSTICK_DELTA_XY_MIN;
		if (JOYSTICK_DELTA_XY_MIN < abs(r_space.thrust_y))
			psuedo_speed += abs(r_space.thrust_y) - JOYSTICK_DELTA_XY_MIN;
		if (JOYSTICK_DELTA_Z_MIN < abs(r_space.thrust_z))
			psuedo_speed += abs(r_space.thrust_z) - JOYSTICK_DELTA_Z_MIN;
		if (999 < psuedo_speed) psuedo_speed = 999;
		send_Led2(psuedo_speed);
	}
}

static void S_Main_Play_enter () {
	// self-test mode?
	if (self_test) return;

	send_Sound(SOUND_READY);
	send_NeoPixel(NEOPIXEL_READY);
}

static void S_Game_Start_enter () {
	// self-test mode?
	if (self_test) return;

	init_game();

	// start the show
	send_Sound(SOUND_PLAY);
	send_NeoPixel(NEOPIXEL_PLAY);
	updateLedDisplays();
}

static void S_Game_Start_loop () {
	// self-test mode?
	if (self_test) return;

	uint8_t position_status = query_rocket_progress();
	sprintf(state_array[state_now].display_1,"Progress=%4d",position_status);
	display_state();
	if (100 == position_status) {
		// we are done moving
		jump_state("S_Game_Play");
	}
}

static void S_Game_Play_loop () {

	// self-test mode?
	if (self_test) return;
	
	// compute the rocket position
	compute_rocket_next_position();

	// compute the tower cable lengths
	compute_rocket_cable_lengths();

	// Move the tower cables
	move_rocket_next_position();

	// Landed?
	if (GAME_XYZ_FLIGHT != r_game.game) {
		if (r_space.rocket_z <= 0) {
			goto_state("S_Game_Done");
			return;
		}
	}

	// display the rocket state
	if        (GAME_DISPLAY_RAW_XYZF  == r_game.play_display_mode) {
		// display the rocket state
		//	 "1234567890123456",
		sprintf(state_array[state_now].display_1,"X=%5d  Y=%5d",r_space.rocket_x/1000,r_space.rocket_y/1000);
		sprintf(state_array[state_now].display_2,"Z=%5d  f=%5d",r_space.rocket_z/1000,r_space.rocket_fuel);
		display_state();
	} else if (GAME_DISPLAY_RAW_CABLE == r_game.play_display_mode) {
		// display the rocket state
		sprintf(state_array[state_now].display_1,"NW=%4d NE=%4d",
			r_towers[ROCKET_TOWER_NW].length_goal/1000,
			r_towers[ROCKET_TOWER_NE].length_goal/1000);
		sprintf(state_array[state_now].display_2,"SW=%4d SE=%4d",
			r_towers[ROCKET_TOWER_SW].length_goal/1000,
			r_towers[ROCKET_TOWER_SE].length_goal/1000);
		display_state();
	} else if (GAME_DISPLAY_RAW_STEPS == r_game.play_display_mode)  {
		sprintf(buffer,"NW=%05d E=%05d",
			r_towers[ROCKET_TOWER_NW].step_count,
			r_towers[ROCKET_TOWER_NE].step_count);
		set_lcd_display(0,buffer);
		sprintf(buffer,"SW=%05d E=%05d",
			r_towers[ROCKET_TOWER_SW].step_count,
			r_towers[ROCKET_TOWER_SE].step_count);
		set_lcd_display(1,buffer);
		display_state();
	} else {
		sprintf(buffer,"Z=%02d X=%03d Y=%03d",
			r_space.rocket_z/SCALE_GAME_UMETER_TO_MOON_CMETER,
			r_space.rocket_x/SCALE_GAME_UMETER_TO_MOON_CMETER,
			r_space.rocket_y/SCALE_GAME_UMETER_TO_MOON_CMETER
			);
		set_lcd_display(0,buffer);
		sprintf(buffer,"S=%04d F=%04d",r_space.rocket_delta_z,r_space.rocket_fuel);
		set_lcd_display(1,buffer);
		display_state();
	}

	// update the displays
	updateLedDisplays();
}

static void S_Game_Done_enter () {
	// self-test mode?
	if (self_test) return;

	if (SAFE_UMETER_PER_SECOND < abs(r_space.rocket_delta_z)) {
		sprintf(buffer,"CRASH :-( S=%04d",abs(r_space.rocket_delta_z));
		// stop the show
		send_Sound(SOUND_CRASH);
		send_NeoPixel(NEOPIXEL_CRASH);
	} else {
		sprintf(buffer,"WIN! :-) S=%04d",abs(r_space.rocket_delta_z));
		// stop the show
		send_Sound(SOUND_LAND);
		send_NeoPixel(NEOPIXEL_LAND);
	}
	set_lcd_display(0,buffer);
}

static void S_Game_Display_Next_enter () {
	if        (GAME_DISPLAY_NORMAL == r_game.play_display_mode) {
		r_game.play_display_mode = GAME_DISPLAY_RAW_XYZF;
	} else if (GAME_DISPLAY_RAW_XYZF == r_game.play_display_mode) {
		r_game.play_display_mode = GAME_DISPLAY_RAW_CABLE;
	} else if (GAME_DISPLAY_RAW_CABLE == r_game.play_display_mode) {
		r_game.play_display_mode = GAME_DISPLAY_RAW_STEPS;
	} else {
		r_game.play_display_mode = GAME_DISPLAY_NORMAL;
	}
	jump_state("S_Game_Play");
}


/**** GAME OPTIONS SELECT ***************************************************/

static void S_Opt_Game_Z_Enter () {
	r_game.game = GAME_Z_LAND;
	jump_state("S_Main_Play");
}
static void S_Opt_Game_XYZ_Enter () {
	r_game.game = GAME_XYZ_LAND;
	jump_state("S_Main_Play");
}
static void S_Opt_Game_Flight_Enter () {
	r_game.game = GAME_XYZ_FLIGHT;
	jump_state("S_Main_Play");
}
static void S_Opt_Game_Move_Enter () {
	r_game.game = GAME_XYZ_MOVE;
	jump_state("S_Main_Play");
}
static void S_Opt_Game_Auto_Enter () {
	r_game.game = GAME_XYZ_AUTO;
	jump_state("S_Main_Play");
}

static void S_Opt_Gravity_Full_Enter () {
	r_game.gravity_option = GAME_GRAVITY_NORMAL;
	jump_state("S_Main_Play");
}
static void S_Opt_Gravity_High_Enter () {
	r_game.gravity_option = GAME_GRAVITY_HIGH;
	jump_state("S_Main_Play");
}
static void S_Opt_Gravity_None_Enter () {
	r_game.gravity_option = GAME_GRAVITY_NONE;
	jump_state("S_Main_Play");
}
static void S_Opt_Gravity_Negative_Enter () {
	r_game.gravity_option = GAME_GRAVITY_NEGATIVE;
	jump_state("S_Main_Play");
}

static void S_Opt_Fuel_Normal_Enter () {
	r_game.fuel_option = GAME_FUEL_NORMAL;
	jump_state("S_Main_Play");
}
static void S_Opt_Fuel_Low_Enter () {
	r_game.fuel_option = GAME_FUEL_LOW;
	jump_state("S_Main_Play");
}
static void S_Opt_Fuel_Nolimit_Enter () {
	r_game.fuel_option = GAME_FUEL_NOLIMIT;
	jump_state("S_Main_Play");
}

static void S_Opt_Pos_Center_Enter () {
	r_game.start_option = GAME_START_CENTER;
	jump_state("S_Main_Play");
}
static void S_Opt_Pos_Random_Enter () {
	r_game.start_option = GAME_START_RANDOM;
	jump_state("S_Main_Play");
}


/**** TEST FUNCTIONS ********************************************************/

static void S_Test_enter () {
	// self-test mode?
	if (self_test) return;

	// stop current sound and Neo
	send_Sound(SOUND_QUIET);
	send_NeoPixel(NEOPIXEL_QUIET);
}

static void S_IO_STATE_loop () {
	// self-test mode?
	if (self_test) return;

	// display the raw I/O input controls for board bringup
	sprintf(buffer,"[I/O] X=%3d Y=%3d Z=%3d A=%d B=%d | D4=%d, D5=%d, D6=%d, D7=%d, D8=%d\n",
		r_control.analog_x,
		r_control.analog_y,
		r_control.analog_z,
		r_control.button_a,
		r_control.button_b,
		gpioInputGet(4),
		gpioInputGet(5),
		gpioInputGet(6),
		gpioInputGet(7),
		gpioInputGet(8)
		);
	log(buffer);
}


/**** TEST I2C to Rocket_Display ****************************************************/

static void S_Test_I2C_enter () {
	char *msg = "x";
	static uint8_t x = 0;
	uint8_t buf[100];

	// self-test mode?
	if (self_test) return;

	strncpy(buf,msg,strlen(msg));
	buf[strlen(msg)]=x;

	PRINT("FOO\n");
	send_I2c_slave(buf, strlen(msg)+1);
	PRINT("BAR\n");

	x++;

	send_Led1(1234+x);
	send_Led2(42+x);
	send_Pan_Tilt(25,57);
	send_Led_Rgb(50,100,200);
	send_NeoPixel(8);
	send_Sound(9);


	jump_state("S_Test_I2C_Select");
}


/**** TEST LARGE 7-SEGMENT  ********************************************************/

static void S_Test_Segment_Open_enter () {
	// self-test mode?
	if (self_test) return;

	if (IO_LED_BACKPACK_ENABLE) {
		bp_setdevice(i2c);
		bp_begin();
		bp_clear();
	}
	jump_state("S_Test_Segment_Select");
}

static void S_Test_Segment_enter () {
	static uint8_t x = 123;

	// self-test mode?
	if (self_test) return;

	if (IO_LED_BACKPACK_ENABLE) {
		seg_writeNumber(x);
	}
	x++;
	jump_state("S_Test_Segment_Select");
}


/**** TEST GAME SIMULATION  ********************************************************/

static int32_t title_loop=0;
static char* resume_state_name="S_Main_Play";
static void do_Simulation_Pause_enter () {
	resume_state_name = state_array[state_prev].state_name;
}
static void do_Simulation_Resume_enter () {
	goto_state(resume_state_name);
}

static void S_Test_Simulation_Meters_enter () {
	sprintf(buffer,"=== Rocket Controls to Position in game space ===\n");
	// preset start location, and options
	r_game.game=GAME_XYZ_MOVE;
	init_rocket_game (0, 0, Z_POS_MAX/2, GAME_FUEL_NOLIMIT, GAME_GRAVITY_NONE,GAME_SIMULATE);
	title_loop=0;
}

static void do_Simulation_Meters_loop (bool do_millimeters) {
	// self-test mode?
	if (self_test) return;

	// update the rocket position
	compute_rocket_next_position();

	if (0 == title_loop) {
		if (!do_millimeters)
			PRINT("----------------------------   jx  jy  jz  ---    dx    dy    dz ---    newx    newy    newz ---\n");
		else
			PRINT("----------------------------   jx  jy  jz  ---  dx  dy  dz --- newx newy newz ---\n");
	}
	if (++title_loop > 10) title_loop=0;

	// display the raw I/O input controls for board bringup
	if (!do_millimeters) {
		sprintf(buffer,"[Thrust Joy=>Delta=>uMeters] (%3d,%3d,%3d) => (%5d,%5d,%5d) => (%7d,%7d,%7d) \n",
			r_control.analog_x,
			r_control.analog_y,
			r_control.analog_z,
			r_space.rocket_delta_x,
			r_space.rocket_delta_y,
			r_space.rocket_delta_z,
			r_space.rocket_goal_x,
			r_space.rocket_goal_y,
			r_space.rocket_goal_z
			);
	} else {
		sprintf(buffer,"[Thrust Joy=>Delta=>mMeters] (%3d,%3d,%3d) => (%5d,%5d,%5d) => (%5d,%5d,%5d) \n",
			r_control.analog_x,
			r_control.analog_y,
			r_control.analog_z,
			r_space.rocket_delta_x/1000,
			r_space.rocket_delta_y/1000,
			r_space.rocket_delta_z/1000,
			r_space.rocket_goal_x/1000,
			r_space.rocket_goal_y/1000,
			r_space.rocket_goal_z/1000
			);
	}
	log(buffer);
}

static void S_Test_Simulation_MicroMeters_loop () {
	do_Simulation_Meters_loop(false);
}
static void S_Test_Simulation_MilliMeters_loop () {
	do_Simulation_Meters_loop(true);
}

static void S_Test_Simulation_Cables_enter () {
	sprintf(buffer,"=== Rocket Position (mM) to Cable Lengths (NW,NE,SW,SE) ===\n");

	// preset start location, and options
	r_game.game=GAME_XYZ_MOVE;
	init_rocket_game (0, 0, Z_POS_MAX/2, GAME_FUEL_NOLIMIT, GAME_GRAVITY_NONE,GAME_SIMULATE);
	title_loop=0;
}

static void S_Test_Simulation_Cables_loop () {

	// self-test mode?
	if (self_test) return;

	// update the rocket position
	compute_rocket_next_position();

	// update the tower cable lengths
	compute_rocket_cable_lengths();

	if (0 == title_loop) {
		PRINT("--mm------mm------ x   y   z--|--- NW  NE  SW  SE ---\n");
	}
	if (++title_loop > 10) title_loop=0;

	// display the raw I/O input controls for board bringup
	sprintf(buffer,    "[Pos => Cables] (%3d,%3d,%3d) => (%3d,%3d,%3d,%3d)\n",
		r_space.rocket_goal_x/1000,
		r_space.rocket_goal_y/1000,
		r_space.rocket_goal_z/1000,
		r_towers[ROCKET_TOWER_NW].length_goal/1000,
		r_towers[ROCKET_TOWER_NE].length_goal/1000,
		r_towers[ROCKET_TOWER_SW].length_goal/1000,
		r_towers[ROCKET_TOWER_SE].length_goal/1000
		);
	log(buffer);
}

static void S_Test_Simulation_Steps_enter () {
	sprintf(buffer,"=== Rocket Position (mM) to Cable Steps (NW,NE,SW,SE) ===\n");

	// preset start location, and options
	r_game.game=GAME_XYZ_MOVE;
	init_rocket_game (0, 0, Z_POS_MAX/2, GAME_FUEL_NOLIMIT, GAME_GRAVITY_NONE,GAME_SIMULATE);
	title_loop=0;
}

static void S_Test_Simulation_Steps_loop () {

	// self-test mode?
	if (self_test) return;

	// update the rocket position
	compute_rocket_next_position();

	// update the tower cable lengths
	compute_rocket_cable_lengths();

	if (0 == title_loop) {
		PRINT("-------------------------------    x     y     z ------ NW   NE   SW   SE --- NW   NE   SW   SE ---\n");
	}
	if (++title_loop > 10) title_loop=0;

	// display the raw I/O input controls for board bringup
	sprintf(buffer,"[PosDiff => LengthDiff,Steps] (%5d,%5d,%5d) => (%4d,%4d,%4d,%4d) (%4d,%4d,%4d,%4d)\n",
		(r_space.rocket_goal_x-r_space.rocket_x),
		(r_space.rocket_goal_y-r_space.rocket_y),
		(r_space.rocket_goal_z-r_space.rocket_z),
		(r_towers[ROCKET_TOWER_NW].length_goal-r_towers[ROCKET_TOWER_NW].length)/1000,
		(r_towers[ROCKET_TOWER_NE].length_goal-r_towers[ROCKET_TOWER_NW].length)/1000,
		(r_towers[ROCKET_TOWER_SW].length_goal-r_towers[ROCKET_TOWER_NW].length)/1000,
		(r_towers[ROCKET_TOWER_SE].length_goal-r_towers[ROCKET_TOWER_NW].length)/1000,
		r_towers[ROCKET_TOWER_NW].step_diff,
		r_towers[ROCKET_TOWER_NE].step_diff,
		r_towers[ROCKET_TOWER_SW].step_diff,
		r_towers[ROCKET_TOWER_SE].step_diff
		);
	log(buffer);

	// Move the tower cables
	move_rocket_next_position();
}


/**** TEST PAN/TILT ********************************************************/

static int32_t antennae_number=0;
static int32_t antennae_pan=(PAN_MID)*4;
static int32_t antennae_tilt=(PAN_MID)*4;
static void S_Test_Antennae_Select_enter () {
	antennae_number=0;
	jump_state("S_Test_Antennae");
}
static void S_Test_Antennae_enter () {
	// initialize current PWM
}
static void S_Test_Antennae_loop () {
	static int32_t value_z_prev=0;
	// self-test mode?
	if (self_test) return;

	// map 0..1023 to 0..255, centered on joystick middle value
	if (0 == antennae_number) {
		antennae_pan = r_control.analog_z/4;
	} else {
		antennae_tilt = r_control.analog_z/4;
	}

	// pass Z to current motor's speed
	if (4 < abs(value_z_prev-r_control.analog_z)) {
		sprintf(buffer,"[%c] Pan=%0x,Tilt=%0x, Z=%04d\n",
			(0 == antennae_number) ? 'P':'T',
			antennae_pan,antennae_tilt,r_control.analog_z);
		log(buffer);
		send_Pan_Tilt(antennae_pan,antennae_tilt);
		value_z_prev = r_control.analog_z;
	}
}
static void S_Test_Antennae_exit () {
	// stop current PWM
}
static void S_Test_Antennae_Next_enter () {
	antennae_number++;
	if (antennae_number>1)
		antennae_number=0;
	jump_state("S_Test_Antennae");
}

/**** TEST LED-RGB ********************************************************/

static int32_t ledrgb=0;
static void S_Test_LedRgb_enter () {
	ledrgb=0;
}
static void S_Test_LedRgb_loop () {
	int32_t red  = (r_control.analog_z & 0x000f)      * 16;
	int32_t green= (r_control.analog_z & 0x00f0 >> 4) * 16;
	int32_t blue = (r_control.analog_z & 0x0f00 >> 8) * 64;

	// self-test mode?
	if (self_test) return;

	// pass Z to LED RGB
	sprintf(buffer,"[LED RGB] Z=%#04x RGB=%d,%d,%d\n",
		r_control.analog_z,
		red,
		green,
		blue
		);
	log(buffer);
}
static void S_Test_LedRgb_exit () {
	// turn LED RGB off
}


/**** TEST SOUNDS ********************************************************/

static int32_t sound_number=0;
static void S_Test_Sound_Select_enter () {
	sound_number=0;
	jump_state("S_Test_Sound");
}
static void S_Test_Sound_enter () {
	// self-test mode?
	if (self_test) return;

	// initialize current sound and Neo
	send_Sound(sound_number);
	send_NeoPixel(sound_number);
}
static void S_Test_Sound_loop () {
	// self-test mode?
	if (self_test) return;

	// pass Z to current motor's speed
	sprintf(buffer,"[Sound & Neo %d] Play\n",sound_number);
	log(buffer);
	send_Sound(sound_number);
	send_NeoPixel(sound_number);
}
static void S_Test_Sound_exit () {
	// self-test mode?
	if (self_test) return;

	// stop current sound and Neo
	send_Sound(SOUND_QUIET);
	send_NeoPixel(NEOPIXEL_QUIET);
}
static void S_Test_Sound_Next_enter () {
	sound_number++;
	if (sound_number>SOUND_MAX)
		sound_number=0;
	jump_state("S_Test_Sound");
}


/**** TEST MOTOR STEPPING ********************************************************/

static uint32_t motor_nextset_value=1L;
static void test_set_motor_position(uint32_t motor_position) {
	r_towers[ROCKET_TOWER_NW].step_count = motor_position;
	r_towers[ROCKET_TOWER_NE].step_count = motor_position;
	r_towers[ROCKET_TOWER_SW].step_count = motor_position;
	r_towers[ROCKET_TOWER_SE].step_count = motor_position;
	rocket_position_send();
	rocket_command_send(ROCKET_MOTOR_CMD_PRESET);
}
static void S_TestMotor_NextSet_enter () {
	PRINT("\nMotor_NextSet=%lx\n",motor_nextset_value);
	test_set_motor_position(motor_nextset_value);
	motor_nextset_value = motor_nextset_value << 1;
	if (motor_nextset_value > 0x100000L) motor_nextset_value = 0;
	jump_state("S_TestMotor_NextSet");
}
static void S_TestMotor_NextSet_Done_enter () {
	test_set_motor_position(0L);
	jump_state("S_TestMotor_PlusStep");
}

static void S_TestMotor_PlusStep_enter () {
	rocket_increment_send (1, 1, 1, 1);
	jump_state("S_TestMotor_PlusStep");
}

static void S_TestMotor_MinusStep_enter () {
	rocket_increment_send (-1, -1, -1, -1);
	jump_state("S_TestMotor_MinusStep");
}

static void S_TestMotor_Plus360_enter () {
	rocket_increment_send (200, 200, 200, 200);
	jump_state("S_TestMotor_Plus360");
}

static void S_TestMotor_Minus360_enter () {
	rocket_increment_send (-200, -200, -200, -200);
	jump_state("S_TestMotor_Minus360");
}


/**** TEST CABLE CALCULATIONS ********************************************************/

void cable_calc_test(char *msg, int32_t x, int32_t y) {
	int32_t i;

	for (i=Z_POS_MAX;i>=0;i-=Z_POS_MAX/4) {
		r_space.rocket_goal_x = x;
		r_space.rocket_goal_y = y;
		r_space.rocket_goal_z = i;
		compute_rocket_cable_lengths();
		PRINT("[%s] Cable(%4d,%4d,%4d)=%5d,%5d,%5d,%5d\n",
			msg,
			x/1000,y/1000,i/1000,
			r_towers[ROCKET_TOWER_NW].length_goal/1000,
			r_towers[ROCKET_TOWER_NE].length_goal/1000,
			r_towers[ROCKET_TOWER_SW].length_goal/1000,
			r_towers[ROCKET_TOWER_SE].length_goal/1000
			);
	}
}

void cable_steps_start(int32_t x, int32_t y, int32_t z) {
	r_space.rocket_x = x;
	r_space.rocket_y = y;
	r_space.rocket_z = z;
	r_space.rocket_goal_x = x;
	r_space.rocket_goal_y = y;
	r_space.rocket_goal_z = z;

	compute_rocket_cable_lengths();
	move_rocket_next_position();
}

void cable_steps_move(int32_t x, int32_t y, int32_t z) {
	r_space.rocket_goal_x = x;
	r_space.rocket_goal_y = y;
	r_space.rocket_goal_z = z;
	compute_rocket_cable_lengths();
	PRINT("Start(%8d,%8d,%8d) Move=(%8d,%8d,%8d) Steps(%8d,%8d,%8d,%8d)\n",
		r_space.rocket_x,
		r_space.rocket_y,
		r_space.rocket_z,
		r_space.rocket_goal_x-r_space.rocket_x,
		r_space.rocket_goal_y-r_space.rocket_y,
		r_space.rocket_goal_z-r_space.rocket_z,
		r_towers[ROCKET_TOWER_NW].step_diff,
		r_towers[ROCKET_TOWER_NE].step_diff,
		r_towers[ROCKET_TOWER_SW].step_diff,
		r_towers[ROCKET_TOWER_SE].step_diff
		);
	move_rocket_next_position();
}


/**** TEST STATE SANITY ********************************************************/

static void S_Test_Sanity_enter () {
	int32_t i,j;
	bool state_is_called;

	// already in self-test mode?
	if (self_test) return;
	self_test=true;

	PRINT("\n=== Self Test: State table =%d of %d ===\n",StateGuiCount,StateGuiMax);

	// check all state entries ...
	for (i=0;i<StateGuiCount;i++) {
		// check state goto K1 and K2
		jump_state(state_array[i].k1);
		jump_state(state_array[i].k2);
		// check state callbacks
		state_callback(state_array[i].state_enter);
		state_callback(state_array[i].state_loop);
		state_callback(state_array[i].state_exit);

		// check that this state is called by someone
		state_is_called=false;
		for (j=0;j<StateGuiCount;j++) {
			if ((0 == strcmp(state_array[i].state_name,state_array[j].k1)) ||
			    (0 == strcmp(state_array[i].state_name,state_array[j].k2)) ) {
			    state_is_called=true;
			}
		}
		if (!state_is_called) {
			if (0x0000 != (state_array[i].state_flags & STATE_FROM_CALLBACK)) {
				PRINT("NOTE: Callback to otherwise Orphan State=%s\n",state_array[i].state_name);
			} else {
				PRINT("ERROR: Orphan State=%s\n",state_array[i].state_name);
			}
		}

		// check that this state is registered twice
		state_is_called=false;
		for (j=i+1;j<StateGuiCount;j++) {
			if (0 == strcmp(state_array[i].state_name,state_array[j].state_name)) {
				PRINT("ERROR: Duplicate State=%s (%d and %d)\n",state_array[i].state_name,i,j);
			}
		}

	}
	PRINT("========================================\n\n");

	PRINT("==== sqrt test ===\n");
	for (i=0;i<(Z_POS_MAX*2);i+=(Z_POS_MAX/32)) {
		j=i/1000;
		j=j*j;
		PRINT("%4d:Sqrt(%d)=%d\n",i/1000,j,sqrt_with_accuracy(j, 500));
	}

	PRINT("\n==== Cable Length Calculation Test ===\n");
	cable_calc_test("NW",r_towers[ROCKET_TOWER_NW].pos_x,r_towers[ROCKET_TOWER_NW].pos_y);
	cable_calc_test("NE",r_towers[ROCKET_TOWER_NE].pos_x,r_towers[ROCKET_TOWER_NE].pos_y);
	cable_calc_test("SW",r_towers[ROCKET_TOWER_SW].pos_x,r_towers[ROCKET_TOWER_SW].pos_y);
	cable_calc_test("SE",r_towers[ROCKET_TOWER_SE].pos_x,r_towers[ROCKET_TOWER_SE].pos_y);
	cable_calc_test("CN",0,0);

	PRINT("\n==== Cable Steps Calculation Test ===\n");
	PRINT("ROCKET_TOWER_NW:\n");
	cable_steps_start(r_towers[ROCKET_TOWER_NW].pos_x     ,r_towers[ROCKET_TOWER_NW].pos_y     ,r_towers[ROCKET_TOWER_NW].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_NW].pos_x+1000,r_towers[ROCKET_TOWER_NW].pos_y     ,r_towers[ROCKET_TOWER_NW].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_NW].pos_x+1000,r_towers[ROCKET_TOWER_NW].pos_y+1000,r_towers[ROCKET_TOWER_NW].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_NW].pos_x     ,r_towers[ROCKET_TOWER_NW].pos_y+1000,r_towers[ROCKET_TOWER_NW].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_NW].pos_x     ,r_towers[ROCKET_TOWER_NW].pos_y     ,r_towers[ROCKET_TOWER_NW].pos_z);
	PRINT("ROCKET_TOWER_NE:\n");
	cable_steps_start(r_towers[ROCKET_TOWER_NE].pos_x     ,r_towers[ROCKET_TOWER_NE].pos_y     ,r_towers[ROCKET_TOWER_NE].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_NE].pos_x+1000,r_towers[ROCKET_TOWER_NE].pos_y     ,r_towers[ROCKET_TOWER_NE].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_NE].pos_x+1000,r_towers[ROCKET_TOWER_NE].pos_y+1000,r_towers[ROCKET_TOWER_NE].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_NE].pos_x     ,r_towers[ROCKET_TOWER_NE].pos_y+1000,r_towers[ROCKET_TOWER_NE].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_NE].pos_x     ,r_towers[ROCKET_TOWER_NE].pos_y     ,r_towers[ROCKET_TOWER_NE].pos_z);
	PRINT("ROCKET_TOWER_SW:\n");
	cable_steps_start(r_towers[ROCKET_TOWER_SW].pos_x     ,r_towers[ROCKET_TOWER_SW].pos_y     ,r_towers[ROCKET_TOWER_SW].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_SW].pos_x+1000,r_towers[ROCKET_TOWER_SW].pos_y     ,r_towers[ROCKET_TOWER_SW].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_SW].pos_x+1000,r_towers[ROCKET_TOWER_SW].pos_y+1000,r_towers[ROCKET_TOWER_SW].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_SW].pos_x     ,r_towers[ROCKET_TOWER_SW].pos_y+1000,r_towers[ROCKET_TOWER_SW].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_SW].pos_x     ,r_towers[ROCKET_TOWER_SW].pos_y     ,r_towers[ROCKET_TOWER_SW].pos_z);
	PRINT("ROCKET_TOWER_SE:\n");
	cable_steps_start(r_towers[ROCKET_TOWER_SE].pos_x     ,r_towers[ROCKET_TOWER_SE].pos_y     ,r_towers[ROCKET_TOWER_SE].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_SE].pos_x+1000,r_towers[ROCKET_TOWER_SE].pos_y     ,r_towers[ROCKET_TOWER_SE].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_SE].pos_x+1000,r_towers[ROCKET_TOWER_SE].pos_y+1000,r_towers[ROCKET_TOWER_SE].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_SE].pos_x     ,r_towers[ROCKET_TOWER_SE].pos_y+1000,r_towers[ROCKET_TOWER_SE].pos_z);
	cable_steps_move( r_towers[ROCKET_TOWER_SE].pos_x     ,r_towers[ROCKET_TOWER_SE].pos_y     ,r_towers[ROCKET_TOWER_SE].pos_z);
	PRINT("Center:\n");
	cable_steps_start(0     ,0     , r_towers[ROCKET_TOWER_NW].pos_z/2);
	cable_steps_move( 0+1000,0     ,(r_towers[ROCKET_TOWER_NW].pos_z/2));
	cable_steps_move( 0+1000,0+1000,(r_towers[ROCKET_TOWER_NW].pos_z/2));
	cable_steps_move( 0     ,0+1000,(r_towers[ROCKET_TOWER_NW].pos_z/2));
	cable_steps_move( 0     ,0     ,(r_towers[ROCKET_TOWER_NW].pos_z/2));
	cable_steps_move( 0     ,0     ,(r_towers[ROCKET_TOWER_NW].pos_z/2)+1000);
	cable_steps_move( 0     ,0     ,(r_towers[ROCKET_TOWER_NW].pos_z/2)-1000);

	PRINT("==================\n\n");

	// finally, reset game defaults
	init_main();
	self_test=false;
	jump_state("S_Main_Play");
}


/**** SHUTDOWN ********************************************************/

void S_Shutdown_enter () {
	/* move the rocket to the default home position, for power off */
	init_rocket_game(ROCKET_HOME_X, ROCKET_HOME_Y, ROCKET_HOME_Z, GAME_FUEL_NOLIMIT, GAME_GRAVITY_NONE,GAME_GO_HOME);
}

void S_Shutdown_loop () {
	uint8_t position_status = query_rocket_progress();
	sprintf(state_array[state_now].display_1,"Progress=%4d",position_status);
	display_state();
	if (100 == position_status) {
		// we are done moving
		goto_state("S_Shutdown_Done");
	}
}


/*
 * state_callback: do explicit calls and not via function pointers to avoid Cloud9 debugger crashes
 *
 */
 
void state_callback(char *call_name) {
	if      (ACTION_NOP == call_name)
		return;

	else if (0 == strcmp("S_Main_Play_enter",call_name))
		S_Main_Play_enter();

	else if (0 == strcmp("S_Start_Home_enter",call_name))
		S_Start_Home_enter();
	else if (0 == strcmp("S_Calibrate_enter",call_name))
		S_Calibrate_enter();
	else if (0 == strcmp("S_Calibrate_loop",call_name))
		S_Calibrate_loop();
	else if (0 == strcmp("S_Calibrate_Done_enter",call_name))
		S_Calibrate_Done_enter();
	else if (0 == strcmp("S_Calibrate_Status_enter",call_name))
		S_Calibrate_Status_enter();
		
	else if (0 == strcmp("S_Test_Motor_Status_loop",call_name))
		S_Test_Motor_Status_loop();

	else if (0 == strcmp("S_Game_Start_enter",call_name))
		S_Game_Start_enter();
	else if (0 == strcmp("S_Game_Start_loop",call_name))
		S_Game_Start_loop();		
	else if (0 == strcmp("S_Game_Play_loop",call_name))
		S_Game_Play_loop();
	else if (0 == strcmp("S_Game_Done_enter",call_name))
		S_Game_Done_enter();
	else if (0 == strcmp("S_Game_Display_Next_enter",call_name))
		S_Game_Display_Next_enter();

	else if (0 == strcmp("S_Opt_Game_Z_Enter",call_name))
		S_Opt_Game_Z_Enter();
	else if (0 == strcmp("S_Opt_Game_XYZ_Enter",call_name))
		S_Opt_Game_XYZ_Enter();
	else if (0 == strcmp("S_Opt_Game_Flight_Enter",call_name))
		S_Opt_Game_Flight_Enter();
	else if (0 == strcmp("S_Opt_Game_Move_Enter",call_name))
		S_Opt_Game_Move_Enter();
	else if (0 == strcmp("S_Opt_Game_Auto_Enter",call_name))
		S_Opt_Game_Auto_Enter();

	else if (0 == strcmp("S_Opt_Gravity_Full_Enter",call_name))
		S_Opt_Gravity_Full_Enter();
	else if (0 == strcmp("S_Opt_Gravity_High_Enter",call_name))
		S_Opt_Gravity_High_Enter();
	else if (0 == strcmp("S_Opt_Gravity_None_Enter",call_name))
		S_Opt_Gravity_None_Enter();
	else if (0 == strcmp("S_Opt_Gravity_Negative_Enter",call_name))
		S_Opt_Gravity_Negative_Enter();

	else if (0 == strcmp("S_Opt_Fuel_Normal_Enter",call_name))
		S_Opt_Fuel_Normal_Enter();
	else if (0 == strcmp("S_Opt_Fuel_Low_Enter",call_name))
		S_Opt_Fuel_Low_Enter();
	else if (0 == strcmp("S_Opt_Fuel_Nolimit_Enter",call_name))
		S_Opt_Fuel_Nolimit_Enter();

	else if (0 == strcmp("S_Opt_Pos_Center_Enter",call_name))
		S_Opt_Pos_Center_Enter();
	else if (0 == strcmp("S_Opt_Pos_Random_Enter",call_name))
		S_Opt_Pos_Random_Enter();

	else if (0 == strcmp("S_Test_I2C_enter",call_name))
		S_Test_I2C_enter();

	else if (0 == strcmp("S_Test_Segment_Open_enter",call_name))
		S_Test_Segment_Open_enter();
	else if (0 == strcmp("S_Test_Segment_enter",call_name))
		S_Test_Segment_enter();

	else if (0 == strcmp("S_IO_STATE_loop",call_name))
		S_IO_STATE_loop();
	else if (0 == strcmp("S_Test_enter",call_name))
		S_Test_enter();

	else if (0 == strcmp("S_Test_Simulation_Meters_enter",call_name))
		S_Test_Simulation_Meters_enter();
	else if (0 == strcmp("S_Test_Simulation_MicroMeters_loop",call_name))
		S_Test_Simulation_MicroMeters_loop();
	else if (0 == strcmp("S_Test_Simulation_MilliMeters_loop",call_name))
		S_Test_Simulation_MilliMeters_loop();
	else if (0 == strcmp("S_Test_Simulation_Cables_enter",call_name))
		S_Test_Simulation_Cables_enter();
	else if (0 == strcmp("S_Test_Simulation_Cables_loop",call_name))
		S_Test_Simulation_Cables_loop();
	else if (0 == strcmp("S_Test_Simulation_Steps_enter",call_name))
		S_Test_Simulation_Steps_enter();
	else if (0 == strcmp("S_Test_Simulation_Steps_loop",call_name))
		S_Test_Simulation_Steps_loop();
	else if (0 == strcmp("do_Simulation_Pause_enter",call_name))
		do_Simulation_Pause_enter();
	else if (0 == strcmp("do_Simulation_Resume_enter",call_name))
		do_Simulation_Resume_enter();

	else if (0 == strcmp("S_Test_Sanity_enter",call_name))
		S_Test_Sanity_enter();

	else if (0 == strcmp("S_Test_Antennae_Select_enter",call_name))
		S_Test_Antennae_Select_enter();
	else if (0 == strcmp("S_Test_Antennae_enter",call_name))
		S_Test_Antennae_enter();
	else if (0 == strcmp("S_Test_Antennae_loop",call_name))
		S_Test_Antennae_loop();
	else if (0 == strcmp("S_Test_Antennae_exit",call_name))
		S_Test_Antennae_exit();
	else if (0 == strcmp("S_Test_Antennae_Next_enter",call_name))
		S_Test_Antennae_Next_enter();

	else if (0 == strcmp("S_Test_LedRgb_enter",call_name))
		S_Test_LedRgb_enter();
	else if (0 == strcmp("S_Test_LedRgb_loop",call_name))
		S_Test_LedRgb_loop();
	else if (0 == strcmp("S_Test_LedRgb_exit",call_name))
		S_Test_LedRgb_exit();

	else if (0 == strcmp("S_TestMotor_NextSet_enter",call_name))
		S_TestMotor_NextSet_enter();
	else if (0 == strcmp("S_TestMotor_NextSet_Done_enter",call_name))
		S_TestMotor_NextSet_Done_enter();
	else if (0 == strcmp("S_TestMotor_PlusStep_enter",call_name))
		S_TestMotor_PlusStep_enter();
	else if (0 == strcmp("S_TestMotor_MinusStep_enter",call_name))
		S_TestMotor_MinusStep_enter();
	else if (0 == strcmp("S_TestMotor_Plus360_enter",call_name))
		S_TestMotor_Plus360_enter();
	else if (0 == strcmp("S_TestMotor_Minus360_enter",call_name))
		S_TestMotor_Minus360_enter();

	else if (0 == strcmp("S_Test_Sound_Select_enter",call_name))
		S_Test_Sound_Select_enter();
	else if (0 == strcmp("S_Test_Sound_enter",call_name))
		S_Test_Sound_enter();
	else if (0 == strcmp("S_Test_Sound_loop",call_name))
		S_Test_Sound_loop();
	else if (0 == strcmp("S_Test_Sound_exit",call_name))
		S_Test_Sound_exit();
	else if (0 == strcmp("S_Test_Sound_Next_enter",call_name))
		S_Test_Sound_Next_enter();

	else if (0 == strcmp("S_Shutdown_enter",call_name))
		S_Shutdown_enter();
	else if (0 == strcmp("S_Shutdown_loop",call_name))
		S_Shutdown_loop();

	else log_val("ERROR: MISSING_CALLBACK=%s\n",call_name);

}

/*
 * init_state - instantiate the state table
 *
 */

void init_state () {

	if (!self_test) {
		state_now=0;
		state_prev=0;
	}

//	 "1234567890123456",

// Initial screen

	StateGuiAdd("S_Init",
	 STATE_FROM_CALLBACK,
	 "  Rocket Lander!",
//	 "1234567890123456",
	 "I/O_Test   Start",
	 "S_IO_STATE","S_Start",
	 "S_Main_Play_enter",ACTION_NOP,ACTION_NOP);

// Top Menus

	StateGuiAdd("S_Start",
	 STATE_NO_FLAGS,
	 "Rocket Position?",
//	 "1234567890123456",
	 "Home   Calibrate",
	 "S_Start_Home","S_Calibrate",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Start_Home",
		 STATE_NO_VERBOSE,
		 "",
		 "",
		 STATE_NOP,STATE_NOP,
		 "S_Start_Home_enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Calibrate",
		 STATE_NO_VERBOSE,
		 "Rocket Calibrate",
	//	 "1234567890123456",
		 "Done      Status",
		 "S_Calibrate_Done","S_Calibrate_Status",
		 "S_Calibrate_enter","S_Calibrate_loop",ACTION_NOP);

			StateGuiAdd("S_Calibrate_Done",
			 STATE_NO_FLAGS,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Calibrate_Done_enter",ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Calibrate_Status",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Calibrate_Status_enter",ACTION_NOP,ACTION_NOP);

	StateGuiAdd("S_Main_Play",
	 STATE_NO_FLAGS,
	 "  Rocket Lander!",
//	 "1234567890123456",
	 " Play!      Next",
	 "S_Game_Start","S_Main_Options",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Game_Start",
		 STATE_NO_VERBOSE,
		 "",
		 "          Cancel",
		 STATE_NOP,"S_Game_Done",
		 "S_Game_Start_enter","S_Game_Start_loop",ACTION_NOP);

		StateGuiAdd("S_Game_Position",
		 STATE_NO_FLAGS,
		 "",
	//	 "1234567890123456",
		 "          Cancel",
		 STATE_NOP,"S_Game_Done",
		 "S_Shutdown_enter","S_Shutdown_loop",ACTION_NOP);

		StateGuiAdd("S_Game_Play",
		 STATE_NO_VERBOSE,
		 "",
		 "",
		 "S_Main_Play","S_Game_Display_Next",
		 STATE_NOP,"S_Game_Play_loop",ACTION_NOP);

			StateGuiAdd("S_Game_Display_Next",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Game_Display_Next_enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Game_Done",
		 STATE_FROM_CALLBACK,
		 "",
		 " Replay   Init",
		 "S_Game_Start","S_Main_Play",
		 "S_Game_Done_enter",ACTION_NOP,ACTION_NOP);

	StateGuiAdd("S_Main_Options",
	 STATE_NO_FLAGS,
	 "  Rocket Lander!",
//	 "1234567890123456",
	 " Options    Next",
	 "S_Options","S_Main_Test",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

	StateGuiAdd("S_Main_Test",
	 STATE_NO_FLAGS,
	 "  Rocket Lander!",
//	 "1234567890123456",
	 "Test        Next",
	 "S_Test","S_Shutdown",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

	StateGuiAdd("S_Shutdown",
	 STATE_NO_FLAGS,
	 "  Rocket Lander!",
//	 "1234567890123456",
	 "Shutdown    Next",
	 "S_Shutdown_Select","S_Main_Play",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Shutdown_Select",
		 STATE_NO_FLAGS,
		 "",
	//	 "1234567890123456",
		 "          Cancel",
		 STATE_NOP,"S_Shutdown_Done",
		 "S_Shutdown_enter","S_Shutdown_loop",ACTION_NOP);

		StateGuiAdd("S_Shutdown_Done",
		 STATE_FROM_CALLBACK,
		 "SAFE TO TURN OFF",
	//	 "1234567890123456",
		 "Return to Main",
		 "S_Main_Play","S_Main_Play",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);


// TOP: Options

	StateGuiAdd("S_Options",
	 STATE_NO_FLAGS,
	 "Select ...",
//	 "1234567890123456",
	 " Game      Next",
	 "S_Opt_Game_Z","S_Opt_Gravity",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Game_Z",
		 STATE_NO_FLAGS,
		 "Game   ...",
	//	 "1234567890123456",
		 "Land:Z      Next",
		 "S_Opt_Game_Z_Select","S_Opt_Game_XYZ",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Game_Z_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Game_Z_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Game_XYZ",
		 STATE_NO_FLAGS,
		 "Game   ...",
	//	 "1234567890123456",
		 "Land:XYZ    Next",
		 "S_Opt_Game_XYZ_Select","S_Opt_Game_Flight",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Game_XYZ_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Game_XYZ_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Game_Flight",
		 STATE_NO_FLAGS,
		 "Game   ...",
	//	 "1234567890123456",
		 "Flight:XYZ  Next",
		 "S_Opt_Game_Flight_Select","S_Opt_Game_Move",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Game_Flight_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Game_Flight_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Game_Move",
		 STATE_NO_FLAGS,
		 "Game   ...",
	//	 "1234567890123456",
		 "Move:XYZ    Next",
		 "S_Opt_Game_Move_Select","S_Opt_Game_Auto",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Game_Move_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Game_Move_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Game_Auto",
		 STATE_NO_FLAGS,
		 "Game   ...",
	//	 "1234567890123456",
		 "Autopilot   Next",
		 "S_Opt_Game_Auto_Select","S_Opt_Game_Back",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Game_Auto_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Game_Auto_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Game_Back",
		 STATE_NO_FLAGS,
		 "Game   ...",
	//	 "1234567890123456",
		 "Back        Next",
		 "S_Main_Play","S_Opt_Game_Z",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

// Opt: Gravity

	StateGuiAdd("S_Opt_Gravity",
	 STATE_NO_FLAGS,
	 "Select ...",
//	 "1234567890123456",
	 "Gravity     Next",
	 "S_Opt_Gravity_Full","S_Opt_Fuel",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Gravity_Full",
		 STATE_NO_FLAGS,
		 "Gravity...",
	//	 "1234567890123456",
		 "Normal      Next",
		 "S_Opt_Gravity_Full_Select","S_Opt_Gravity_High",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Gravity_Full_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Gravity_Full_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Gravity_High",
		 STATE_NO_FLAGS,
		 "Gravity...",
	//	 "1234567890123456",
		 "High        Next",
		 "S_Opt_Gravity_High_Select","S_Opt_Gravity_None",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Gravity_High_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Gravity_High_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Gravity_None",
		 STATE_NO_FLAGS,
		 "Gravity...",
	//	 "1234567890123456",
		 "Half        Next",
		 "S_Opt_Gravity_None_Select","S_Opt_Gravity_Back",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Gravity_None_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Gravity_None_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Gravity_Back",
		 STATE_NO_FLAGS,
		 "Gravity...",
	//	 "1234567890123456",
		 "Back        Next",
		 "S_Main_Play","S_Opt_Gravity_Full",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

// Opt: Fuel

	StateGuiAdd("S_Opt_Fuel",
	 STATE_NO_FLAGS,
	 "Select ...",
//	 "1234567890123456",
	 "Fuel       Next",
	 "S_Opt_Fuel_Normal","S_Opt_Pos",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Fuel_Normal",
		 STATE_NO_FLAGS,
		 "Init Position...",
	//	 "1234567890123456",
		 "Normal      Next",
		 "S_Opt_Fuel_Normal_Select","S_Opt_Fuel_Low",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Fuel_Normal_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Fuel_Normal_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Fuel_Low",
		 STATE_NO_FLAGS,
		 "Init Position...",
	//	 "1234567890123456",
		 "Normal      Next",
		 "S_Opt_Fuel_Low_Select","S_Opt_Fuel_Nolimit",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Fuel_Low_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Fuel_Low_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Fuel_Nolimit",
		 STATE_NO_FLAGS,
		 "Init Position...",
	//	 "1234567890123456",
		 "Normal      Next",
		 "S_Opt_Fuel_Nolimit_Select","S_Opt_Fuel_Back",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Fuel_Nolimit_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Fuel_Nolimit_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Fuel_Back",
		 STATE_NO_FLAGS,
		 "Init Position...",
	//	 "1234567890123456",
		 "Back        Next",
		 "S_Main_Play","S_Opt_Fuel_Normal",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);


// Opt: Start Position

	StateGuiAdd("S_Opt_Pos",
	 STATE_NO_FLAGS,
	 "Select ...",
//	 "1234567890123456",
	 "Start_Pos  Next",
	 "S_Opt_Pos_Center","S_Opt_Back",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Pos_Center",
		 STATE_NO_FLAGS,
		 "Init Position...",
	//	 "1234567890123456",
		 "Center      Next",
		 "S_Opt_Pos_Center_Select","S_Opt_Pos_Random",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Pos_Center_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Pos_Center_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Pos_Random",
		 STATE_NO_FLAGS,
		 "Init Position...",
	//	 "1234567890123456",
		 "Random      Next",
		 "S_Opt_Pos_Random_Select","S_Opt_Pos_Back",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_Opt_Pos_Random_Select",
			 STATE_NO_VERBOSE,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_Opt_Pos_Random_Enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Opt_Pos_Back",
		 STATE_NO_FLAGS,
		 "Init Position...",
	//	 "1234567890123456",
		 "Back        Next",
		 "S_Main_Play","S_Opt_Pos_Center",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);


	StateGuiAdd("S_Opt_Back",
	 STATE_NO_FLAGS,
	 "Select ...",
//	 "1234567890123456",
	 "Back       Next",
	 "S_Main_Play","S_Options",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);


// TOP: Test

	StateGuiAdd("S_Test",
	 STATE_NO_FLAGS,
	 "Setup...        ",
//	 "1234567890123456",
	 "I/O_Values  Next",
	 "S_IO_STATE","S_Test_I2cSlaveTest",
	 "S_Test_enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_IO_STATE",
		 STATE_NO_VERBOSE,
		 "I/O State       ",
		 "  Display...    ",
		 STATE_NOP,STATE_NOP,
		 ACTION_NOP,"S_IO_STATE_loop",ACTION_NOP);

	StateGuiAdd("S_Test_I2cSlaveTest",
	 STATE_NO_FLAGS,
	 "Test...         ",
//	 "1234567890123456",
	 "I2C_test Next",
	 "S_Test_I2C_Select","S_Test_Segment_Test",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_I2C_Select",
		 STATE_NO_FLAGS,
		 "Test...         ",
	//	 "1234567890123456",
		 "Exit        Send",
		 "S_Main_Play","S_Test_I2C_Send",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_I2C_Send",
		 STATE_NO_FLAGS,
		 "",
		 "",
		 STATE_NOP,STATE_NOP,
		 "S_Test_I2C_enter",ACTION_NOP,ACTION_NOP);

	StateGuiAdd("S_Test_Segment_Test",
	 STATE_NO_FLAGS,
	 "Test...         ",
//	 "1234567890123456",
	 "SegmentTest Next",
	 "S_Test_Segment_Init","S_Test_Motor_Test",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_Segment_Init",
		 STATE_NO_FLAGS,
		 "",
		 "",
		 STATE_NOP,STATE_NOP,
		 "S_Test_Segment_Open_enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_Segment_Select",
		 STATE_FROM_CALLBACK,
		 "Test...         ",
	//	 "1234567890123456",
		 "Exit        Send",
		 "S_Main_Play","S_Test_Segment_Send",
		ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_Segment_Send",
		 STATE_NO_FLAGS,
		 "",
		 "",
		 STATE_NOP,STATE_NOP,
		 "S_Test_Segment_enter",ACTION_NOP,ACTION_NOP);

	StateGuiAdd("S_Test_Motor_Test",
	 STATE_NO_FLAGS,
	 "Test...         ",
//	 "1234567890123456",
	 "Motor_Test  Next",
	 "S_TestMotor_NextSet","S_Test_SanityTest",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_TestMotor_NextSet",
		 STATE_NO_FLAGS,
		 "Test Motor  +set",
	//	 "1234567890123456",
		 "Next        +set",
		 "S_TestMotor_NextSet_Done","S_TestMotor_NextSet_Go",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_TestMotor_NextSet_Go",
			 STATE_NO_FLAGS,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_TestMotor_NextSet_enter",ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_TestMotor_NextSet_Done",
			 STATE_NO_FLAGS,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_TestMotor_NextSet_Done_enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_TestMotor_PlusStep",
		 STATE_FROM_CALLBACK,
		 "Test Motor +step",
	//	 "1234567890123456",
		 "Next       +step",
		 "S_TestMotor_MinusStep","S_TestMotor_PlusStep_Go",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_TestMotor_PlusStep_Go",
			 STATE_NO_FLAGS,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_TestMotor_PlusStep_enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_TestMotor_MinusStep",
		 STATE_NO_FLAGS,
		 "Test Motor  -step",
	//	 "1234567890123456",
		 "Next        -step",
		 "S_TestMotor_Plus360","S_TestMotor_MinusStep_Go",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_TestMotor_MinusStep_Go",
			 STATE_NO_FLAGS,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_TestMotor_MinusStep_enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_TestMotor_Plus360",
		 STATE_NO_FLAGS,
		 "Test Motor   +360",
	//	 "1234567890123456",
		 "Next         +360",
		 "S_TestMotor_Minus360","S_TestMotor_Plus360_Go",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_TestMotor_Plus360_Go",
			 STATE_NO_FLAGS,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_TestMotor_Plus360_enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_TestMotor_Minus360",
		 STATE_NO_FLAGS,
		 "Test Motor   -360",
	//	 "1234567890123456",
		 "Next         -360",
		 "S_Main_Play","S_TestMotor_Minus360_Go",
		 ACTION_NOP,ACTION_NOP,ACTION_NOP);

			StateGuiAdd("S_TestMotor_Minus360_Go",
			 STATE_NO_FLAGS,
			 "",
			 "",
			 STATE_NOP,STATE_NOP,
			 "S_TestMotor_Minus360_enter",ACTION_NOP,ACTION_NOP);

	StateGuiAdd("S_Test_SanityTest",
	 STATE_NO_FLAGS,
	 "Test...         ",
//	 "1234567890123456",
	 "Sanity_test Next",
	 "S_Test_Sanity_Select","S_Test_Simulation",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_Sanity_Select",
		 STATE_NO_FLAGS,
		 "",
		 "",
		 STATE_NOP,STATE_NOP,
		 "S_Test_Sanity_enter",ACTION_NOP,ACTION_NOP);

	StateGuiAdd("S_Test_Simulation",
	 STATE_NO_FLAGS,
	 "Test...         ",
//	 "1234567890123456",
	 "Simulation  Next",
	 "S_Test_Simulation_MicroMeters_Select","S_Test_Calibrate",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_Simulation_MicroMeters_Select",
		 STATE_NO_FLAGS,
		 "Sim: Pos uMeters",
	//	 "1234567890123456",
		 "Pause       Next",
		 "S_Test_Simulation_Pause","S_Test_Simulation_MilliMeters_Select",
		 "S_Test_Simulation_Meters_enter","S_Test_Simulation_MicroMeters_loop",ACTION_NOP);

		StateGuiAdd("S_Test_Simulation_MilliMeters_Select",
		 STATE_NO_FLAGS,
		 "Sim: Pos mMeters",
	//	 "1234567890123456",
		 "Pause       Next",
		 "S_Test_Simulation_Pause","S_Test_Simulation_Cables_Select",
		 "S_Test_Simulation_Meters_enter","S_Test_Simulation_MilliMeters_loop",ACTION_NOP);

		StateGuiAdd("S_Test_Simulation_Cables_Select",
		 STATE_NO_FLAGS,
		 "Sim: Cables mM",
	//	 "1234567890123456",
		 "Pause       Next",
		 "S_Test_Simulation_Pause","S_Test_Simulation_Steps_Select",
		 "S_Test_Simulation_Cables_enter","S_Test_Simulation_Cables_loop",ACTION_NOP);

		StateGuiAdd("S_Test_Simulation_Steps_Select",
		 STATE_NO_FLAGS,
		 "Sim: Cable steps",
	//	 "1234567890123456",
		 "Pause       Next",
		 "S_Test_Simulation_Pause","S_Test_Simulation_MicroMeters_Select",
		 "S_Test_Simulation_Steps_enter","S_Test_Simulation_Steps_loop",ACTION_NOP);

		StateGuiAdd("S_Test_Simulation_Pause",
		 STATE_NO_VERBOSE,
		 "Sim:    Pause...",
	//	 "1234567890123456",
		 "Resume      Exit",
		 "S_Test_Simulation_Resume","S_Main_Play",
		 "do_Simulation_Pause_enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_Simulation_Resume",
		 STATE_NO_VERBOSE,
		 "",
		 "",
		 STATE_NOP,STATE_NOP,
		 "do_Simulation_Resume_enter",ACTION_NOP,ACTION_NOP);

	StateGuiAdd("S_Test_Calibrate",
	 STATE_NO_FLAGS,
	 "Setup...",
//	 "1234567890123456",
	 "Calibrate   Next",
	 "S_Calibrate","S_Test_Motor_Status",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

	StateGuiAdd("S_Test_Motor_Status",
	 STATE_NO_FLAGS,
	 "Setup...",
//	 "1234567890123456",
	 "Motor_State Next",
	 "S_Test_Motor_Status_Select","S_Test_Antennae_test",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_Motor_Status_Select",
		 STATE_NO_VERBOSE,
		 "Motors at:  100%",
	//	 "1234567890123456",
		 "Done",
		 "S_Main_Play","S_Main_Play",
		 ACTION_NOP,"S_Test_Motor_Status_loop",ACTION_NOP);
	
	StateGuiAdd("S_Test_Antennae_test",
	 STATE_NO_FLAGS,
	 "Test...",
//	 "1234567890123456",
	 "Antennae    Next",
	 "S_Test_Antennae_Select","S_Test_Ledrgb_test",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_Antennae_Select",
		 STATE_NO_FLAGS,
		 "",
		 "",
		 STATE_NOP,STATE_NOP,
		 "S_Test_Antennae_Select_enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_Antennae",
		 STATE_FROM_CALLBACK,
	 	 "Test Antennae",
	//	 "1234567890123456",
		 "Exit   Next_Axis",
		 "S_Test","S_Test_Antennae_Next",
		 "S_Test_Antennae_enter","S_Test_Antennae_loop","S_Test_Antennae_exit");

		StateGuiAdd("S_Test_Antennae_Next",
		 STATE_NO_FLAGS,
		 "",
		 "",
		 STATE_NOP,STATE_NOP,
		 "S_Test_Antennae_Next_enter",ACTION_NOP,ACTION_NOP);

	StateGuiAdd("S_Test_Ledrgb_test",
	 STATE_NO_FLAGS,
	 "Test...",
//	 "1234567890123456",
	 "LED_RGB     Next",
	 "S_Test_LedRgb_Select","S_Test_Sound_test",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_LedRgb_Select",
		 STATE_NO_FLAGS,
	 	 "Test LedRgb",
	//	 "1234567890123456",
		 "Exit",
		 "S_Test","S_Test",
		 "S_Test_LedRgb_enter","S_Test_LedRgb_loop","S_Test_LedRgb_exit");

	StateGuiAdd("S_Test_Sound_test",
	 STATE_NO_FLAGS,
	 "Test...",
//	 "1234567890123456",
	 "Sound/Neo   Next",
	 "S_Test_Sound_Select","S_Test_Back",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_Sound_Select",
		 STATE_NO_FLAGS,
		 "",
		 "",
		 STATE_NOP,STATE_NOP,
		 "S_Test_Sound_Select_enter",ACTION_NOP,ACTION_NOP);

		StateGuiAdd("S_Test_Sound",
		 STATE_FROM_CALLBACK,
	 	 "Test Sound&Neo  ",
	//	 "1234567890123456",
		 "Exit  Next_Motor",
		 "S_Test","S_Test_Sound_Next",
		 "S_Test_Sound_enter","S_Test_Sound_loop","S_Test_Sound_exit");

		StateGuiAdd("S_Test_Sound_Next",
		 STATE_NO_FLAGS,
		 "",
		 "",
		 STATE_NOP,STATE_NOP,
		 "S_Test_Sound_Next_enter",ACTION_NOP,ACTION_NOP);

	StateGuiAdd("S_Test_Back",
	 STATE_NO_FLAGS,
	 "Setup...        ",
//	 "1234567890123456",
	 " Back       Next",
	 "S_Main_Play","S_Test",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);



// To-do Catch all state

	StateGuiAdd("S_Todo",
	 STATE_NO_FLAGS,
	 "  <<< TO DO >>>",
//	 "1234567890123456",
	 "Back            ",
	 "S_Main_Play","S_Main_Play",
	 ACTION_NOP,ACTION_NOP,ACTION_NOP);

// Self test: state orphaned with bad-links

	StateGuiAdd("S_Orphan_Error",
	 STATE_NO_FLAGS,
	 "",
	 "",
	 "S_Orphan_Error_K1","S_Orphan_Error_K2",
	 "S_Orphan_Error_Enter","S_Orphan_Error_Loop","S_Orphan_Error_Exit");

}


/*
 * state_loop - called from main loop
 *
 */

void state_loop() {
	/* Process Buttons (default mode is toggle) */
	if (r_control.button_a && r_control.button_b) {
		if (state_now == find_state("S_IO_STATE"))
			goto_state("S_Init");
		else
			goto_state("S_Main_Play");
	} else if (r_control.button_a && !r_control.button_a_prev) {
		goto_state(state_array[state_now].k1);
	} else if (r_control.button_b && !r_control.button_b_prev) {
		goto_state(state_array[state_now].k2);
	}
	if (0x0000 == (state_array[state_now].state_flags & STATE_BUTTON_HOLD_A)) {
		r_control.button_a_prev = r_control.button_a;
	}
	if (0x0000 == (state_array[state_now].state_flags & STATE_BUTTON_HOLD_B)) {
		r_control.button_b_prev = r_control.button_b;
	}

	// execute any state loop function
	state_callback(state_array[state_now].state_loop);
}

