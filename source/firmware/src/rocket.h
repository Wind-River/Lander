/* rocket.h - Rocket Lander Game */

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
 
#ifndef boolean
typedef char boolean;
#endif

// General Enablement

#define IO_BUTTON_BRINGUP			true	// init state to display I/O button and slider values

#define IO_BUTTONS_ENABLE			true	// enable the buttons to change state
#define IO_JOYSTICK_ENABLE			true	// enable the joystick for X-Y
#define IO_LCD_ENABLE 				true	// enable the LCD display
#define IO_LED_BACKPACK_ENABLE		true	// enable the Adafruit LED i2c backback device
#define IO_TRACKER_LOCAL_ENABLE		true	// enable the Pan&Tilt 'antenae' device
#define IO_MOTOR_ENABLE				true	// enable the motors

#define IO_REMOTE_ENABLE			false	// enable the sister i2c-slave board
											// NOTE: the connection and remote board must be up else we will hang in "setup.c"
#define IO_LEDS_REMOTE_ENABLE		true	// enable the remote LED space lighting
#define IO_NEO_REMOTE_ENABLE		true	// enable the remote NeoPixels space lighting
#define IO_SOUND_REMOTE_ENABLE		true	// enable the remote sound effects device
#define IO_TRACKER_REMOTE_ENABLE	false	// enable the remote Pan&Tilt 'antenae' device
#define IO_LEDRGB_REMOTE_ENABLE		false	// enable the remote LED_RGB 'antenae status' device

// Debugging
#define DEBUG_TIMING_ENABLE			true	// enable the timing measurements for QOS
#define DEBUG_GAME_AT_START			true	// for game play testing, assume rocket already at start position

// Specific installed hardware
#define IO_GROVE_JOYSTICK_ENABLE true
#define IO_GROVE_JOYSTICK_TOGGLE true
#define IO_ADAFRUIT_JOYSTICK_ENABLE false

// Time Controls

/* specify main loop delay (in ms); compute equivalent in ticks */
/* Assume loop of 1/5 seconds */
#define SLEEPTIME  200
#define SLEEPTICKS (SLEEPTIME * sys_clock_ticks_per_sec / 1000)
#define TIME_LOOPS_PER_SECOND	5

/* Specific loop count timeouts per action */
#define XYZ_CONTROL_COUNT		1
#define TRACK_CONTROL_COUNT		4
#define POSITION_CONTROL_COUNT	1

// Remote control boards

#define ROCKET_DISPLAY_I2C_ADDRESS 18
#define ROCKET_MOTOR_I2C_ADDRESS   19

// Game Options

#define GAME_Z_LAND			1
#define GAME_XYZ_LAND		2
#define GAME_XYZ_FLIGHT		3
#define GAME_XYZ_AUTO		4
#define GAME_XYZ_MOVE		5
#define GAME_COME_HITHER	9

#define GAME_FUEL_NORMAL	1
#define GAME_FUEL_LOW		2
#define GAME_FUEL_NOLIMIT	3

#define GAME_GRAVITY_NORMAL		1
#define GAME_GRAVITY_HIGH		2
#define GAME_GRAVITY_NONE		3
#define GAME_GRAVITY_NEGATIVE	4

#define GAME_START_CENTER	1
#define GAME_START_RANDOM	2

#define GAME_DISPLAY_NORMAL	   0
#define GAME_DISPLAY_RAW_XYZF  1
#define GAME_DISPLAY_RAW_CABLE 2
#define GAME_DISPLAY_RAW_STEPS 3

/* specify which Arduino connector pin is used as input, output and LED */
#define INPUT_A_PIN   3
#define INPUT_B_PIN   7	// 5
#define GREEN_LED    13  // D13 is the on-board LED
#define JOYSTICK_A_PIN  0 /* A0 */
#define JOYSTICK_B_PIN  1 /* A1 */

/* LCD Data */
#define LCD_DISPLAY_POS_MAX 16

/* Joystick Data */
#ifdef IO_GROVE_JOYSTICK_ENABLE

#define JOYSTICK_X_PORT 0
#define JOYSTICK_Y_PORT 1
#define JOYSTICK_Z_PORT 2

#define JOYSTICK_X_MIN 200
#define JOYSTICK_X_MID 413
#define JOYSTICK_X_MAX 650
#define JOYSTICK_Y_MIN 200
#define JOYSTICK_Y_MID 413
#define JOYSTICK_Y_MAX 650
#define JOYSTICK_Z_MIN 0
#define JOYSTICK_Z_MID 422
#define JOYSTICK_Z_MAX 1023

#define JOYSTICK_DELTA_XY_MIN 40	// no move at center value zone
#define JOYSTICK_DELTA_Z_MIN  40	// no move at center value zone
#endif

/* Sound Setup */
#define SOUND_QUIET  0
#define SOUND_READY  1
#define SOUND_PLAY   2
#define SOUND_DANGER 3
#define SOUND_LAND   4
#define SOUND_CRASH  5
#define SOUND_MAX    6

/* NeoPixel Setup */
#define NEOPIXEL_QUIET  0
#define NEOPIXEL_READY  1
#define NEOPIXEL_PLAY   2
#define NEOPIXEL_DANGER 3
#define NEOPIXEL_LAND   4
#define NEOPIXEL_CRASH  5
#define NEOPIXEL_MAX    6


/* ADC Setup */
#define ADC_MAX JOYSTICK_Z_PORT+1

/* Pan and Tilt Setup (if local) */
#define PWM_PAN_PORT   	5	// WARNING: IO5 => PWM pin 3: requires setup.c port 5=PINMUX_FUNC_C
#define PWM_TILT_PORT  	6	// WARNING: IO6 => PWM pin 5: requires setup.c port 6=PINMUX_FUNC_C
#define PWM_PAN_PWM		3	// WARNING: IO5 => PWM pin 3: requires setup.c port 5=PINMUX_FUNC_C
#define PWM_TILT_PWM	5	// WARNING: IO6 => PWM pin 5: requires setup.c port 6=PINMUX_FUNC_C
#define PAN_P90		(165/4)
#define PAN_M90		(518/4)
#define PAN_MID		((PAN_P90+PAN_M90)/2)
#define TILT_P90	(215/4)
#define TILT_P00	(401/4)
#define TILT_M45	(497/4)
#define TILT_MID	((TILT_P90+TILT_P00)/2)

/* Game Mode */
#define GAME_PLAY		0x01	// normal game play with motor motion	
#define GAME_SIMULATE	0x22	// simulated play, no motors, already at start
#define GAME_AT_START	0x20	// assume rocket already at start position

// Exported Structures and Funtions

struct ROCKET_GAME_S {
	int32_t	game;				// selected game
	int32_t	fuel_option;		// selected fuel option
	int32_t	gravity_option;		// selected gravity option
	int32_t	start_option;		// selected start option
	int32_t	play_display_mode;	// selected play display format
	int32_t	game_mode;			// selected play or simulate
};

struct ROCKET_CONTROL_S {
	int32_t	button_a;		// button inputs
	int32_t	button_b;

	bool button_a_prev;		// Previous value for toggle detection
    bool button_b_prev;

	int32_t	analog_x;		// analog inputs
	int32_t	analog_y;
	int32_t	analog_z;

	uint8_t	lcd_line0[20];	// LCD display
	uint8_t	lcd_line1[20];

};

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif


extern struct ROCKET_GAME_S r_game;
extern struct ROCKET_CONTROL_S r_control;
extern bool verbose;
extern bool self_test;

extern struct device *i2c;

extern void log(char *message);
extern void log_val(char *format, void *val);
extern int32_t abs(int32_t val);

void send_LED_Backpack(uint32_t x);
void send_rocket_display(uint8_t *buffer,uint8_t i2c_len);
void send_Led1(uint32_t value);
void send_Led2(uint32_t value);
void send_NeoPixel(uint32_t pattern);
void send_Sound(uint32_t pattern);
void send_Pan_Tilt(uint32_t pan,uint32_t tilt);
void send_Led_Rgb(uint32_t r,uint32_t g,uint32_t b);

void init_game();
void init_main();

