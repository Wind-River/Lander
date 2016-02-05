/* main.c - Rocket Lander Game */

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

/*
 * Microkernel demo of a Rocket Lander Game
 *
 *  Features: I2C by displaying information on a Grove-LCD
 *            Digital I/O on a Grove-Buttons
 *            Analog I/O on a Grove-Joystick
 *            X-Y-Z Stepper Motor Control on XXX
 *            Pan and Tilt Servos
 *            I2C control on a Grove-LED_RGB
 *
 * The green LED on the Galileo2 board next to the USB connector flashes
 * continuously to show a heartbeat.
 *
 */


/*
	*	E:\temp\MyNewRocket-virtual-gateway
Todo:
	x	Enable buttons, test state transitions
	x	Enable Joystick, basic game play
	x	Finish Z,XYZ game play
	--- Demo 1 checkpoint
	x	Re-parameterize code
		x	calibrate m/s to game units
		x	calibrate change/s to loop units
		x	option states
	x	Rocket position to motor parameters
		x	compute rocket position to cable lengths
		x	compute rocket position to cable stepping!
	x	Create 'Move' game for movement testing
		x	no inertia/gravity (e./g. gantry motion model)
		x	fast x/y/z
==>	--- Demo 2 checkpoint
	*	Game display modes
		*	x,y,z
		*	x,y,z,speed,fuel
		*	cables
	*	Large and small state LEDs
	*	Communication with daughter Arduino-101
		*	Serial server
		*	Serial client
	*	import thrust method from sample game
	*	Implement power-up setup and calibration states
	--- Demo 3 checkpoint
	*	Arduino-101 with slider and I2C motor control
	*	Integration with X-Y-Z motors
	--- Demo 4 checkpoint
	*	Enable Tracker Pan-Tilt
		*	cos table
		*	PWM
	*	Enable Tracker LED-RGB
		*	i2c
	*	Game Auto-pilot (for self-play, come hither mode)
		*	import auto-pilot method from sample game
	--- Demo 5 checkpoint
	*	Enable Sound
	*	Enable Cloud

*/

/*
 * Hardware Enablement, Global Environment
 *
 */

#include <zephyr.h>

#include <i2c.h>
#include "groveLCD.h"
#include "hd44780Lcd.h"
#include "groveLCDUtils.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include <gpio.h>
#include <pwm.h>
#include <adc.h>
#include "galileo2ADCPWM.h"

#include "rocket.h"
#include "rocket_space.h"
#include "rocket_state.h"

/*
 * Game Variables
 *
 */

struct ROCKET_GAME_S r_game;
struct ROCKET_CONTROL_S r_control;
bool verbose = true;
bool self_test = false;


/*
 * common and debugging routines
 *
 */

void log(char *message) {
	if (verbose) {
		PRINT(message);
	}
}

void log_val(char *format, void *val) {
	char buffer[1000];
	sprintf(buffer,format,val);
	log(buffer);
}

int32_t abs(int32_t val) {
	if (0 < val)
		return val;
	else
		return -val;
}

/*
 * Hardware definitions
 *
 */

/* specify which Arduino connector pin is used as input, output and LED */
#define INPUT_A_PIN   3
#define INPUT_B_PIN   5
#define GREEN_LED    13  // D13 is the on-board LED
#define JOYSTICK_A_PIN  0 /* A0 */
#define JOYSTICK_B_PIN  1 /* A1 */

struct device *i2c;

/* LCD Data */
#define LCD_MESSAGE1 " Rocket Lander  "
#define LCD_MESSAGE2 "    Hello!      "


/* ADC Definitions */

/* Buffer holds ADC conversion result for A0 .. A2 */
static uint16_t seq_buffer[ADC_MAX] = { 0 , 0 , 0};
struct adc_seq_entry sample[] = {
    {
        .sampling_delay = 1,
        .channel_id     = JOYSTICK_X_PORT,
        .buffer         = (uint8_t*) &seq_buffer[0],
        .buffer_length  = 1,
    },
    {
        .sampling_delay = 1,
        .channel_id     = JOYSTICK_Y_PORT,
        .buffer         = (uint8_t*) &seq_buffer[1],
        .buffer_length  = 1,
    },
	{
        .sampling_delay = 1,
        .channel_id     = JOYSTICK_Z_PORT,
        .buffer         = (uint8_t*) &seq_buffer[2],
        .buffer_length  = 1,
    },
};

struct adc_seq_table table =
{
    .entries     = sample,
    .num_entries = 3,
};

void adcCallback
    (
    struct device *dev,
    enum adc_callback_type cb_type
    );

/*
 * init_hardware
 *
 */

void init_hardware()
	{

	/* Init the I/O pins */
	if (IO_JOYSTICK_ENABLE) {
		adc_set_callback(adc, adcCallback);
	}

	/* Init the LCD RGB */
	if (IO_LCD_ENABLE) {
		groveLcdInit(i2c);

		groveLcdCommand(i2c, LCD_CLEAR);
		groveLcdColorSet(i2c, 0, 100, 200);
		groveLcdPrint(i2c, 0, 0, LCD_MESSAGE1, strlen(LCD_MESSAGE1));
		groveLcdPrint(i2c, 1, 0, "                   ",19);
	}

	/* Init the X-Y-Z table */
	init_rocket_hardware();

	/* Init the Tracker */

	/* Init the Phone/Server */


 }

/*
 * scan_controls : read the control inputs
 *
 */

void scan_controls ()
 {
    int32_t rc;

	if (IO_BUTTONS_ENABLE) {
		r_control.button_a = gpioInputGet(INPUT_A_PIN);
		r_control.button_b = gpioInputGet(INPUT_B_PIN);
	}

	if (IO_JOYSTICK_ENABLE) {
		/* the example code for some reason re-asserts the channel numbers?? */
        rc = adc_read(adc, &table);
        if (DEV_OK != rc)
            {
            PRINT("ADC read error! (%d)\n", rc);
            }
        else
            {
            /* PRINT("wait for callback\n"); */
            task_event_recv_wait(ADCREADY);
			// TODO #################### How much time is spent waiting?
            }
        r_control.analog_x = seq_buffer[0];
        r_control.analog_y = seq_buffer[1];
        r_control.analog_z = seq_buffer[2];

		if (IO_ADAFRUIT_JOYSTICK_ENABLE) {
			// TODO ####################
		}
 	}

 }

/*
 * init_game : initialize variables for a new game
 *
 */

void init_game () {
	int32_t	init_x;
	int32_t	init_y;
	char buffer[1024];

	// display the setup game state
	if        (r_game.game == GAME_Z_LAND) {
		sprintf(buffer," Game: Land Z!");
		set_lcd_display(0,buffer);
	} else if (r_game.game == GAME_XYZ_LAND) {
		sprintf(buffer," Game: Land Z!");
		set_lcd_display(0,buffer);
	} else if (r_game.game == GAME_XYZ_FLIGHT) {
		sprintf(buffer," Game: Land Z!");
		set_lcd_display(0,buffer);
	} else if (r_game.game == GAME_COME_HITHER) {
		sprintf(buffer," Game: Land Z!");
		set_lcd_display(0,buffer);
	} else if (r_game.game == GAME_COME_HITHER) {
		sprintf(buffer," Game: Unknown :-(");
		set_lcd_display(0,buffer);
	}
	set_lcd_display(1," waiting ... " );


	// set initial game controls

	r_control.button_a=0;
	r_control.button_b=0;
	r_control.button_a_prev=false;
    r_control.button_b_prev=false;

	r_control.analog_x=JOYSTICK_X_MID;
	r_control.analog_y=JOYSTICK_Y_MID;
	r_control.analog_z=JOYSTICK_Z_MID;

	if        (r_game.start_option == GAME_START_RANDOM) {
		// TODO ###################
		init_x=0;
		init_y=0;
	} else {
		init_x=0;
		init_y=0;
	}


	if        (r_game.game == GAME_Z_LAND) {
		init_rocket_game(init_x, init_y, Z_POS_MAX, r_game.fuel_option, r_game.gravity_option);
	} else if (r_game.game == GAME_XYZ_LAND) {
		init_rocket_game(init_x, init_y, Z_POS_MAX, r_game.fuel_option, r_game.gravity_option);
	} else if (r_game.game == GAME_XYZ_FLIGHT) {
		init_rocket_game(init_x, init_y,         0, r_game.fuel_option, r_game.gravity_option);
	} else if (r_game.game == GAME_XYZ_AUTO) {
		init_rocket_game(init_x, init_y, Z_POS_MAX, r_game.fuel_option, r_game.gravity_option);
	} else {
		init_rocket_game(init_x, init_y, Z_POS_MAX, r_game.fuel_option, r_game.gravity_option);
	}

	// LCD and debug Display mode
	r_game.play_display_mode=GAME_DISPLAY_RAW;
}

/*
 * init_main - initialize top-level game variables
 *
 */
void init_main()
	{
    // Preset the game defaults
	if (IO_BUTTON_BRINGUP) {
	    r_game.game = GAME_XYZ_MOVE;
	} else {
	    r_game.game = GAME_XYZ_LAND;
	}
    r_game.fuel_option = GAME_FUEL_NORMAL;
    r_game.gravity_option = GAME_GRAVITY_NONE;
    r_game.start_option = GAME_START_CENTER;
    r_game.play_display_mode = GAME_DISPLAY_RAW;

	}

/*
 * main - LCD demo
 *
 */
void main()
    {
    bool flag = false;

	init_main();
	init_state();

    if (DEV_OK != setup())
        {
        return;
        }
    task_sleep(100);
    task_sleep(100);

	init_hardware();
    task_sleep(100);

	// Start the initial state
	if (IO_BUTTON_BRINGUP) {
		goto_state("S_Init");
	} else {
		goto_state("S_Main_Play");
	}

    while (1)
        {

        /* Blink the on-board LED */
        if (flag)
            {
            gpioOutputSet(GREEN_LED, 1);
            flag = false;
            }
        else
            {
            gpioOutputSet(GREEN_LED, 0);
            flag = true;
            }

		/* fetch the control states */
		scan_controls();

		/* Process Buttons (default mode is toggle) */
		state_loop();

        /* wait a while, then loop again */

        task_sleep(SLEEPTICKS);
        }
    }

/*
 * @brief Callback, which is invoked when ADC gets new data
 *
 * @param dev ADC device structure
 * @param cb_type can be ADC_CB_DONE or ADC_CB_ERROR
 */
void adcCallback
    (
    struct device *dev,
    enum adc_callback_type cb_type
    )
    {
    if (dev == adc && cb_type == ADC_CB_DONE)
        {
        isr_event_send(ADCREADY);
		}
    }
