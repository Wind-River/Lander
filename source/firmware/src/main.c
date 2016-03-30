/* main.c - Rocket Lander Game */

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
 *  Features: Grove 16x2 LCD (ic2)
 *            Two Digital I/O for Buttons
 *            Analog I/O on a Joystick (X-Y)
 *            Analog I/O on a slider (Z)
 *            2 PWM Control for Pan and Tilt Servos (antennae)
 *            Daughter Arduino controller for 4 tower stepper motor control (Rocket Motor)
 *            Daughter Arduino controller for: (Rocket Display)
 *               Grove-LED_RGB (antennae status)
 *               Adafruit large 4 digit LED display (Fuel)
 *               Grove 4 digit LED displays (speed,height)
 *               Adafruit Neo-Pixel strips (lighting)
 *               Adafruit Sound card (effects)
 *
 * Main manages:
 *   - I/O bringup
 *   - Main loop (at 1/5 second)
 *   - Core I/O inputs (buttons and slider)
 *   - Rocket Display daughter board management
 *
 * The green LED on the Galileo2 board next to the USB connector flashes
 * continuously to show a heartbeat.
 *
 */


#include <zephyr.h>

#include <i2c.h>
#include "groveLCD.h"
#include "hd44780Lcd.h"
#include "groveLCDUtils.h"
#include "Adafruit_LEDBackpack.h"

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

void init_hardware() {

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

 	// Adafruit LED Backpack
	if (IO_LED_BACKPACK_ENABLE) {
		bp_setdevice(i2c);
		bp_begin();
		bp_clear();
		send_LED_Backpack(0);
	}

	/* Init the Tracker */
	if (IO_TRACKER_LOCAL_ENABLE) {
		 send_Pan_Tilt(PAN_MID,TILT_MID);
	}

	/* Init the Phone/Server */
	// TODO ###############

 }

/*
 * scan_controls : read the control inputs
 *
 */

void scan_controls () {
    int32_t rc;

	if (IO_BUTTONS_ENABLE) {
		r_control.button_a = gpioInputGet(INPUT_A_PIN);
		r_control.button_b = gpioInputGet(INPUT_B_PIN);
	}

	if (IO_JOYSTICK_ENABLE) {
        rc = adc_read(adc, &table);
        if (DEV_OK != rc)
            {
            PRINT("ERROR: ADC read error! (%d)\n", rc);
            }
        else
            {
            /* PRINT("ADCREADY...); */
            task_event_recv_wait(ADCREADY);
            /* PRINT("DONE\n); */
			if (DEBUG_TIMING_ENABLE) {
				// TODO #################### How much time is spent waiting?
            }
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
 * LED 7-Segment Backpack display
 *
 */

void send_LED_Backpack(uint32_t x) {
	if (IO_LED_BACKPACK_ENABLE) {
		seg_writeNumber(x);
	}
}


/*
 * sister board control (i2c slave)
 *
 */

void send_rocket_display(uint8_t *buffer,uint8_t i2c_len) {
	if (IO_REMOTE_ENABLE) {
		i2c_polling_write (i2c, buffer, i2c_len, ROCKET_DISPLAY_I2C_ADDRESS);
	}
}

void send_Led1(uint32_t value) {
	static uint32_t value_prev = 9999;

	uint8_t buf[10];
	uint32_t i;

	if (value_prev == value) return;
	value_prev = value;

	if (IO_LEDS_REMOTE_ENABLE) {
		buf[0]='1';
		for (i=4;i>0;i--) {
			buf[i]=value % 10;
			value /= 10;
		}
		send_rocket_display(buf,5);
	}
}

void send_Led2(uint32_t value) {
	static uint32_t value_prev = 9999;
	uint8_t buf[10];
	uint32_t i;

	if (value_prev == value) return;
	value_prev = value;

	if (IO_LEDS_REMOTE_ENABLE) {
		buf[0]='2';
		for (i=4;i>0;i--) {
			buf[i]=value % 10;
			value /= 10;
		}
		send_rocket_display(buf,5);
	}
}

void send_Led_Rgb(uint32_t r,uint32_t g,uint32_t b) {
	static uint32_t r_prev = 9999;
	static uint32_t g_prev = 9999;
	static uint32_t b_prev = 9999;
	uint8_t buf[10];

	if ((r_prev == r) && (g_prev == g) && (b_prev == b)) return;
	r_prev = r;
	g_prev = g;
	b_prev = b;

	if (IO_LEDRGB_REMOTE_ENABLE) {
		buf[0]='l';
		buf[1]=(uint8_t) r;
		buf[2]=(uint8_t) g;
		buf[2]=(uint8_t) b;
		send_rocket_display(buf,3);
	}
}

void send_NeoPixel(uint32_t value) {
	static uint32_t value_prev = 9999;
	uint8_t buf[10];

	if (value_prev == value) return;
	value_prev = value;

	if (IO_NEO_REMOTE_ENABLE) {
		buf[0]='n';
		buf[1]=(uint8_t) value;
		send_rocket_display(buf,2);
	}
}

void send_Sound(uint32_t value) {
	static uint32_t value_prev = 9999;
	uint8_t buf[10];

	if (value_prev == value) return;
	value_prev = value;

	if (IO_SOUND_REMOTE_ENABLE) {
		PRINT("** Sound: %d ***",value);
		buf[0]='s';
		buf[1]=(uint8_t) value;
		send_rocket_display(buf,2);
	}
}

// pan and tilt: 0 .. 255
void send_Pan_Tilt(uint32_t pan,uint32_t tilt) {
	static uint32_t pan_prev = 9999;
	static uint32_t tilt_prev = 9999;

	if ((pan_prev == pan) && (tilt_prev == tilt)) return;
	pan_prev = pan;
	tilt_prev = tilt;

	if (IO_TRACKER_LOCAL_ENABLE) {
		uint32_t percent;
        percent = (100 * pan)/256;
        pwm_pin_set_duty_cycle(pwm, PWM_PAN_PWM, percent);
        percent = (100 * tilt)/256;
        pwm_pin_set_duty_cycle(pwm, PWM_TILT_PWM, percent);
	} else if (IO_TRACKER_REMOTE_ENABLE) {
		uint8_t buf[10];

		buf[0]='p';
		buf[1]=(uint8_t) pan;
		buf[2]=(uint8_t) tilt;
		send_rocket_display(buf,3);
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
		set_lcd_display(LCD_BUFFER_1,buffer);
	} else if (r_game.game == GAME_XYZ_LAND) {
		sprintf(buffer," Game: Land Z!");
		set_lcd_display(LCD_BUFFER_1,buffer);
	} else if (r_game.game == GAME_XYZ_FLIGHT) {
		sprintf(buffer," Game: Land Z!");
		set_lcd_display(LCD_BUFFER_1,buffer);
	} else if (r_game.game == GAME_COME_HITHER) {
		sprintf(buffer," Game: Land Z!");
		set_lcd_display(LCD_BUFFER_1,buffer);
	} else if (r_game.game == GAME_COME_HITHER) {
		sprintf(buffer," Game: Unknown :-(");
		set_lcd_display(LCD_BUFFER_1,buffer);
	}
	set_lcd_display(1," waiting ... " );


	if        (r_game.start_option == GAME_START_RANDOM) {
		// TODO ###################
		init_x=ROCKET_HOME_X;
		init_y=ROCKET_HOME_Y;
	} else {
		init_x=ROCKET_HOME_X;
		init_y=ROCKET_HOME_Y;
	}

	// init the rocket
	if        (r_game.game == GAME_Z_LAND) {
		init_rocket_game(init_x, init_y, GAME_Z_POS_MAX, r_game.fuel_option, r_game.gravity_option,GAME_PLAY);
	} else if (r_game.game == GAME_XYZ_LAND) {
		init_rocket_game(init_x, init_y, GAME_Z_POS_MAX, r_game.fuel_option, r_game.gravity_option,GAME_PLAY);
	} else if (r_game.game == GAME_XYZ_FLIGHT) {
		init_rocket_game(init_x, init_y, ROCKET_HOME_Z, r_game.fuel_option, r_game.gravity_option,GAME_PLAY);
	} else if (r_game.game == GAME_XYZ_AUTO) {
		init_rocket_game(init_x, init_y, GAME_Z_POS_MAX, r_game.fuel_option, r_game.gravity_option,GAME_PLAY);
	} else if (r_game.game == GAME_XYZ_MOVE) {
		init_rocket_game(init_x, init_y, ROCKET_HOME_Z /* GAME_Z_POS_MAX/2 */, r_game.fuel_option, r_game.gravity_option,GAME_PLAY);
	} else {
		init_rocket_game(init_x, init_y, GAME_Z_POS_MAX, r_game.fuel_option, r_game.gravity_option,GAME_PLAY);
	}

    // Preset the game display default
	if (IO_BUTTON_BRINGUP) {
	    r_game.play_display_mode = GAME_DISPLAY_RAW_XYZF;
	} else {
	    r_game.play_display_mode = GAME_DISPLAY_NORMAL;
	}
}

/*
 * init_main - initialize top-level game variables
 *
 */

void init_main() {
    // Preset the game defaults
	if (IO_BUTTON_BRINGUP) {
	    r_game.game = GAME_XYZ_MOVE;
	} else {
	    r_game.game = GAME_XYZ_LAND;
	}
    r_game.fuel_option = GAME_FUEL_NOLIMIT /*GAME_FUEL_NORMAL*/;
    r_game.gravity_option = GAME_GRAVITY_NONE /* GAME_GRAVITY_NORMAL */;
    r_game.start_option = GAME_START_CENTER;

	// set initial game controls
	r_control.button_a=0;
	r_control.button_b=0;
	r_control.button_a_prev=false;
    r_control.button_b_prev=false;

	r_control.analog_x=JOYSTICK_X_MID;
	r_control.analog_y=JOYSTICK_Y_MID;
	r_control.analog_z=JOYSTICK_Z_MID;

}

/*
 * main - Rocket Lander
 *
 */
 
 void main_time_test(uint32_t time_start,uint32_t time_stop,uint32_t time_cycle_start,uint32_t time_cycle_stop) {
	static uint32_t time_sum = 0L;
	static uint32_t time_cnt = 0L;
	static uint32_t time_cycle_sum = 0L;
	static uint32_t time_cycle_cnt = 0L;
	static uint32_t time_max3 = 0L;
	static uint32_t time_max2 = 0L;
	static uint32_t time_max1 = 0L;
	
	uint32_t time_diff = 0;
	
	if (DEBUG_TIMING_ENABLE) {
		time_cnt++;
		if (time_cnt > 0) {
			time_diff = time_stop - time_start;
			time_sum += time_diff;
			if (time_cycle_stop > time_cycle_start) {
				time_cycle_cnt++;
				time_cycle_sum += time_cycle_stop - time_cycle_start;
			}
			
			if (time_diff > time_max3) {
			    time_max1 = time_max2;
			    time_max2 = time_max3;
			    time_max3 = time_diff;
			} else if (time_diff > time_max2) {
			    time_max1 = time_max2;
			    time_max2 = time_diff;
			} else if (time_diff > time_max1) {
			    time_max1 = time_diff;
			}
	
			// display time ave every 64 * 1/5 second ~= 13 seconds
			if (0x0000 == (time_cnt & 0x003f)) {
				//PRINT("*** Main_Time(%ld) = %ld:%ld / %ld, %ld < %ld < %ld\n",time_cnt,time_sum/time_cnt,time_cycle_sum/time_cycle_cnt,sys_clock_ticks_per_sec,time_max3,time_max2,time_max1);
			}
		}
	}
}


void main() {
    bool flag = false;
   	uint32_t time_start,time_stop;
	uint32_t time_cycle_start,time_cycle_stop;

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
		goto_state("S_Start");
	}

    while (1)
        {
        	
		/* get the time at start of loop */
	   	time_start = task_tick_get_32();
	   	time_cycle_start = task_cycle_get_32();

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

        /* wait a while to loop again, less the time spent in loop */
	   	time_stop = task_tick_get_32();
	   	time_cycle_stop = task_cycle_get_32();
		main_time_test(time_start,time_stop,time_cycle_start,time_cycle_stop);
	   	if (time_stop < time_start) {
	   		/* time counter looped, use default delay */
	        task_sleep(SLEEPTICKS - 1);
	   	} else if (SLEEPTICKS <= (time_stop - time_start)) {
	        /* no sleep for the busy */
	   	} else {
	   		/* sleep the balance, rounding down one tick */
	        task_sleep(SLEEPTICKS - (time_stop - time_start) - 1);
        }
    }
}

/*
 * @brief Callback, which is invoked when ADC gets new data
 *
 * @param dev ADC device structure
 * @param cb_type can be ADC_CB_DONE or ADC_CB_ERROR
 */
void adcCallback (struct device *dev, enum adc_callback_type cb_type) {
    if (dev == adc && cb_type == ADC_CB_DONE)
        {
        isr_event_send(ADCREADY);
		}
}
