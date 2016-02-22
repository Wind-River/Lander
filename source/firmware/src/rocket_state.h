/* rocket_state.h - Rocket Lander Game */

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
 

#define STATE_NO_FLAGS       0x0000
#define STATE_NO_DISPLAY     0x0001
#define STATE_NO_VERBOSE     0x0002
#define STATE_BUTTON_HOLD_A  0x0010
#define STATE_BUTTON_HOLD_B  0x0020
#define STATE_FROM_CALLBACK  0x0040
#define STATE_NOT_FOUND 999


struct StateGuiRec {
	char*		state_name;	// String name of state
	uint32_t	state_flags;		// Optional state flags
	char 		display_1[LCD_DISPLAY_POS_MAX+100]; 	// Display string Line 1 (16 chars) (empty string for no change)
	char		display_2[LCD_DISPLAY_POS_MAX+100]; 	// Display string Line 2 (16 chars)
	char*		k1;			// Key1 goto state name (Use <STATE_NOP> for no action)
	char*		k2;			// Key2 goto state name
	char*		state_enter; // Callback on state entry (Use <ACTION_NOP> for no action)
	char*		state_loop;	// Callback on state loop
	char*		state_exit;	// Callback on state exit
};

//extern int32_t state_now;

void init_state();
void state_loop();
void goto_state(char *select_state_name);
void set_lcd_display(int line,char *buffer);

