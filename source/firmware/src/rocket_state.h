/* rocket_state.h - Rocket Lander Game */

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


#define STATE_NO_FLAGS       0x0000
#define STATE_NO_DISPLAY     0x0001		// do not update the LCD display (avoid unneeded overhead)
#define STATE_NO_VERBOSE     0x0002		// do not display the state on the serial console (avoid overflow)
#define STATE_BUTTON_HOLD_A  0x0010		// Treat button as on/off instead of toggle
#define STATE_BUTTON_HOLD_B  0x0020		// Treat button as on/off instead of toggle
#define STATE_FROM_CALLBACK  0x0040		// For usage validation, mark this state directly used by a callback
#define STATE_NO_MENU_TEXT   0x0080		// this state does not have menu labels for the buttons
#define STATE_NOT_FOUND         999

// reverse the button menus if the green button (B) is placed on the left,
// unless this state has STATE_NO_MENU_TEXT set
#define STATE_REVERSE_MENUS  true

#define ACTION_NOP 		NULL			// no callback action
#define STATE_NOP		NULL 			// no next state
#define STATE_INHERIT_S	"<INHERIT>"		// inherit button #1 state from parent
#define STATE_INHERIT_1	((char *)1L)	// inherit button #1 state from parent
#define STATE_INHERIT_2	((char *)2L)	// inherit button #2 state from parent

#define LCD_BUFFER_1  1 // top line of LCD
#define LCD_BUFFER_2  2 // bottom line of LCD

struct StateGuiRec {
	char*		state_name;	// String name of state
	uint32_t	state_flags;		// Optional state flags
	char 		display_1[LCD_DISPLAY_POS_MAX+100]; 	// Display string Line 1 (16 chars) (empty string for no change)
	char		display_2[LCD_DISPLAY_POS_MAX+100]; 	// Display string Line 2 (16 chars)
	char*		k1;			// Key1 goto state name (Use <STATE_NOP> for no action)
	char*		k2;			// Key2 goto state name
	void		(*state_enter)(); // Callback on state entry (Use <ACTION_NOP> for no action)
	void		(*state_loop)();	// Callback on state loop
	void		(*state_exit)();	// Callback on state exit
};

//extern int32_t state_now;

void init_state();
void state_loop();
void goto_state(char *select_state_name);
void set_lcd_display(int line,char *buffer);

