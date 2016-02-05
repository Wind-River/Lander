/* rocket_state.h - Rocket Lander Game */

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

