1) Introduction

This is the source code for the Rocket Lander main board, the Galileo.


2) Board Bring Up

  (a) Setup the Galileo with a Grove LCD on I2C and a Grove buttom on D3

  (b) Get the Grove-LCD sample Helix Application running
  
This will show that Rocket is working with the miniaml I/O.

  (c) Setup the Rocket_Lander project in Helix
  
  (d) In the "Init" page, select "I/O Inputs"

The two button states and the 3 ADC inputs will be traced on the debug console, to insure that the full set up inputs are working. Push both buttons together to return to the "Init" page.

  (e) Select "Main > Test > Sanity Test" 
  
This will run the following sanity tests:
  * State table sanity
  * Square Root function unit test
  * Cable Lengths function unit test
  * Tower Stepper Counts function unit test

  (f) Select "Main > Test > Simulation"

There are several flight simulation modes, using the "Next" button select.
  *	"Movement in MicroMeter": move around the game space (in microMeters)
  * "Movement in MilliMeter": move around the game space (in milliMeters)
  * "Tower Cable Lengths"   : move around and observe cable length calc
  * "Tower Motor Steps"     " move around and observe cable and stepper diffs

  (g) TBD
  

3) Menu Structure:

	"Init"
		"I/O Inputs Test"
		"Main"

	"Main
		"Play"
		"Options"
		"Test"

	"Options"
		"Game Z"
		"Game XYZ"
		"Game Flight"
		"Game Move"
		"Game Auto Pilot"
	"Opt Gravity"
		"Gravity Full"
		"Gravity High"
		"Gravity None"
	"Opt Fuel"
		"Fuel Normal"
		"Fuel Low"
		"Fuel Nolimit"
	"Opt Pos"
		"Position Center"
		"Position Random"

	"Test"
		"I/O Inputs"
		"Test Sanity Test"
		"Test Simulation"
			"Simulation: Movement in MicroMeter"
			"Simulation: Movement in MilliMeter"
			"Simulation: Tower Cable Lengths"
			"Simulation: Tower Motor Steps"
		"Test Tower Motors"
		"Test Calibrate"
		"Test Antennae"
		"Test LED-RGB"
