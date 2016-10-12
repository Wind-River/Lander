### Overview

We will present a physical interactive model of the famous LEM Lander game, where the user attempts to land a Rocket model (suspended from a four tower cable system) that has limited fuel onto a crater-filled lunar surface.  The game has outputs to indicate the speed, altitude, and fuel level remaining, together with light and sound effects. There will be physical controller inputs for the thrust and direction, plus controls for the game play.

### Project License

The license for this project is the Apache-2.0. Text of Apache-2.0 license and other applicable license notices can be found in the LICENSE-NOTICES.txt file in the top level directory. Each source files should include a license notice that designates the licensing terms for the respective file.

### Legal Notices

Disclaimer of Warranty / No Support: Wind River does not provide support and maintenance services for this software, under Wind River’s standard Software Support and Maintenance Agreement or otherwise. Unless required by applicable law, Wind River provides the software (and each contributor provides its contribution) on an “AS IS” BASIS, WITHOUT WARRANTIES OF ANY KIND, either express or implied, including, without limitation, any warranties of TITLE, NONINFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A PARTICULAR PURPOSE. You are solely responsible for determining the appropriateness of using or redistributing the software and assume ay risks associated with your exercise of permissions under the license.

### Materials Used

 * Galileo Rev2 running the Rocket (Zephyr) OS
 * Two Arduino Coprocessors 
 * 4 Stepper Motors and Drivers 
 * I2C LCD Display
 * I2C LED Large Display
 * 2 TM1637 LED Displays
 * Audio FX Sound Board
 * Pan and Tilt mount with 2 servo motors
 * Digital RGB LED Strip
 * Arcade Joystick, Buttons, and Slider Controls
 * AC power for a power strip connection
 * An internet connection (for Helix Application Cloud)
 * W=28”, D=20”, H=24” play field

### Software Used

  * Wind River Helix Application Cloud (HAC), open registration
    * http://app.cloud.windriver.com
  * Helix Application Cloud, sample application "Grove-LCD"

#### CAD and STL

  * All STL files are located in the /cad directory
  * CAD drawings for the Lander are available at - https://cad.onshape.com/documents/7df8b4dcfbc827b905a6e076

### Installation

  * Set up a free Wind River Helix Application Cloud (HAC) account (https://app.cloud.windriver.com)
  * Download the card image for the Galileo from HAC, write it to a microSD card, and insert into the Galileo
  * Create a new "Rocket_Lander" project in HAC, using the "Grove-LCD" sample application as the base
  * Set up a Galilieo Rev2 board with the following hardware:
    * A Seeed Studio Grove base shield
    * A Seeed Studio Grove button to D3
    * A Seeed Studio Grove button to D7
    * A Seeed Studio Grove LCD module to I2C
    * An FTDI cable to your host
  * Boot the Galileo, download the "Rocket_Lander" project, and do a test run with this initial application
  * Follow the Frizting diagram to attach the remaining hardware, for example:
     * The joystick cable to A0
     * The rheostat slider to A3
     * A Seeed Studio Grove PWM splitter cable from D5 to the pan and tilt stage
     * A Seeed Studio Grove breakout cable from I2C to the Adafruit I2C LED backpack
     * A Seeed Studio Grove cable with +5v cut from I2C to each of the the satellite boards
  * Follow the Frizting diagram to construct the motor driver circuit board 
  * Add code from the "source/firmware" to the "Rocket_Lander" project
  * Add code from the "source/satelliteware/libraries" to Arduino libraries directory
  * Set up the Rocket Display Arduino board
    * Install code from the "source/satelliteware/rocket_display"
    * Attach the two Seeed Studio Grove LED displays
    * Attach the Seeed Studio Grove Chainable RGB LED
    * Attach the Adafruit sound board
  * Set up the Rocket Motor Arduino board
    * Install code from the "source/satelliteware/rocket_motor"
    * Connect to the motor driver circuit board
  * Assemble the play field with the towers, stepper motors, fish line cables, and the model rocket:
    * cut a board to 28 inches by 22 inches for the playing field
    * cut four 1/2 inch square aluminum rods to 24 inches, to be the line towers
    * 3D print the four rod/motor mounts, and install on the playing field at 24 inches by 18 inches
    * 3D print the four spool cylinders and attach to the stepper motors
    * 3D print the four line rollers, and install at the top of the four towers
    * 3D print the rocket model, or provide a desired alternative
    * cut four lenghts of fish line to be the line cable, to about 60 inches
    * attach each of the four cable lines from each of the spools to the respective rocket model corner
    * attach the pan and tilt stage
    * connect the motor driver board to the stepper motors
  * Power the system, run the full "Rocket_Lander" project from HAC, follow the play instructions on the LED display interface
