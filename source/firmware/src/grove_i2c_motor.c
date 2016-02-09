// grove_i2c_motor.c :

/*
  Grove- i2C motor driver demo v1.0
  by: http://www.seeedstudio.com
//  Author:LG
//
//
//  This demo code is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
*/

#include <zephyr.h>

#include <i2c.h>
#include "grove_i2c_motor.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "rocket.h"

///////////////////////////////////////////////////////////////////////////////

void send_i2c_motor(uint8_t motor_number, uint8_t *buffer, uint8_t i2c_len) {
	uint8_t i2c_address=I2CMotorDriverAdd;

	if (IO_XYZ_ENABLE) {
		if (0 == motor_number) {
			i2c_address=I2CMotorDriverAdd;
		}

		i2c_polling_write (i2c, buffer, i2c_len, i2c_address);
	}
}


///////////////////////////////////////////////////////////////////////////////
// Enanble the i2c motor driver to drive a 4-wire stepper. the i2c motor driver will
//driver a 4-wire with 8 polarity  .
//Direction: stepper direction ; 1/0
//motor speed: defines the time interval the i2C motor driver change it output to drive the stepper
//the actul interval time is : motorspeed * 4ms. that is , when motor speed is 10, the interval time
//would be 40 ms

//////////////////////////////////////////////////////////////////////////////////
// set the steps you want, if 255, the stepper will rotate continuely;
void StepperStepset(uint8_t motor_number, uint8_t stepnu)
{
//  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
//  Wire.write(Stepernu);          // Send the stepernu command
//  Wire.write(stepnu);            // send the steps
//  Wire.write(Nothing);           // send nothing
//  Wire.endTransmission();        // stop transmitting

  	uint8_t buf[10];

  	buf[0]=Stepernu;
  	buf[1]=stepnu;
  	buf[2]=Nothing;
	send_i2c_motor(motor_number, buf,3);
}

void StepperMotorEnable(uint8_t motor_number, uint8_t Direction, uint8_t motorspeed)
{
//  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
//  Wire.write(EnableStepper);        // set pwm header
//  Wire.write(Direction);              // send pwma
//  Wire.write(motorspeed);              // send pwmb
//  Wire.endTransmission();    // stop transmitting

  	uint8_t buf[10];

  	buf[0]=EnableStepper;
  	buf[1]=Direction;
  	buf[2]=motorspeed;
	send_i2c_motor(motor_number, buf,3);
}

//function to uneanble i2C motor drive to drive the stepper.
void StepperMotorUnenable(uint8_t motor_number)
{
//  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
//  Wire.write(UnenableStepper);        // set unenable commmand
//  Wire.write(Nothing);
//  Wire.write(Nothing);
//  Wire.endTransmission();    // stop transmitting
  	uint8_t buf[10];

  	buf[0]=UnenableStepper;
  	buf[1]=Nothing;
  	buf[2]=Nothing;
	send_i2c_motor(motor_number, buf,3);
}

// the folling code sent command to I2C motor driver to drive a 4 wire stepper;

#define WAITTICKS ((2000 * sys_clock_ticks_per_sec) / 1000)

void stepper_unit_test(uint8_t motor_number)
{
	PRINT("sent command to + direction, very fast\n");
	StepperStepset(motor_number, 255);
	StepperMotorEnable(motor_number, 1, 1);// ennable the i2c motor driver a stepper.
	task_sleep(WAITTICKS);

	PRINT("sent command to - direction, slow\n");
	StepperStepset(motor_number, 255);
	StepperMotorEnable(motor_number, 0, 20);
	task_sleep(WAITTICKS);

	PRINT("sent command to - direction, fast\n");
	StepperMotorEnable(motor_number, 0, 2);// ennable the i2c motor driver a stepper.
	task_sleep(WAITTICKS);

	PRINT("sent command to + direction,100 steps, fast\n");
	StepperStepset(motor_number, 100);
	StepperMotorEnable(motor_number, 1,5);
	task_sleep(WAITTICKS);

	PRINT("sent command to shut down the stepper\n");
	StepperMotorUnenable(motor_number);
	task_sleep(WAITTICKS);

	PRINT("sent command to - direction, slow, and 10 steps then stop\n");
	StepperStepset(motor_number, 10);
	StepperMotorEnable(motor_number, 0,40);
	task_sleep(WAITTICKS);

	PRINT("sent command to shut down the stepper\n");
	StepperMotorUnenable(motor_number);
	task_sleep(WAITTICKS);
}

