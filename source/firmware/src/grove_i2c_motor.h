// grove_i2c_motor.h :

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

#define MotorSpeedSet             0x82
#define PWMFrequenceSet           0x84
#define DirectionSet              0xaa
#define Nothing                   0x01
#define EnableStepper             0x1a
#define UnenableStepper           0x1b
#define Stepernu                  0x1c
#define I2CMotorDriverAdd         0x0f   // Set the address of the I2CMotorDriver

void send_i2c_motor(uint8_t motor_number, uint8_t *buffer, uint8_t i2c_len);
void StepperStepset(uint8_t motor_number, uint8_t stepnu);
void StepperMotorEnable(uint8_t motor_number, uint8_t Direction, uint8_t motorspeed);
void StepperMotorUnenable(uint8_t motor_number);
void stepper_unit_test(uint8_t motor_number);
