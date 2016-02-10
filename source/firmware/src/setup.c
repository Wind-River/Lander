/* setup.c - GPIO setup routine */

/*
 * Copyright (c) 2015, Wind River Systems, Inc.
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


#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

/*
 * Initialize GPIO: pin 3 is input, pin 13 (on-board LED) is output.
 * Initialize I2C.
 */

#include <zephyr.h>
#include <i2c.h>
#include <pinmux.h>
#include <pinmux/pinmux.h>

#include "galileo2ADCPWM.h"

/* specify the number of Arduino connector pins used */
#define NUM_IO_PINS_USED CONFIG_PINMUX_NUM_PINS

struct device *gpio07;
struct device *gpioLegacy;
struct device *gpio_sus;
struct device *gpioEXP0;
struct device *gpioEXP1;
struct device *gpioEXP2;
struct device *i2c;
struct device *pinmux;

struct device *pwm;
struct device *adc;
struct device *pinmux;


/*
 * This table changes the default IO pin settings on the Galileo Gen2
 * board.
 *
 * The set of functions for each pin and the default pin settings are
 * defined in the Galileo platform source code:
 * zephyr/arch/x86/platforms/galileo/galileo_pinmux.c
 *
 * Set the PINMUX_FUNC_* value for a given pin in the following table
 * to represent the desired functionality.
 *
 * In this example:
 * - pin 3 is set as a GPIO input;
 * - pin 13 (on-board LED) is set as a GPIO output.
 *
 * All other pins in the table below are left as the default found in
 * galileo_pinmux.c
 */

static struct pin_config mux_config[NUM_IO_PINS_USED] = {
    /* pin, selected mode    <mode A, mode B, mode C, mode D> */
    /* Analog Inputs */
    { 0,  PINMUX_FUNC_C }, /* GPIO3 (out), GPIO3 (in), UART0_RXD, NA */
    { 1,  PINMUX_FUNC_C }, /* GPIO4 (out), GPIO4 (in), UART0_TXD, NA */
    { 2,  PINMUX_FUNC_C }, /* GPIO5 (out), GPIO5 (in), UART1_RXD, NA */
    { 3,  PINMUX_FUNC_B }, /* GPIO6 (out), GPIO6 (in), UART1_TXD, PWM.LED1 */
    { 4,  PINMUX_FUNC_B }, /* GPIO_SUS4 (out), GPIO_SUS4 (in), NA, NA */
    { 5,  PINMUX_FUNC_C }, /* GPIO8 (out), GPIO8 (in), PWM.LED3, NA */
    { 6,  PINMUX_FUNC_C }, /* GPIO9 (out), GPIO9 (in), PWM.LED5, NA */
    { 7,  PINMUX_FUNC_B }, /* EXP1.P0_6 (out), EXP1.P0_6 (in), NA, NA */
    { 8,  PINMUX_FUNC_B }, /* EXP1.P1_0 (out), EXP1.P1_0 (in), NA, NA */
    { 9,  PINMUX_FUNC_B }, /* GPIO_SUS2 (out), GPIO_SUS2 (in), PWM.LED7, NA */
    { 10, PINMUX_FUNC_B }, /* GPIO2 (out), GPIO2 (in), PWM.LED11, NA */
    { 11, PINMUX_FUNC_B }, /* GPIO_SUS3 (out), GPIO_SUS3 (in), PWM.LED9, SPI1_MOSI */
    { 12, PINMUX_FUNC_B }, /* GPIO7 (out), GPIO7 (in), SPI1_MISO, NA */
    { 13, PINMUX_FUNC_A }, /* GPIO_SUS5 (out), GPIO_SUS5(in), SPI1_SCK, NA */
    { 14, PINMUX_FUNC_B }, /* EXP2.P0_0 (out)/ADC.IN0, EXP2.P0_0 (in)/ADC.IN0, NA, NA */
    { 15, PINMUX_FUNC_B }, /* EXP2.P0_2 (out)/ADC.IN1, EXP2.P0_2 (in)/ADC.IN1, NA, NA */
    { 16, PINMUX_FUNC_B }, /* EXP2.P0_4 (out)/ADC.IN2, EXP2.P0_4 (in)/ADC.IN2, NA, NA */
    { 17, PINMUX_FUNC_B }, /* EXP2.P0_6 (out)/ADC.IN3, EXP2.P0_6 (in)/ADC.IN3, NA, NA */
    { 18, PINMUX_FUNC_C }, /* EXP2.P1_0 (out), ADC.IN4, I2C_SDA, NA */
    { 19, PINMUX_FUNC_C }, /* EXP2.P1_2 (out), ADC.IN5, I2C_SCL, NA */
};


/*
 * setup - Configure the Arduino connector IO pins for the application
 *
 * Returns DEV_OK if the configuration succeeded, otherwise DEV_FAIL.
 */

int setup (void)
    {
    int status;
    int i;

    /* get the device bindings */

    /* On-chip Quark GPIO */
    gpio07 = device_get_binding(CONFIG_GPIO_DW_0_NAME);            // GPIO[7:0]
    gpioLegacy = device_get_binding(CONFIG_GPIO_MMIO_0_DEV_NAME);  // GPIO[9:8]
    gpio_sus = device_get_binding(CONFIG_GPIO_MMIO_1_DEV_NAME);  //GPIO_SUS[5:0]

    /* external components */
    gpioEXP0 = device_get_binding(CONFIG_GPIO_PCAL9535A_0_DEV_NAME);
    gpioEXP1 = device_get_binding(CONFIG_GPIO_PCAL9535A_1_DEV_NAME);
    gpioEXP2 = device_get_binding(CONFIG_GPIO_PCAL9535A_2_DEV_NAME);

    /* i2c master of gpioEXP0/1/2 */
    i2c = device_get_binding(CONFIG_GPIO_PCAL9535A_1_I2C_MASTER_DEV_NAME);

	/* ADC and PWD */
    pwm  = device_get_binding(CONFIG_PWM_PCA9685_0_DEV_NAME);
    adc  = device_get_binding(CONFIG_ADC_TI_ADC108S102_0_DRV_NAME);

    pinmux = device_get_binding(PINMUX_NAME);

    if (!gpio07)
        {
        PRINT("GPIO DW not found!!\n");
        }

    if (!gpioLegacy)
        {
        PRINT("GPIO MMIO 0 not found!!\n");
        }

    if (!gpio_sus)
        {
        PRINT("GPIO MMIO 1 not found!!\n");
        }

    if (!gpioEXP0)
        {
        PRINT("EXP0 not found!!\n");
        }

    if (!gpioEXP1)
        {
        PRINT("EXP1 not found!!\n");
        }

    if (!gpioEXP2)
        {
        PRINT("EXP2 not found!!\n");
        }

    if (!i2c)
        {
        PRINT("I2C not found!!\n");
        }

    if (!adc)
        {
        PRINT("ADC not found!!\n");
        }

    if (!pwm)
        {
        PRINT("PWM not found!!\n");
        }

    if (!pinmux)
        {
        PRINT("Pinmux not found!!\n");
        }

    if (!(gpio07 && gpioLegacy && gpio_sus && gpioEXP0 &&
          gpioEXP1 && gpioEXP2 && i2c && pinmux))
        {
        PRINT("Stopped.\n");
        return(DEV_FAIL);
        }

    status = i2c_configure(i2c, (I2C_SPEED_FAST << 1) | I2C_MODE_MASTER);

    if (status != DEV_OK)
        {
        PRINT("I2C configuration error: %d Stopped.\n", status);
        return(DEV_FAIL);
        }

    for (i=0; i < NUM_IO_PINS_USED; i++)
        {
        status =
            pinmux_set_pin(pinmux, mux_config[i].pin_num, mux_config[i].mode);

        if (status != DEV_OK)
            {
            PRINT("Pin %d not configured!! Stopped.\n", mux_config[i].pin_num);
            return(DEV_FAIL);
            }
        else
            PRINT("Pin %d configured\n", mux_config[i].pin_num);

        }

    return(DEV_OK);

    }
