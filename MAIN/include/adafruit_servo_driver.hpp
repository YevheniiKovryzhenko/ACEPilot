/*
 * adafruit_servo_driver.hpp
 *
 * Copyright Yevhenii Kovryzhenko, Department of Aerospace Engineering, Auburn University.
 * Contact: yzk0058@auburn.edu
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL I
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef ADAFRUIT_SERVO_DRIVER_HPP
#define ADAFRUIT_SERVO_DRIVER_HPP

#include <stdio.h>
//#include "servos.hpp"
#include <unistd.h>
#include "i2c_driver.hpp"

// REGISTER ADDRESSES
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1 0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

class Adafruit_PWMServoDriver_mod
{
private:
	i2c device;
	int initialized = 0;
	uint32_t _oscillator_freq;

	/*!
	 *  @brief  Checks if class has been initialized
	 */
	 /* Returns 0 on success or -1 on failure */
	bool check_init(void);
public:


	/*!
	*  @brief  Setups the I2C interface and hardware
	*  @param  prescale
	*          Sets External Clock (Optional)
	*/
	/* Returns 0 on success or -1 on failure */
	int init(i2c new_device);
	int init(i2c new_device, uint8_t prescale);
	

	/*!
	 *  @brief  Sends a reset command to the PCA9685 chip over I2C
	 */
	 /* Returns 0 on success or -1 on failure */
	int reset();


	/*!
	 *  @brief  Puts board into sleep mode
	 */
	 /* Returns 0 on success or -1 on failure */
	int sleep();

	/*!
	*  @brief  Wakes board from sleep
	*/
	/* Returns 0 on success or -1 on failure */
	int wakeup();


	/*!
	 *  @brief  Sets EXTCLK pin to use the external clock
	 *  @param  prescale
	 *          Configures the prescale value to be used by the external clock
	 */
	 /* Returns 0 on success or -1 on failure */
	int setExtClk(uint8_t prescale);


	/*!
	*  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
	*  @param  freq Floating point frequency that we will attempt to match
	*/
	/* Returns 0 on success or -1 on failure */
	int setPWMFreq(float freq);



	/*!
	*  @brief  Sets the output mode of the PCA9685 to either
	*  open drain or push pull / totempole.
	*  Warning: LEDs with integrated zener diodes should
	*  only be driven in open drain mode.
	*  @param  totempole Totempole if true, open drain if false.
	*/
	/* Returns 0 on success or -1 on failure */
	int setOutputMode(bool totempole);


	/*!
	*  @brief  Reads set Prescale from PCA9685
	*  @return prescale value
	*/
	/* Returns 0 on success or -1 on failure */
	int readPrescale(uint8_t* prescale);


	/*!
	*  @brief  Gets the PWM output of one of the PCA9685 pins
	*  @param  num One of the PWM output pins, from 0 to 15
	*  @return requested PWM output value
	*/
	/* Returns 0 on success or -1 on failure */
	int getPWM(uint8_t num, uint8_t* PWM);


	/*!
	*  @brief  Sets the PWM output of one of the PCA9685 pins
	*  @param  num One of the PWM output pins, from 0 to 15
	*  @param  on At what point in the 4096-part cycle to turn the PWM output ON
	*  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
	*/
	/* Returns 0 on success or -1 on failure */
	int setPWM(uint8_t num, uint16_t on, uint16_t off);



	/*!
	*  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
	* microseconds, output is not precise
	*  @param  num One of the PWM output pins, from 0 to 15
	*  @param  Microseconds The number of Microseconds to turn the PWM output ON
	*/
	/* Returns 0 on success or -1 on failure */
	int writeMicroseconds(uint8_t num, double pulse_us);


	/*!
	*  @brief  Getter for the internally tracked oscillator used for freq
	* calculations
	*  @returns The frequency the PCA9685 thinks it is running at (it cannot
	* introspect)
	*/
	/* Returns 0 on success or -1 on failure */
	int getOscillatorFrequency(uint32_t* freq);


	/*!
	*  @brief Setter for the internally tracked oscillator used for freq
	* calculations
	*  @param freq The frequency the PCA9685 should use for frequency calculations
	*/
	void setOscillatorFrequency(uint32_t freq);
};

#endif //ADAFRUIT_SERVO_DRIVER_HPP