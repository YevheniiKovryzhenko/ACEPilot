/*
 * adafruit_servo_driver.cpp
 *
 * Author:	Yevhenii Kovryzhenko, Department of Aerospace Engineering, Auburn University.
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

#include "adafruit_servo_driver.hpp"

 // preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)


/*!
*  @brief  Setups the I2C interface and hardware
*  @param  prescale
*          Sets External Clock (Optional)
*/
/* Returns 0 on success or -1 on failure */
int Adafruit_PWMServoDriver_mod::init(i2c new_device)
{
	printf("\nInitializing servo driver...");
	device = new_device;
	initialized = 1;

	reset();
	
	// set a default frequency
	if (unlikely(setPWMFreq(1000) == -1))
	{
		printf("\nERROR: failed to initialize");
		return -1;
	}
	

	// set the default internal frequency
	setOscillatorFrequency(FREQUENCY_OSCILLATOR);

	printf("\nSucessfully initialized servo driver");
	return 0;
}


/*!
*  @brief  Setups the I2C interface and hardware
*  @param  prescale
*          Sets External Clock (Optional)
*/
/* Returns 0 on success or -1 on failure */
int Adafruit_PWMServoDriver_mod::init(i2c new_device, uint8_t prescale)
{
	printf("\nInitializing servo driver...");
	device = new_device;
	initialized = 1;

	reset();
	if (unlikely(setExtClk(prescale) == -1))
	{
		printf("\nERROR: failed to initialize");
		return -1;
	}

	// set the default internal frequency
	setOscillatorFrequency(FREQUENCY_OSCILLATOR);

	printf("\nSucessfully initialized servo driver");
	return 0;
}


/*!
 *  @brief  Sends a reset command to the PCA9685 chip over I2C
 */
int Adafruit_PWMServoDriver_mod::reset() 
{
	if (likely(check_init()))
	{
		if (device.write_byte(PCA9685_MODE1, MODE1_RESTART) == -1)
		{
			printf("\nERROR in reset: failed to send restart byte");
			return -1;
		}
		else
		{
			usleep(10E3);
		}
	}
	else 
	{
		printf("\nERROR in reset: not initialized");
		return -1;
	}
	return 0;
}

/*!
 *  @brief  Puts board into sleep mode
 */
int Adafruit_PWMServoDriver_mod::sleep() 
{
	if (likely(check_init()))
	{
		uint8_t awake;
		if (device.read_byte(PCA9685_MODE1, &awake) == -1)
		{
			printf("\nERROR in sleep: failed to read byte");
			return -1;
		}
		uint8_t sleep = awake | MODE1_SLEEP; // set sleep bit high
		if (device.write_byte(PCA9685_MODE1, sleep) == -1)
		{
			printf("\nERROR in sleep: failed to set sleep bt high");
			return -1;
		}
		usleep(5E3); // wait until cycle ends for sleep to be active
	}
	else 
	{
		printf("\nERROR in sleep: not initialized");
		return -1;
	}
	return 0;
}


/*!
 *  @brief  Wakes board from sleep
 */
 /* Returns 0 on success or -1 on failure */
int Adafruit_PWMServoDriver_mod::wakeup() 
{
	if (likely(check_init()))
	{
		uint8_t sleep;
		if (device.read_byte(PCA9685_MODE1,&sleep) == -1)
		{
			printf("\nERROR in wakeup: failed to read byte");
			return -1;
		}

		uint8_t wakeup = sleep & ~MODE1_SLEEP; // set sleep bit low
		if (device.write_byte(PCA9685_MODE1, wakeup) == -1)
		{
			printf("\nERROR in wakeup: failed to set sleep bit low");
			return -1;
		}
	}
	else 
	{
		printf("\nERROR in wakeup: not initialized");
		return -1;
	}
	return 0;
}

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
int Adafruit_PWMServoDriver_mod::setExtClk(uint8_t prescale) 
{
	if (likely(check_init()))
	{
		uint8_t oldmode;
		if (unlikely(device.read_byte(PCA9685_MODE1, &oldmode) == -1))
		{
			printf("\nERROR in setExtClk: Failed to read byte");
			return -1;
		}
		uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
		if (unlikely(device.write_byte(PCA9685_MODE1, newmode) == -1)) // go to sleep, turn off internal oscillator
		{
			printf("\nERROR in setExtClk: failed to go to sleep");
			return -1;
		}
		// This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
		// use the external clock.
		if (unlikely(device.write_byte(PCA9685_MODE1, (newmode |= MODE1_EXTCLK)) == -1))
		{
			printf("\nERROR in setExtClk: failed to sets both the SLEEP and EXTCLK bits");
			return -1;
		}
		if (unlikely(device.write_byte(PCA9685_PRESCALE, prescale) == -1)) // set the prescaler
		{
			printf("\nERROR in setExtClk: failed to set the prescaler");
			return -1;
		}

		usleep(5);
		// clear the SLEEP bit to start
		if (unlikely(device.write_byte(PCA9685_MODE1, \
			(newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI) == -1))
		{
			printf("\nERROR in setExtClk: failed to clear the SLEEP bit");
			return -1;
		}
	}
	else 
	{
		printf("\nERROR in setExtClk: not initialized");
		return -1;
	}
	return 0;
}


/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
 *  @param  freq Floating point frequency that we will attempt to match
 */
int Adafruit_PWMServoDriver_mod::setPWMFreq(float freq) 
{
	if (likely(check_init()))
	{
		// Range output modulation frequency is dependant on oscillator
		if (freq < 1)
		{
			printf("\nWARNING: trying to set PWM frequency less than 1, overwriting...");
			freq = 1;
		}
		if (freq > 3500)
		{
			printf("\nWARNING: trying to set PWM frequency more than 3500 (50Hz), overwriting...");
			freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)
		}

		float prescaleval = ((_oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
		if (prescaleval < PCA9685_PRESCALE_MIN)\
			prescaleval = PCA9685_PRESCALE_MIN;
		if (prescaleval > PCA9685_PRESCALE_MAX)\
			prescaleval = PCA9685_PRESCALE_MAX;
		uint8_t prescale = (uint8_t)prescaleval;


		uint8_t oldmode; 
		if (unlikely(device.read_byte(PCA9685_MODE1, &oldmode) == -1))
		{
			printf("\nERROR in setPWMFreq: failed to read mode");
			return -1;
		}
		uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
		if (unlikely(device.write_byte(PCA9685_MODE1, newmode) == -1))    // go to sleep
		{
			printf("\nERROR in setPWMFreq: failed to go to sleep");
			return -1;
		}
		if (unlikely(device.write_byte(PCA9685_PRESCALE, prescale) == -1)) // set the prescaler
		{
			printf("\nERROR in setPWMFreq: failed to set the prescaler");
			return -1;
		}
		if (unlikely(device.write_byte(PCA9685_MODE1, oldmode) == -1))
		{
			printf("\nERROR in setPWMFreq: failed to wakeup");
			return -1;
		}
		usleep(5);
		// This sets the MODE1 register to turn on auto increment.
		if (unlikely(device.write_byte(PCA9685_MODE1,\
			oldmode | MODE1_RESTART | MODE1_AI) == -1))
		{
			printf("\nERROR in setPWMFreq: failed to turn on auto increment");
			return -1;
		}
	}
	else 
	{
		printf("\nERROR in setPWMFreq: not initialzied");
		return -1;
	}
	
	return 0;
}


/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull / totempole.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
int Adafruit_PWMServoDriver_mod::setOutputMode(bool totempole) 
{
	if (likely(check_init()))
	{
		uint8_t oldmode;
		if (unlikely(device.read_byte(PCA9685_MODE2, &oldmode) == -1))
		{
			printf("\nERROR in setOutputMode: failed to read byte");
			return -1;
		}
		uint8_t newmode;
		if (totempole) {
			newmode = oldmode | MODE2_OUTDRV;
		}
		else {
			newmode = oldmode & ~MODE2_OUTDRV;
		}
		if (unlikely(device.write_byte(PCA9685_MODE2, newmode) == -1))
		{
			printf("\nERROR in setOutputMode: failed to set new mode");
			return -1;
		}
	}
	else 
	{
		printf("\nERROR in setOutputMode: not initialized");
		return -1;
	}
	
	return 0;
}


/*!
 *  @brief  Reads set Prescale from PCA9685
 *  @return prescale value
 */
 /* Returns 0 on success or -1 on failure */
int Adafruit_PWMServoDriver_mod::readPrescale(uint8_t* prescale) 
{
	if (likely(check_init()))
	{
		if (unlikely(device.read_byte(PCA9685_PRESCALE, prescale) == -1))
		{
			printf("\nERROR in readPrescale: failed to read byte");
		}
	}
	else 
	{
		printf("\nERROR in readPrescale: not initialized");
		return -1;
	}
	return 0;
}


/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @return requested PWM output value
 */
 /* Returns 0 on success or -1 on failure */
int Adafruit_PWMServoDriver_mod::getPWM(uint8_t num, uint8_t* PWM) 
{
	if (likely(check_init()))
	{
		if (unlikely(device.read_byte(PCA9685_LED0_ON_L + 4 * num,\
			PWM) == -1))
		{
			printf("\nERROR in getPWM: failed to read byte");
			return -1;
		}
	}
	else 
	{
		printf("\nERROR in getPWM: not initialized");
		return -1;
	}
	
	return 0;
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 */
 /* Returns 0 on success or -1 on failure */
int Adafruit_PWMServoDriver_mod::setPWM(uint8_t num, uint16_t on, uint16_t off) {
	if (likely(check_init()))
	{
		// must send (regAdrr, data, data, data, data)
		uint8_t tmp[4] = {(uint8_t) on,(uint8_t) (on >> 8),(uint8_t) off,(uint8_t) (off >> 8) };
		if (unlikely(device.write_bytes((uint8_t)(PCA9685_LED0_ON_L + 4 * num),\
			sizeof(tmp), tmp) == -1))
		{
			printf("\nERROR in setPWM: failed to send bytes, size %d",sizeof(tmp));
			return -1;
		}
	}
	else 
	{
		printf("\nERROR in setPWM: not initialized");
		return -1;
	}
	return 0;
}


/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
 * microseconds, output is not precise
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
 */
 /* Returns 0 on success or -1 on failure */
int Adafruit_PWMServoDriver_mod::writeMicroseconds(uint8_t num,
	double pulse_us)
{
	if (likely(check_init()))
	{
		double pulselength = 1E6; // 1,000,000 us per second

		// Read prescale
		uint8_t prescale_tmp;
		if (unlikely(readPrescale(&prescale_tmp) == -1))
		{
			printf("\nERROR in writeMicroseconds: failed to read prescale");
			return -1;
		}
		uint16_t prescale = prescale_tmp;


		// Calculate the pulse for PWM based on Equation 1 from the datasheet section
		// 7.3.5
		prescale += 1;
		pulselength *= (double) prescale;
		pulselength /= _oscillator_freq;

		pulse_us /= pulselength;
		//printf("\npulse length = %f prescale = %d", pulselength, prescale);
		//printf("\ndouble pulse_us = %f", pulse_us);
		//printf("\n(uint16_t) pulse_us = %d", (uint16_t) pulse_us);

		setPWM(num, 0, pulse_us);
	}
	else
	{
		printf("\nERROR in writeMicroseconds: not initialized :%d",initialized);
		return -1;
	}
	return 0;
}


/*!
 *  @brief  Getter for the internally tracked oscillator used for freq
 * calculations
 *  @returns The frequency the PCA9685 thinks it is running at (it cannot
 * introspect)
 */
 /* Returns 0 on success or -1 on failure */
int Adafruit_PWMServoDriver_mod::getOscillatorFrequency(uint32_t* freq) 
{
	if (likely(check_init()))
		*freq = _oscillator_freq;
	else
	{
		printf("ERROR in getOscillatorFrequency: not initialized");
		return -1;
	}
	return 0;
}



/*!
 *  @brief Setter for the internally tracked oscillator used for freq
 * calculations
 *  @param freq The frequency the PCA9685 should use for frequency calculations
 */
void Adafruit_PWMServoDriver_mod::setOscillatorFrequency(uint32_t freq) 
{
	_oscillator_freq = freq;
	return;
}

/*!
*  @brief  Checks if class has been initialized
*/
bool Adafruit_PWMServoDriver_mod::check_init(void)
{
	if (likely(initialized == 1))
	{
		return 1;
	}
	else
	{
		printf("\nERROR: Servo driver not initialized, call init() first :%d.",initialized);
		return 0;
	}
}