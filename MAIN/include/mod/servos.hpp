/*
 * servos.hpp
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
 * 
 * Last Edit:  07/30/2020 (MM/DD/YYYY)
 */

#ifndef SERVOS_HPP
#define SERVOS_HPP

#include <stdio.h>
#include "i2c_driver.hpp"
#include "adafruit_servo_driver.hpp"
#include "rc_pilot_defs.h"
#include "mix_servos.hpp"
#include "servo_controller.hpp"

#ifndef MAX_SERVOS
#define MAX_SERVOS 16
#endif // !MAX_SERVOS

#define SERVO_MAX_US 2400.0		///< Default maximum pulse width in microseconds
#define SERVO_NOM_US 900.0		///< Default nominal pulse width in microseconds
#define SERVO_MIN_US 500.0		///< Default minimum pulse width in microseconds

#define SERVO_DEG_US_INC 1900.0/300.0

/*!
 *  @brief  Class that stores state and functions for interacting with servo motors
 */
class servo_state_t
{
private:
	bool initialized;							///< set to 1 after servos_init(void)
	i2c i2c_driver;								///< i2c driver interface
	Adafruit_PWMServoDriver_mod servo_driver;	///< servo driver interface

#ifdef RC_PILOT_DEFS_H //only need arming/disarming if used with rc_pilot
	arm_state_t arm_state;						///< servo arm state
	feedback_servo_controller_t controller;
#endif // RC_PILOT_DEFS_H

	uint64_t arm_time_ns;						///< arm time

	/*
	Min/nom/max defines operating range of each servo mottor.
	First column should the minimum position of each servo
	corresponding to the servo pulse signal in miliseconds
	(see rc_test_servos.c). Second column is the nominal/safe
	value for servos to return to when armed and before being
	disarmed. Signal should be in the range of [500 2500]us.
	For details, see rc_servo_send_pulse_us().
	*/
	double max_us[MAX_SERVOS];			///< servo maximum PWM calibrated value
	double nom_us[MAX_SERVOS];			///< servo nominal PWM calibrated value
	double min_us[MAX_SERVOS];			///< servo minimum PWM calibrated value

	double m[MAX_SERVOS];				///< current servo motor signals for each pin in [0 1] range
	double m_centered[MAX_SERVOS];		///< current servo motor signals for each pin in [-1 1] range
	double m_us[MAX_SERVOS];			///< current servo motor signals for each pin in PWM

	/*
	* This function should be used anytime
	* the servos need to be returned to
	* their nominal (safe) positions
	*/
	/* Returns 0 on success or -1 on failure */
	int set_nom_pulse(void);


	/*
	* This function should be used anytime
	* all the servos need to be set to
	* their min positions
	*/
	/* Returns 0 on success or -1 on failure */
	int set_min_pulse(void);


	/*
	* This function should be used anytime
	* all the servos need to be set to
	* their max positions
	*/
	/* Returns 0 on success or -1 on failure */
	int set_max_pulse(void);


	/*
	* This function converts commanded signal 
	* in [0 1] range into pulse width in 
	* miroseconds for motor i with respect to 
	* calibrated min and max.
	*/
	/* Returns 0 on success or -1 on failure */
	int cmnd2us(int i);

	/*
	* This function converts commanded signal
	* in [-1 1] range into pulse width in
	* miroseconds for motor i with respect to
	* the calibrated center, min and max.
	*/
	/* Returns 0 on success or -1 on failure */
	int cmnd2us_centered(int i);



	/*
	* This is how the user should change the servo
	* command. Accepts servo command signal in
	* between 0 and 1 range for servo i.
	*/
	/* Returns 0 on success or -1 on failure */
	int set_cmnd_signal(int i, double signal);


	/*
	* This is a simple saturation function which
	* limits the signal between 0 and 1
	*/
	/* Returns 0 on success or -1 on failure */
	int cmnd_signal_saturate(int i, double signal);

	/*
	* This is a simple saturation function which
	* limits the signal between -1 and 1
	*/
	/* Returns 0 on success or -1 on failure */
	int cmnd_signal_saturate_center(int i, double signal);

	int set_servo_calibration(void);

public:
	
	/*
	* This is how the user should initialize 
	* servos. Initializes both the servo driver
	* and i2c driver
	*/
	/* Returns 0 on success or -1 on failure */
	int init(int driver_bus_id);
	int init(int driver_bus_id, uint8_t devAddr);
	bool is_initialized(void);

#ifdef RC_PILOT_DEFS_H //only need arming/disarming if used with rc_pilot
	int return_to_nominal(void);

	int arm(void);
	bool is_armed(void);

	int march_controller(double(&u)[MAX_SERVO_INPUTS], double(&mot)[MAX_SERVOS]);

	int disarm(void);
#endif // RC_PILOT_DEFS_H //only need arming/disarming if used with rc_pilot
	
	/*
	* This is how the user should march (apply)
	* servo command. Input is commanded signal in 
	* 0 to 1 rangle, which gets coverted into PWM  
	* motor signal for motor i. 
	*/
	/* Returns 0 on success or -1 on failure */
	int march(int i, double signal);

	/*
	* This is how the user should march (apply)
	* servo command. Input is commanded signal in
	* -1 to 1 rangle, which gets coverted into PWM
	* motor signal for motor i.
	*/
	/* Returns 0 on success or -1 on failure */
	int march_with_centering(int i, double signal);

	/*
	* This is how the user should preset minimum 
	* servo pulse width in microseconds for motor i.
	*/
	/* Returns 0 on success or -1 on failure */
	int set_min_us(int i, double signal_us);


	/*
	* This is how the user should preset nominal
	* servo pulse width in microseconds for motor i.
	*/
	/* Returns 0 on success or -1 on failure */
	int set_nom_us(int i, double signal_us);


	/*
	* This is how the user should preset maximum
	* servo pulse width in microseconds for motor i.
	*/
	/* Returns 0 on success or -1 on failure */
	int set_max_us(int i, double signal_us);


	/*
	* This is testing function for checking the primary
	* functunality of servo driver. Sweeps servos from 0
	* to 1 for all channels.
	*/
	/* Returns 0 on success or -1 on failure */
	int test_min_max(void);


	/*
	* This is how the user should terminate the servo 
	* driver and i2c driver. Run this on exit.
	*/
	/* Returns 0 on success or -1 on failure */
	int cleanup(void);
};

/*!
 *  @brief  Class that stores state and functions for interacting with PCA9685
 * PWM chip
 */
extern servo_state_t sstate;

#endif //SERVOS_HPP