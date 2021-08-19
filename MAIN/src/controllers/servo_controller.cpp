/*
 * servo_controller.cpp
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
 * Last Edit:  08/18/2020 (MM/DD/YYYY)
 */
#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include <rc/math.h>
#include <rc/time.h>
#include <rc/math/filter.h>
#include "input_manager.hpp"
#include "setpoint_manager.hpp"
#include "mix_servos.hpp"
//#include "state_estimator.h"

#include "servo_controller.hpp"

 // preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

 /***************************************************************************
 * Roll Pitch Yaw controllers
 ***************************************************************************/
int feedback_servo_controller_t::rpy_init(void)
{
	

	last_en_rpy_ctrl = false;
	return 0;
}

int feedback_servo_controller_t::rpy_march(void)
{
	if (!last_en_rpy_ctrl)
	{
		rpy_reset();

		last_en_rpy_ctrl = true;
	}

	double err_roll, err_pitch, err_yaw;
	err_roll = setpoint.roll_servo;
	err_pitch = setpoint.pitch_servo;
	err_yaw = setpoint.yaw_servo;
	//use smooth transition if control blending is enabled:
	if (setpoint.en_rpy_trans) rpy_transition(err_roll, err_pitch, err_yaw);

	//feedback loop: angle error -> angular rate cmd.
	//no feedback for now:
	setpoint.roll_dot_servo = err_roll;
	setpoint.pitch_dot_servo = err_pitch;
	setpoint.yaw_dot_servo = err_yaw;

	//final selection and saturation: 
	if (!setpoint.en_rpy_rate_servo_ctrl)
	{
		setpoint.roll_servo_throttle = setpoint.roll_dot_servo;
		setpoint.pitch_servo_throttle = setpoint.pitch_dot_servo;
		setpoint.yaw_servo_throttle = setpoint.yaw_dot_servo;
	}
	else
	{
		rc_saturate_double(&setpoint.roll_dot_servo, -MAX_SERVO_ROLL_RATE, MAX_SERVO_ROLL_RATE);
		rc_saturate_double(&setpoint.pitch_dot_servo, -MAX_SERVO_PITCH_RATE, MAX_SERVO_PITCH_RATE);
		rc_saturate_double(&setpoint.yaw_dot_servo, -MAX_SERVO_YAW_RATE, MAX_SERVO_YAW_RATE);
	}

	last_en_rpy_ctrl = true;
	return 0;
}

int feedback_servo_controller_t::rpy_reset(void)
{
	
	return 0;
}

int feedback_servo_controller_t::rpy_transition(double& roll_err, double& pitch_err, double& yaw_err)
{
	r_transition(roll_err);
	p_transition(pitch_err);
	y_transition(yaw_err);
	return 0;
}
int feedback_servo_controller_t::r_transition(double& roll_err)
{
	roll_err = roll_err * (tanh(-7.0 - setpoint.roll_servo_tr / 0.06) + 1.0) / 2.0;
	return 0;
}
int feedback_servo_controller_t::p_transition(double& pitch_err)
{
	pitch_err = pitch_err * (tanh(-7.0 - setpoint.pitch_servo_tr / 0.06) + 1.0) / 2.0;
	return 0;
}
int feedback_servo_controller_t::y_transition(double& yaw_err)
{
	yaw_err = yaw_err * (tanh(7.0 + setpoint.yaw_servo_tr / 0.06) + 1.0) / 2.0;
	return 0;
}








/***************************************************************************
* Roll Pitch Yaw rate controllers
***************************************************************************/
int feedback_servo_controller_t::rpy_rate_init(void)
{
	

	last_en_rpy_rate_ctrl = false;

	return 0;
}

int feedback_servo_controller_t::rpy_rate_march(void)
{
	if (!last_en_rpy_rate_ctrl)
	{
		rpy_rate_reset();

		last_en_rpy_rate_ctrl = true;
	}
	
	double err_roll_dot, err_pitch_dot, err_yaw_dot;
	err_roll_dot = setpoint.roll_dot_servo;
	err_pitch_dot = setpoint.pitch_dot_servo;
	err_yaw_dot = setpoint.yaw_dot_servo;
	//use smooth transition if control blending is enabled:
	if (setpoint.en_rpy_trans) rpy_rate_transition(err_roll_dot, err_pitch_dot, err_yaw_dot);

	//feedback loop: angular rate error -> torque cmd.
	//no feedback for now:
	setpoint.roll_servo_throttle = err_roll_dot;
	setpoint.pitch_servo_throttle = err_pitch_dot;
	setpoint.yaw_servo_throttle = err_yaw_dot;

	//final selection and saturation: 

	last_en_rpy_rate_ctrl = true;
	return 0;
}

int feedback_servo_controller_t::rpy_rate_reset(void)
{
	
	return 0;
}

int feedback_servo_controller_t::rpy_rate_transition(double& roll_dot_err, \
	double& pitch_dot_err, double& yaw_dot_err)
{
	r_rate_transition(roll_dot_err);
	p_rate_transition(pitch_dot_err);
	y_rate_transition(yaw_dot_err);
	return 0;
}
int feedback_servo_controller_t::r_rate_transition(double& roll_dot_err)
{
	roll_dot_err = roll_dot_err * (tanh(7.0 + setpoint.roll_dot_tr / 0.06) + 1.0) / 2.0;
	return 0;
}
int feedback_servo_controller_t::p_rate_transition(double& pitch_dot_err)
{
	pitch_dot_err = pitch_dot_err * (tanh(4.9 + setpoint.pitch_dot_tr / 0.12) + 1.0) / 2.0;
	return 0;
}
int feedback_servo_controller_t::y_rate_transition(double& yaw_dot_err)
{
	yaw_dot_err = yaw_dot_err * (tanh(4.9 + setpoint.yaw_dot_tr / 0.12) + 1.0) / 2.0;
	return 0;
}




int feedback_servo_controller_t::xy_transition(double& x_err, double& y_err)
{
	x_err = x_err * (tanh(-7.0 - setpoint.X_servo_tr / 0.06) + 1.0) / 2.0;
	y_err = y_err * (tanh(-7.0 - setpoint.X_servo_tr / 0.06) + 1.0) / 2.0;
	return 0;
}


int feedback_servo_controller_t::init(void)
{
    if (initialized)
    {
        printf("\nWARNING in init: feedback servo controller already initialized");
        return 0;
    }

    initialized = true;
    return 0;
}

int feedback_servo_controller_t::mix_all_control(double(&u)[MAX_SERVO_INPUTS], double(&mot)[MAX_SERVOS])
{
	double min, max;

	/* 1. Throttle/Altitude Control */
	rc_saturate_double(&setpoint.Z_servo_throttle, MIN_SERVO_THRUST_COMPONENT, MAX_SERVO_THRUST_COMPONENT);
	u[VEC_Z] = setpoint.Z_servo_throttle;
	servo_mix.add_input(u[VEC_Z], VEC_Z, mot);


	/* 2. Roll (X) Control */
	servo_mix.check_saturation(VEC_ROLL, mot, &min, &max);
	if (max > MAX_SERVO_ROLL_COMPONENT)  max = MAX_SERVO_ROLL_COMPONENT;
	if (min < MIN_SERVO_ROLL_COMPONENT) min = MIN_SERVO_ROLL_COMPONENT;
	u[VEC_ROLL] = setpoint.roll_servo_throttle;
	rc_saturate_double(&u[VEC_ROLL], min, max);
	servo_mix.add_input(u[VEC_ROLL], VEC_ROLL, mot);

	/* 2. Pitch (Y) Control */
	servo_mix.check_saturation(VEC_PITCH, mot, &min, &max);
	if (max > MAX_SERVO_PITCH_COMPONENT)  max = MAX_SERVO_PITCH_COMPONENT;
	if (min < MIN_SERVO_PITCH_COMPONENT) min = MIN_SERVO_PITCH_COMPONENT;
	u[VEC_PITCH] = setpoint.pitch_servo_throttle;
	rc_saturate_double(&u[VEC_PITCH], min, max);
	servo_mix.add_input(u[VEC_PITCH], VEC_PITCH, mot);
	
	/* 3. Yaw (Z) Control */
	servo_mix.check_saturation(VEC_YAW, mot, &min, &max);
	if (max > MAX_SERVO_YAW_COMPONENT)  max = MAX_SERVO_YAW_COMPONENT;
	if (min < MIN_SERVO_YAW_COMPONENT) min = MIN_SERVO_YAW_COMPONENT;
	u[VEC_YAW] = setpoint.yaw_servo_throttle;
	rc_saturate_double(&u[VEC_YAW], min, max);
	servo_mix.add_input(u[VEC_YAW], VEC_YAW, mot);

	// for 6dof systems, add X and Y
	if (setpoint.en_6dof_servo) {
		// X
		servo_mix.check_saturation(VEC_X, mot, &min, &max);
		if (max > MAX_SERVO_X_COMPONENT)  max = MAX_SERVO_X_COMPONENT;
		if (min < MIN_SERVO_X_COMPONENT) min = MIN_SERVO_X_COMPONENT;
		u[VEC_X] = setpoint.X_servo_throttle;
		rc_saturate_double(&u[VEC_X], min, max);
		servo_mix.add_input(u[VEC_X], VEC_X, mot);

		// Y
		servo_mix.check_saturation(VEC_Y, mot, &min, &max);
		if (max > MAX_SERVO_Y_COMPONENT)  max = MAX_SERVO_Y_COMPONENT;
		if (min < MIN_SERVO_Y_COMPONENT) min = MIN_SERVO_Y_COMPONENT;
		u[VEC_Y] = setpoint.Y_servo_throttle;
		rc_saturate_double(&u[VEC_Y], min, max);
		servo_mix.add_input(u[VEC_Y], VEC_Y, mot);
	}

	return 0;
}


int feedback_servo_controller_t::march(double(&u)[MAX_SERVO_INPUTS], double(&mot)[MAX_SERVOS])
{
    if (!initialized)
    {
        printf("\nERROR in reset: feedback servo controller not initialized");
        return -1;
    }
	
	/* Make sure controller flags are set to off when not used for proper
	switching between flight modes - we want to make sure controllers are being
	reset every time flight mode is switched.
	*/
	if (!setpoint.en_rpy_ctrl) last_en_rpy_ctrl = false;
	if (!setpoint.en_rpy_rate_ctrl) last_en_rpy_rate_ctrl = false;

	// run attitude controllers if enabled
	if (setpoint.en_rpy_ctrl)
	{
		rpy_march(); //marches only attitude ctrl. + transition
	}
	else if (setpoint.en_rpy_servo_trans)
	{
		rpy_transition(setpoint.roll_servo_throttle,\
			setpoint.pitch_servo_throttle, setpoint.yaw_servo_throttle);

		if (user_input.flight_mode == ZEPPELIN)
		{
			r_transition(setpoint.X_servo_throttle);
		}
	}

	// run attitude rate controllers if enabled
	if (setpoint.en_rpy_rate_ctrl)
	{
		rpy_rate_march(); //marches only attitude rates ctrl. + transition
	}

	// now use motor mixing matrix to get individual servo inputs in [0 1] range
	mix_all_control(u, mot);


	return 0;
}


int feedback_servo_controller_t::reset(void)
{
    if (!initialized)
    {
        printf("\nERROR in reset: feedback servo controller not initialized");
        return -1;
    }

    return 0;
}