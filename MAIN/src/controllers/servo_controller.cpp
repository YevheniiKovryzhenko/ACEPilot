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
 * Last Edit:  07/29/2020 (MM/DD/YYYY)
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

#include "servo_controller.hpp"



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
	rc_saturate_double(&setpoint.Z_servo_throttle, MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
	u[VEC_Z] = setpoint.Z_servo_throttle;
	servo_mix.add_input(u[VEC_Z], VEC_Z, mot);


	/* 2. Roll (X) Control */
	servo_mix.check_saturation(VEC_ROLL, mot, &min, &max);
	if (max > MAX_ROLL_COMPONENT)  max = MAX_ROLL_COMPONENT;
	if (min < -MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
	u[VEC_ROLL] = setpoint.roll_servo_throttle;
	rc_saturate_double(&u[VEC_ROLL], min, max);
	servo_mix.add_input(u[VEC_ROLL], VEC_ROLL, mot);

	/* 2. Pitch (Y) Control */
	servo_mix.check_saturation(VEC_PITCH, mot, &min, &max);
	if (max > MAX_PITCH_COMPONENT)  max = MAX_PITCH_COMPONENT;
	if (min < -MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
	u[VEC_PITCH] = setpoint.pitch_servo_throttle;
	rc_saturate_double(&u[VEC_PITCH], min, max);
	servo_mix.add_input(u[VEC_PITCH], VEC_PITCH, mot);
	
	/* 3. Yaw (Z) Control */
	servo_mix.check_saturation(VEC_YAW, mot, &min, &max);
	if (max > MAX_YAW_COMPONENT)  max = MAX_YAW_COMPONENT;
	if (min < -MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
	u[VEC_YAW] = setpoint.yaw_servo_throttle;
	rc_saturate_double(&u[VEC_YAW], min, max);
	servo_mix.add_input(u[VEC_YAW], VEC_YAW, mot);

	return 0;
}


int feedback_servo_controller_t::march(double(&u)[MAX_SERVO_INPUTS], double(&mot)[MAX_SERVOS])
{
    if (!initialized)
    {
        printf("\nERROR in reset: feedback servo controller not initialized");
        return -1;
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