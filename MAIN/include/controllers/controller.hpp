/*
 * controller.hpp
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
 * Last Edit:  08/31/2022 (MM/DD/YYYY)
 */

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include <rc/math.h>
#include <rc/time.h>
#include <rc/math/filter.h>

#include "mix.hpp"
#include <stdbool.h>
#include "feedback_filter_gen.hpp"

/* This is a class for the actual control system of the vehicle (ESC control) */
class feedback_controller_t
{
private:
	bool initialized = false;


	// for in-flight flight mode switching :
	bool last_en_rpy_ctrl = false;
	bool last_en_rpy_rate_ctrl = false;
	bool last_en_Z_ctrl = false;
	bool last_en_Zdot_ctrl = false;
	bool last_en_XY_ctrl = false;
	bool last_en_XYdot_ctrl = false;

	// filters:
	feedback_filter_gen_t roll_dot;
	feedback_filter_gen_t pitch_dot;
	feedback_filter_gen_t yaw_dot;

	feedback_filter_gen_t roll;
	feedback_filter_gen_t pitch;
	feedback_filter_gen_t yaw;

	feedback_filter_gen_t x_dot;
	feedback_filter_gen_t y_dot;
	feedback_filter_gen_t z_dot;

	feedback_filter_gen_t x;
	feedback_filter_gen_t y;
	feedback_filter_gen_t z;

	bool tune_status_fl = false; //indicates if gains have been updated last time 
	PID_vars_set_t received_gain_set;

	int rpy_init(void);
	int rpy_march(void);
	int rpy_reset(void);
	int rpy_transition(double& roll_err, double& pitch_err, double& yaw_err);

	int rpy_rate_init(void);
	int rpy_rate_march(void);
	int rpy_rate_reset(void);
	int rpy_rate_transition(double& roll_dot_err, double& pitch_dot_err, double& yaw_dot_err);

	int xy_init(void);
	int xy_march(void);
	int xy_reset(void);

	int xy_rate_init(void);
	int xy_rate_march(void);
	int xy_rate_reset(void);

	int XY_accel_2_attitude(void);
	int Z_accel_2_throttle(void);

	int z_init(void);
	int z_march(void);
	int z_reset(void);


	int z_rate_init(void);
	int z_rate_march(void);
	int z_rate_reset(void);

	int mix_all_control(double(&u)[MAX_INPUTS], double(&mot)[MAX_ROTORS]);

public:

	int init(void);

	char gain_tune_march(void);
	char update_gains(void);

	int march(double(&u)[MAX_INPUTS], double(&mot)[MAX_ROTORS]);

	int reset(void);

};
#endif // CONTROLLER_HPP