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
 * Last Edit:  08/03/2020 (MM/DD/YYYY)
 */

#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include <rc/math.h>
#include <rc/time.h>
#include <rc/math/filter.h>

#include "mix.h"
#include <stdbool.h>

class feedback_controller_t
{
private:
	bool initialized;


	// for in-flight flight mode switching :
	bool last_en_rpy_ctrl;
	bool last_en_rpy_rate_ctrl;
	bool last_en_Z_ctrl;
	bool last_en_Zdot_ctrl;
	bool last_en_XY_ctrl;
	bool last_en_XYdot_ctrl;


	// keep original controller gains for scaling later:
	double D_roll_rate_pd_gain_orig, D_pitch_rate_pd_gain_orig, D_yaw_rate_pd_gain_orig;
	double D_roll_rate_i_gain_orig, D_pitch_rate_i_gain_orig, D_yaw_rate_i_gain_orig;
	
	double D_roll_pd_gain_orig, D_pitch_pd_gain_orig, D_yaw_pd_gain_orig;
	double D_roll_i_gain_orig, D_pitch_i_gain_orig, D_yaw_i_gain_orig;

	double D_Xdot_pd_gain_orig, D_Ydot_pd_gain_orig, D_Zdot_pd_gain_orig;
	double D_Xdot_i_gain_orig, D_Ydot_i_gain_orig, D_Zdot_i_gain_orig;

	double D_X_pd_gain_orig, D_Y_pd_gain_orig, D_X_i_gain_orig, D_Y_i_gain_orig;
	double D_Z_pd_gain_orig, D_Z_i_gain_orig;


	// filters:
	rc_filter_t D_roll_rate_pd;
	rc_filter_t D_roll_rate_i;
	rc_filter_t D_pitch_rate_pd;
	rc_filter_t D_pitch_rate_i;
	rc_filter_t D_yaw_rate_pd;
	rc_filter_t D_yaw_rate_i;

	rc_filter_t D_roll_pd;
	rc_filter_t D_pitch_pd;
	rc_filter_t D_yaw_pd;
	rc_filter_t D_roll_i;
	rc_filter_t D_pitch_i;
	rc_filter_t D_yaw_i;

	rc_filter_t D_Xdot_pd;
	rc_filter_t D_Xdot_i;
	rc_filter_t D_Ydot_pd;
	rc_filter_t D_Ydot_i;
	rc_filter_t D_Zdot_pd;
	rc_filter_t D_Zdot_i;

	rc_filter_t D_X_pd;
	rc_filter_t D_Y_pd;
	rc_filter_t D_X_i;
	rc_filter_t D_Y_i;

	rc_filter_t D_Z_pd;
	rc_filter_t D_Z_i;

	int rpy_init(void);
	int rpy_march(void);
	int rpy_reset(void);

	int rpy_rate_init(void);
	int rpy_rate_march(void);
	int rpy_rate_reset(void);

	int xy_init(void);
	int xy_march(void);
	int xy_reset(void);

	int xy_rate_init(void);
	int xy_rate_march(void);
	int xy_rate_reset(void);

	int z_init(void);
	int z_march(void);
	int z_reset(void);


	int z_rate_init(void);
	int z_rate_march(void);
	int z_rate_reset(void);

	int mix_all_control(double(&u)[MAX_INPUTS], double(&mot)[MAX_ROTORS]);

public:

	int init(void);

	int march(double(&u)[MAX_INPUTS], double(&mot)[MAX_ROTORS]);

	int reset(void);

};
#endif // CONTROLLER_HPP