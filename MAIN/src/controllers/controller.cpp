/*
 * controller.cpp
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
 * Last Edit:  05/19/2022 (MM/DD/YYYY)
 */
#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include <rc/math.h>
#include <rc/time.h>
#include <rc/math/filter.h>
#include "settings.h"
#include "input_manager.hpp"
#include "setpoint_manager.hpp"
#include "state_estimator.h"

#include "controller.hpp"
#include "comms_tmp_data_packet.h"

 // preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

/***************************************************************************
* Roll Pitch Yaw controllers
***************************************************************************/
int feedback_controller_t::rpy_init(void)
{
    // filters
    D_roll_pd	= RC_FILTER_INITIALIZER;
    D_pitch_pd	= RC_FILTER_INITIALIZER;
    D_yaw_pd	= RC_FILTER_INITIALIZER;
	D_roll_i	= RC_FILTER_INITIALIZER;
	D_pitch_i	= RC_FILTER_INITIALIZER;
	D_yaw_i		= RC_FILTER_INITIALIZER;

    // get controllers from settings
	if (unlikely(rc_filter_duplicate(&D_roll_pd, settings.roll_controller_pd) == -1))
	{
		printf("Error in rpy_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_pitch_pd, settings.pitch_controller_pd) == -1))
	{
		printf("Error in rpy_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_yaw_pd, settings.yaw_controller_pd) == -1))
	{
		printf("Error in rpy_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_roll_i, settings.roll_controller_i) == -1))
	{
		printf("Error in rpy_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_pitch_i, settings.pitch_controller_i) == -1))
	{
		printf("Error in rpy_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_yaw_i, settings.yaw_controller_i) == -1))
	{
		printf("Error in rpy_init: failed to dublicate controller from settings\n");
		return -1;
	}

#ifdef DEBUG
    printf("ROLL CONTROLLER:\n");
    rc_filter_print(D_roll);
    printf("PITCH CONTROLLER:\n");
    rc_filter_print(D_pitch);
    printf("YAW CONTROLLER:\n");
    rc_filter_print(D_yaw);
#endif

	//save a copy of the original/default gain set:
	D_roll_pd_orig = D_roll_pd;
	D_pitch_pd_orig = D_pitch_pd;
	D_yaw_pd_orig = D_yaw_pd;
	D_roll_i_orig = D_roll_i;
	D_pitch_i_orig = D_pitch_i;
	D_yaw_i_orig = D_yaw_i;

	last_en_rpy_ctrl = false;
    return 0;
}

int feedback_controller_t::rpy_march(void)
{
	if (!last_en_rpy_ctrl)
	{
		setpoint.reset_att();
		setpoint.reset_att_ff();
		rpy_reset();

		last_en_rpy_ctrl = true;
	}

	if (settings.enable_v_gain_scaling)
	{
		// updating the gains based on battery voltage
		D_roll_pd.gain = D_roll_pd_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_pitch_pd.gain = D_pitch_pd_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_yaw_pd.gain = D_yaw_pd_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_roll_i.gain = D_roll_i_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_pitch_i.gain = D_pitch_i_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_yaw_i.gain = D_yaw_i_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
	}

	double err_roll, err_pitch, err_yaw;
	err_roll = setpoint.roll - state_estimate.roll;
	err_pitch = setpoint.pitch - state_estimate.pitch;
	err_yaw = setpoint.yaw - state_estimate.continuous_yaw;
	//use smooth transition if control blending is enabled:
	if (setpoint.en_rpy_trans) rpy_transition(err_roll, err_pitch, err_yaw); //zero out ff terms?

	// 1) Attitude -> Attitude Rate
	setpoint.roll_dot = rc_filter_march(&D_roll_pd, err_roll)
		+ rc_filter_march(&D_roll_i, err_roll) + setpoint.roll_dot_ff;
	setpoint.pitch_dot = rc_filter_march(&D_pitch_pd, err_pitch)
		+ rc_filter_march(&D_pitch_i, err_pitch) + setpoint.pitch_dot_ff;
	setpoint.yaw_dot = rc_filter_march(&D_yaw_pd, err_yaw)
		+ rc_filter_march(&D_yaw_i, err_yaw) + setpoint.yaw_dot_ff;

	

	if (!setpoint.en_rpy_rate_ctrl)
	{
		setpoint.roll_throttle = setpoint.roll_dot;
		setpoint.pitch_throttle = setpoint.pitch_dot;
		setpoint.yaw_throttle = setpoint.yaw_dot;
	}
	else
	{
		rc_saturate_double(&setpoint.roll_dot, -MAX_ROLL_RATE, MAX_ROLL_RATE);
		rc_saturate_double(&setpoint.pitch_dot, -MAX_PITCH_RATE, MAX_PITCH_RATE);
		rc_saturate_double(&setpoint.yaw_dot, -MAX_YAW_RATE, MAX_YAW_RATE);
	}

	last_en_rpy_ctrl = true;
	return 0;
}

int feedback_controller_t::rpy_reset(void)
{
	D_roll_pd = D_roll_pd_orig;
	D_pitch_pd = D_pitch_pd_orig;
	D_yaw_pd = D_yaw_pd_orig;
	D_roll_i = D_roll_i_orig;
	D_pitch_i = D_pitch_i_orig;
	D_yaw_i = D_yaw_i_orig;

    rc_filter_reset(&D_roll_pd);
    rc_filter_reset(&D_pitch_pd);
    rc_filter_reset(&D_yaw_pd);
	rc_filter_reset(&D_roll_i);
	rc_filter_reset(&D_pitch_i);
	rc_filter_reset(&D_yaw_i);

    // prefill filters with current error (only those with D terms)
    rc_filter_prefill_inputs(&D_roll_pd, -state_estimate.roll);
    rc_filter_prefill_inputs(&D_pitch_pd, -state_estimate.pitch);
    return 0;
}

int feedback_controller_t::rpy_transition(double& roll_err, \
	double& pitch_err, double& yaw_err)
{
	roll_err = roll_err * (tanh(7.0 + setpoint.roll_tr / 0.06) + 1.0) / 2.0;
	pitch_err = pitch_err * (tanh(4.9 + setpoint.pitch_tr / 0.12) + 1.0) / 2.0;
	yaw_err = yaw_err * (tanh(4.9 + setpoint.yaw_tr / 0.12) + 1.0) / 2.0;
	return 0;
}



/***************************************************************************
* Roll Pitch Yaw rate controllers
***************************************************************************/
int feedback_controller_t::rpy_rate_init(void)
{
	// filters
	D_roll_rate_pd = RC_FILTER_INITIALIZER;
	D_pitch_rate_pd = RC_FILTER_INITIALIZER;
	D_yaw_rate_pd = RC_FILTER_INITIALIZER;
	D_roll_rate_i = RC_FILTER_INITIALIZER;
	D_pitch_rate_i = RC_FILTER_INITIALIZER;
	D_yaw_rate_i = RC_FILTER_INITIALIZER;

	// get controllers from settings
	if (unlikely(rc_filter_duplicate(&D_roll_rate_pd, settings.roll_rate_controller_pd) == -1))
	{
		printf("Error in rpy_rate_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_pitch_rate_pd, settings.pitch_rate_controller_pd) == -1))
	{
		printf("Error in rpy_rate_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_yaw_rate_pd, settings.yaw_rate_controller_pd) == -1))
	{
		printf("Error in rpy_rate_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_roll_rate_i, settings.roll_rate_controller_i) == -1))
	{
		printf("Error in rpy_rate_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_pitch_rate_i, settings.pitch_rate_controller_i) == -1))
	{
		printf("Error in rpy_rate_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_yaw_rate_i, settings.yaw_rate_controller_i) == -1))
	{
		printf("Error in rpy_rate_init: failed to dublicate controller from settings\n");
		return -1;
	}

	//save a copy of the original/default gain set:
	D_roll_rate_pd_orig = D_roll_rate_pd;
	D_pitch_rate_pd_orig = D_pitch_rate_pd;
	D_yaw_rate_pd_orig = D_yaw_rate_pd;
	D_roll_rate_i_orig = D_roll_rate_i;
	D_pitch_rate_i_orig = D_pitch_rate_i;
	D_yaw_rate_i_orig = D_yaw_rate_i;

	last_en_rpy_rate_ctrl = false;

	return 0;
}

int feedback_controller_t::rpy_rate_march(void)
{
	if (!last_en_rpy_rate_ctrl)
	{
		setpoint.reset_att_dot();
		rpy_rate_reset();

		last_en_rpy_rate_ctrl = true;
	}

	if (settings.enable_v_gain_scaling)
	{
		// updating the gains based on battery voltage
		D_roll_rate_pd.gain = D_roll_rate_pd_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_pitch_rate_pd.gain = D_pitch_rate_pd_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_yaw_rate_pd.gain = D_yaw_rate_pd_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_roll_rate_i.gain = D_roll_rate_i_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_pitch_rate_i.gain = D_pitch_rate_i_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_yaw_rate_i.gain = D_yaw_rate_i_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
	}

	double err_roll_dot, err_pitch_dot, err_yaw_dot;
	err_roll_dot = setpoint.roll_dot - state_estimate.roll_dot;
	err_pitch_dot = setpoint.pitch_dot - state_estimate.pitch_dot;
	err_yaw_dot = setpoint.yaw_dot - state_estimate.yaw_dot;

	//use smooth transition if control blending is enabled:
	if (setpoint.en_rpy_rate_trans) rpy_rate_transition(err_roll_dot, err_pitch_dot, err_yaw_dot);

	// Attitude rate error -> Torque cmd.
	setpoint.roll_throttle = rc_filter_march(&D_roll_rate_pd, err_roll_dot)
		+ rc_filter_march(&D_roll_rate_i, err_roll_dot);
	setpoint.pitch_throttle = rc_filter_march(&D_pitch_rate_pd, err_pitch_dot)
		+ rc_filter_march(&D_pitch_rate_i, err_pitch_dot);
	setpoint.yaw_throttle = rc_filter_march(&D_yaw_rate_pd, err_yaw_dot)
		+ rc_filter_march(&D_yaw_rate_i, err_yaw_dot);

	last_en_rpy_rate_ctrl = true;
	return 0;
}

int feedback_controller_t::rpy_rate_reset(void)
{
	D_roll_rate_pd = D_roll_rate_pd_orig;
	D_pitch_rate_pd = D_pitch_rate_pd_orig;
	D_yaw_rate_pd = D_yaw_rate_pd_orig;
	D_roll_rate_i = D_roll_rate_i_orig;
	D_pitch_rate_i = D_pitch_rate_i_orig;
	D_yaw_rate_i = D_yaw_rate_i_orig;

	rc_filter_reset(&D_roll_rate_pd);
	rc_filter_reset(&D_roll_rate_i);
	rc_filter_reset(&D_pitch_rate_pd);
	rc_filter_reset(&D_pitch_rate_i);
	rc_filter_reset(&D_yaw_rate_pd);
	rc_filter_reset(&D_yaw_rate_i);

	// prefill filters with current error (only those with D terms)
	rc_filter_prefill_inputs(&D_roll_rate_pd, -state_estimate.roll_dot);
	rc_filter_prefill_inputs(&D_pitch_rate_pd, -state_estimate.pitch_dot);
	rc_filter_prefill_inputs(&D_yaw_rate_pd, -state_estimate.yaw_dot);
	return 0;
}


int feedback_controller_t::rpy_rate_transition(double& roll_dot_err,\
	double& pitch_dot_err, double& yaw_dot_err)
{
	roll_dot_err = roll_dot_err * (tanh(7.0 + setpoint.roll_dot_tr / 0.06) + 1.0) / 2.0;
	pitch_dot_err = pitch_dot_err * (tanh(4.9 + setpoint.pitch_dot_tr / 0.12) + 1.0) / 2.0;
	yaw_dot_err = yaw_dot_err * (tanh(4.9 + setpoint.yaw_dot_tr / 0.12) + 1.0) / 2.0;
	return 0;
}




/***************************************************************************
* X Y Position Controller
***************************************************************************/
int feedback_controller_t::xy_init(void)
{
    // filters
	D_X_pd	= RC_FILTER_INITIALIZER;
	D_Y_pd	= RC_FILTER_INITIALIZER;
	D_X_i	= RC_FILTER_INITIALIZER;
	D_Y_i	= RC_FILTER_INITIALIZER;

	if (unlikely(rc_filter_duplicate(&D_X_pd, settings.horiz_pos_ctrl_X_pd) == -1))
	{
		printf("Error in xy_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_X_i, settings.horiz_pos_ctrl_X_i) == -1))
	{
		printf("Error in xy_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_Y_pd, settings.horiz_pos_ctrl_Y_pd) == -1))
	{
		printf("Error in xy_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_Y_i, settings.horiz_pos_ctrl_Y_i) == -1))
	{
		printf("Error in xy_init: failed to dublicate controller from settings\n");
		return -1;
	}


	//save a copy of the original/default gain set:
	D_X_pd_orig = D_X_pd;
	D_Y_pd_orig = D_Y_pd;
	D_X_i_orig = D_X_i;
	D_Y_i_orig = D_Y_i;

	last_en_XY_ctrl = false;

    return 0;
}


int feedback_controller_t::xy_march(void)
{
	if (!last_en_XY_ctrl)
	{
		setpoint.reset_X();
		setpoint.reset_Y();
		setpoint.reset_X_dot_ff();
		setpoint.reset_Y_dot_ff();

		xy_reset();

		last_en_XY_ctrl = true;
	}

	////////////// PID for horizontal positon control /////////////
	if (settings.enable_v_gain_scaling)
	{
		// updating the gains based on battery voltage
		D_X_pd.gain = D_X_pd_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_Y_pd.gain = D_Y_pd_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_X_i.gain = D_X_i_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_Y_i.gain = D_Y_i_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
	}

	// Position error -> Velocity/Acceleration error
	setpoint.X_dot = rc_filter_march(&D_X_pd, setpoint.X - state_estimate.X)\
		+ rc_filter_march(&D_X_i, setpoint.X - state_estimate.X) + setpoint.X_dot_ff;
	setpoint.Y_dot = rc_filter_march(&D_Y_pd, setpoint.Y - state_estimate.Y)\
		+ rc_filter_march(&D_Y_i, setpoint.Y - state_estimate.Y) + setpoint.Y_dot_ff;
	rc_saturate_double(&setpoint.X_dot, -MAX_XY_VELOCITY, MAX_XY_VELOCITY);
	rc_saturate_double(&setpoint.Y_dot, -MAX_XY_VELOCITY, MAX_XY_VELOCITY);
	
	if (setpoint.en_XY_vel_ctrl)
	{
		// Velocity error -> Acceleration error
		xy_rate_march();
	}
	else
	{
		setpoint.X_ddot = setpoint.X_dot;
		setpoint.Y_ddot = setpoint.Y_dot;
	}

	// Acceleration error -> Lean Angles
	setpoint.roll = (-sin(state_estimate.continuous_yaw) * setpoint.X_ddot\
		+ cos(state_estimate.continuous_yaw) * setpoint.Y_ddot) / GRAVITY\
		+ setpoint.roll_ff;
	setpoint.pitch = -(cos(state_estimate.continuous_yaw) * setpoint.X_ddot\
		+ sin(state_estimate.continuous_yaw) * setpoint.Y_ddot) / GRAVITY\
		+ setpoint.pitch_ff;

	rc_saturate_double(&setpoint.roll, -MAX_ROLL_SETPOINT, MAX_ROLL_SETPOINT);
	rc_saturate_double(&setpoint.pitch, -MAX_PITCH_SETPOINT, MAX_PITCH_SETPOINT);

	last_en_XY_ctrl = true;
	return 0;
}

int feedback_controller_t::xy_reset(void)
{
	D_X_pd = D_X_pd_orig;
	D_Y_pd = D_Y_pd_orig;
	D_X_i = D_X_i_orig;
	D_Y_i = D_Y_i_orig;

	rc_filter_reset(&D_X_pd);
	rc_filter_reset(&D_Y_pd);
	rc_filter_reset(&D_X_i);
	rc_filter_reset(&D_Y_i);
	return 0;
}



/***************************************************************************
* X Y Velocity Controller
***************************************************************************/
int feedback_controller_t::xy_rate_init(void)
{
	// filters
	D_Xdot_pd = RC_FILTER_INITIALIZER;
	D_Xdot_i = RC_FILTER_INITIALIZER;
	D_Ydot_pd = RC_FILTER_INITIALIZER;
	D_Ydot_i = RC_FILTER_INITIALIZER;

	if (unlikely(rc_filter_duplicate(&D_Xdot_pd, settings.horiz_vel_ctrl_pd_X) == -1))
	{
		printf("Error in xy_rate_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_Xdot_i, settings.horiz_vel_ctrl_i_X) == -1))
	{
		printf("Error in xy_rate_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_Ydot_pd, settings.horiz_vel_ctrl_pd_Y) == -1))
	{
		printf("Error in xy_rate_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_Ydot_i, settings.horiz_vel_ctrl_i_Y) == -1))
	{
		printf("Error in xy_rate_init: failed to dublicate controller from settings\n");
		return -1;
	}


	//save a copy of the original/default gain set:
	D_Xdot_pd_orig = D_Xdot_pd;
	D_Xdot_i_orig = D_Xdot_i;
	D_Ydot_pd_orig = D_Ydot_pd;
	D_Ydot_i_orig = D_Ydot_i;

	last_en_XYdot_ctrl = false;
	return 0;
}

int feedback_controller_t::xy_rate_march(void)
{
	if (!last_en_XYdot_ctrl)
	{
		setpoint.reset_X_dot();
		setpoint.reset_Y_dot();
		
		xy_rate_reset();

		last_en_XYdot_ctrl = true;
	}

	if (settings.enable_v_gain_scaling)
	{
		// updating the gains based on battery voltage
		D_Xdot_pd.gain = D_Xdot_pd_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_Xdot_i.gain = D_Xdot_i_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_Ydot_pd.gain = D_Ydot_pd_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_Ydot_i.gain = D_Ydot_i_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
	}

	////////////// PID for horizontal velocity control /////////////

	setpoint.X_ddot = rc_filter_march(&D_Xdot_pd, setpoint.X_dot - state_estimate.X_dot)
		+ rc_filter_march(&D_Xdot_i, setpoint.X_dot - state_estimate.X_dot);
	setpoint.Y_ddot = rc_filter_march(&D_Ydot_pd, setpoint.Y_dot - state_estimate.Y_dot)
		+ rc_filter_march(&D_Ydot_i, setpoint.Y_dot - state_estimate.Y_dot);

	rc_saturate_double(&setpoint.X_ddot, -MAX_XY_ACCELERATION, MAX_XY_ACCELERATION);
	rc_saturate_double(&setpoint.Y_ddot, -MAX_XY_ACCELERATION, MAX_XY_ACCELERATION);
	
	last_en_XYdot_ctrl = true;
	return 0;
}

int feedback_controller_t::xy_rate_reset(void)
{
	D_Xdot_pd = D_Xdot_pd_orig;
	D_Xdot_i = D_Xdot_i_orig;
	D_Ydot_pd = D_Ydot_pd_orig;
	D_Ydot_i = D_Ydot_i_orig;


	rc_filter_reset(&D_Xdot_pd);
	rc_filter_reset(&D_Xdot_i);
	rc_filter_reset(&D_Ydot_pd);
	rc_filter_reset(&D_Ydot_i);

	rc_filter_prefill_inputs(&D_Xdot_pd, -state_estimate.X_dot);
	rc_filter_prefill_inputs(&D_Ydot_pd, -state_estimate.Y_dot);
	return 0;
}




/***************************************************************************
* Z Throttle/Altitude Controller
*
* If transitioning from direct throttle to altitude control, prefill the
* filter with current throttle input to make smooth transition. This is also
* true if taking off for the first time in altitude mode as arm_controller
* sets up last_en_Z_ctrl and last_usr_thr every time controller arms
***************************************************************************/
int feedback_controller_t::z_init(void)
{
	// filters
	D_Z_pd = RC_FILTER_INITIALIZER;
	D_Z_i = RC_FILTER_INITIALIZER;

	if (unlikely(rc_filter_duplicate(&D_Z_pd, settings.altitude_controller_pd) == -1))
	{
		printf("Error in z_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_Z_i, settings.altitude_controller_i) == -1))
	{
		printf("Error in z_init: failed to dublicate controller from settings\n");
		return -1;
	}


#ifdef DEBUG
	printf("ALTITUDE CONTROLLER:\n");
	rc_filter_print(D_Z);
#endif

	//save a copy of the original/default gain set:
	D_Z_pd_orig = D_Z_pd;
	D_Z_i_orig = D_Z_i;

	last_en_Z_ctrl = false;

	return 0;
}


int feedback_controller_t::z_march(void)
{
	// only the first step after altitude controll is on
	if (!last_en_Z_ctrl)
	{
		setpoint.reset_Z();
		setpoint.reset_Z_dot_ff();
		//take stick position as nominal hover thrust but leave enough room for manual adjustements for extreme cases
		if (settings.enable_mocap) {
			if (user_input.get_thr_stick() > 0.80) {
				setpoint.Z_throttle_0 = 0.80; //don't let hover thrust be too high (but account for heavy drones with T/W < 1.7)
			}
			else if (user_input.get_thr_stick() < 0.15) {
				setpoint.Z_throttle_0 = 0.15; //don't let hover thrust be too low if starting from the ground
			}
			else {
				setpoint.Z_throttle_0 = user_input.get_thr_stick(); //detect last user input
			}
		}

		
		setpoint.Z = state_estimate.Z; // set altitude setpoint to current altitude

		z_reset();   // reset the filter (works well so far, not need to prefill outputs)

		last_en_Z_ctrl = true;
	}

	if (settings.enable_v_gain_scaling)
	{
		// updating the gains based on battery voltage
		D_Z_pd.gain = D_Z_pd_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_Z_i.gain = D_Z_i_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
	}
	// Position error -> Velocity error:
	setpoint.Z_dot = rc_filter_march(&D_Z_pd, setpoint.Z - state_estimate.Z)
		+ rc_filter_march(&D_Z_i, setpoint.Z - state_estimate.Z) + setpoint.Z_dot_ff;
	
	rc_saturate_double(&setpoint.Z_dot, -MAX_Z_VELOCITY, MAX_Z_VELOCITY);
	if (setpoint.en_Z_rate_ctrl)
	{
		z_rate_march();
	}
	else
	{
		setpoint.Z_ddot = setpoint.Z_dot;
	}
	
	setpoint.Z_throttle = (setpoint.Z_ddot - setpoint.Z_throttle_0)\
		/ (cos(state_estimate.roll) * cos(state_estimate.pitch));

	last_en_Z_ctrl = true;
	return 0;
}

int feedback_controller_t::z_reset(void)
{
	D_Z_pd = D_Z_pd_orig;
	D_Z_i = D_Z_i_orig;

    rc_filter_reset(&D_Z_pd);
	rc_filter_reset(&D_Z_i);
    return 0;
}


/***************************************************************************
* Z Altitude Rate Controller
***************************************************************************/
int feedback_controller_t::z_rate_init(void)
{
	// filters
	D_Zdot_pd = RC_FILTER_INITIALIZER;
	D_Zdot_i = RC_FILTER_INITIALIZER;

	if (unlikely(rc_filter_duplicate(&D_Zdot_pd, settings.altitude_rate_controller_pd) == -1))
	{
		printf("Error in z_rate_init: failed to dublicate controller from settings\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&D_Zdot_i, settings.altitude_rate_controller_i) == -1))
	{
		printf("Error in z_rate_init: failed to dublicate controller from settings\n");
		return -1;
	}

	//save a copy of the original/default gain set:
	D_Zdot_pd_orig = D_Zdot_pd;
	D_Zdot_i_orig = D_Zdot_i;

	last_en_Zdot_ctrl = false;
	return 0;
}

int feedback_controller_t::z_rate_march(void)
{
	if (!last_en_Zdot_ctrl)
	{
		setpoint.reset_Z_dot();
		z_rate_reset();
		
		last_en_Zdot_ctrl = true;
	}

	if (settings.enable_v_gain_scaling)
	{
		// updating the gains based on battery voltage
		D_Zdot_pd.gain = D_Zdot_pd_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
		D_Zdot_i.gain = D_Zdot_i_orig.gain * settings.v_nominal / state_estimate.v_batt_lp;
	}

	// Vertical velocity error -> Vertical acceleration error
	setpoint.Z_ddot = rc_filter_march(&D_Zdot_pd, setpoint.Z_dot - state_estimate.Z_dot)
		+ rc_filter_march(&D_Zdot_i, setpoint.Z_dot - state_estimate.Z_dot);
	rc_saturate_double(&setpoint.Z_ddot, -MAX_Z_ACCELERATION, MAX_Z_ACCELERATION);

	last_en_Zdot_ctrl = true;
	return 0;
}

int feedback_controller_t::z_rate_reset(void)
{
	D_Zdot_pd = D_Zdot_pd_orig;
	D_Zdot_i = D_Zdot_i_orig;

	rc_filter_reset(&D_Zdot_pd);
	rc_filter_reset(&D_Zdot_i);

	rc_filter_prefill_inputs(&D_Zdot_pd, -state_estimate.Z_dot);
	return 0;
}


char feedback_controller_t::gain_tune_march(void)
{
	if (GS_RX.en_tunning)
	{
		received_gain_set.GainCH = GS_RX.GainCH;
		received_gain_set.GainN1_i = GS_RX.GainN1_i;
		received_gain_set.GainN0_pd = GS_RX.GainN0_pd;
		received_gain_set.GainN1_pd = GS_RX.GainN1_pd;
		received_gain_set.GainD1_pd = GS_RX.GainD1_pd;



		if (unlikely(update_gains() < 0))
		{
			printf("ERROR in gain_tune_march: failed to update gains\n");
			tune_status_fl = false;
			return -1;
		}
		tune_status_fl = true;
	}
	else
	{
		if (tune_status_fl) reset(); //switched out of tunning, revert back to def
		tune_status_fl = false;
	}


	return 0;
}

char feedback_controller_t::update_gains(void)
{
	/*
	Update gains using Ground station. Use GS_RX.GainCH for swithing
	between channels, assume 0 is the default mode of operation
	with whatever gains are already set.
	*/

	switch (received_gain_set.GainCH)
	{
	case 0:
		break;
	case 1: //roll
		D_roll_i.num.d[1] = received_gain_set.GainN1_i;
		D_roll_pd.num.d[0] = received_gain_set.GainN0_pd;
		D_roll_pd.num.d[1] = received_gain_set.GainN1_pd;
		D_roll_pd.den.d[1] = received_gain_set.GainD1_pd;
		break;

	case 2: //pitch
		D_pitch_i.num.d[1] = received_gain_set.GainN1_i;
		D_pitch_pd.num.d[0] = received_gain_set.GainN0_pd;
		D_pitch_pd.num.d[1] = received_gain_set.GainN1_pd;
		D_pitch_pd.den.d[1] = received_gain_set.GainD1_pd;
		break;

	case 3: //yaw
		D_yaw_i.num.d[1] = received_gain_set.GainN1_i;
		D_yaw_pd.num.d[0] = received_gain_set.GainN0_pd;
		D_yaw_pd.num.d[1] = received_gain_set.GainN1_pd;
		D_yaw_pd.den.d[1] = received_gain_set.GainD1_pd;
		break;

	case 4: //roll rate
		D_roll_rate_i.num.d[1] = received_gain_set.GainN1_i;
		D_roll_rate_pd.num.d[0] = received_gain_set.GainN0_pd;
		D_roll_rate_pd.num.d[1] = received_gain_set.GainN1_pd;
		D_roll_rate_pd.den.d[1] = received_gain_set.GainD1_pd;
		break;

	case 5: //pitch rate
		D_pitch_rate_i.num.d[1] = received_gain_set.GainN1_i;
		D_pitch_rate_pd.num.d[0] = received_gain_set.GainN0_pd;
		D_pitch_rate_pd.num.d[1] = received_gain_set.GainN1_pd;
		D_pitch_rate_pd.den.d[1] = received_gain_set.GainD1_pd;
		break;

	case 6: //yaw rate
		D_yaw_rate_i.num.d[1] = received_gain_set.GainN1_i;
		D_yaw_rate_pd.num.d[0] = received_gain_set.GainN0_pd;
		D_yaw_rate_pd.num.d[1] = received_gain_set.GainN1_pd;
		D_yaw_rate_pd.den.d[1] = received_gain_set.GainD1_pd;
		break;

	case 7: //x
		D_X_i.num.d[1] = received_gain_set.GainN1_i;
		D_X_pd.num.d[0] = received_gain_set.GainN0_pd;
		D_X_pd.num.d[1] = received_gain_set.GainN1_pd;
		D_X_pd.den.d[1] = received_gain_set.GainD1_pd;
		break;

	case 8: //y
		D_Y_i.num.d[1] = received_gain_set.GainN1_i;
		D_Y_pd.num.d[0] = received_gain_set.GainN0_pd;
		D_Y_pd.num.d[1] = received_gain_set.GainN1_pd;
		D_Y_pd.den.d[1] = received_gain_set.GainD1_pd;
		break;

	case 9: //z
		D_Z_i.num.d[1] = received_gain_set.GainN1_i;
		D_Z_pd.num.d[0] = received_gain_set.GainN0_pd;
		D_Z_pd.num.d[1] = received_gain_set.GainN1_pd;
		D_Z_pd.den.d[1] = received_gain_set.GainD1_pd;
		break;

	case 10: //x rate
		D_Xdot_i.num.d[1] = received_gain_set.GainN1_i;
		D_Xdot_pd.num.d[0] = received_gain_set.GainN0_pd;
		D_Xdot_pd.num.d[1] = received_gain_set.GainN1_pd;
		D_Xdot_pd.den.d[1] = received_gain_set.GainD1_pd;
		break;

	case 11: //y rate
		D_Ydot_i.num.d[1] = received_gain_set.GainN1_i;
		D_Ydot_pd.num.d[0] = received_gain_set.GainN0_pd;
		D_Ydot_pd.num.d[1] = received_gain_set.GainN1_pd;
		D_Ydot_pd.den.d[1] = received_gain_set.GainD1_pd;
		break;

	case 12: //z rate
		D_Zdot_i.num.d[1] = received_gain_set.GainN1_i;
		D_Zdot_pd.num.d[0] = received_gain_set.GainN0_pd;
		D_Zdot_pd.num.d[1] = received_gain_set.GainN1_pd;
		D_Zdot_pd.den.d[1] = received_gain_set.GainD1_pd;
		break;
	
	default: //no changes 
		printf("ERROR in update_gains: undefined gain channel\n");
		return -1;
	}

	return 0;
}


int feedback_controller_t::init(void)
{
    if (initialized)
    {
        printf("WARNING in init: feedback controller already initialized\n");
        return 0;
    }

	if (unlikely(rpy_init() == -1))
	{
		printf("Error in init: failed to initialize rpy controller\n");
		return -1;
	}
	if (unlikely(rpy_rate_init() == -1))
	{
		printf("Error in init: failed to initialize rpy_rate controller\n");
		return -1;
	}
	if (unlikely(z_init() == -1))
	{
		printf("Error in init: failed to initialize z controller\n");
		return -1;
	}
	if (unlikely(z_rate_init() == -1))
	{
		printf("Error in init: failed to initialize z_rate controller\n");
		return -1;
	}
	if (unlikely(xy_init() == -1))
	{
		printf("Error in init: failed to initialize xy controller\n");
		return -1;
	}
	if (unlikely(xy_rate_init() == -1))
	{
		printf("Error in init: failed to initialize xy_rate controller\n");
		return -1;
	}

    initialized = true;
    return 0;
}

int feedback_controller_t::mix_all_control(double(&u)[MAX_INPUTS], double(&mot)[MAX_ROTORS])
{
	if (unlikely(!initialized))
	{
		printf("ERROR in mix_all_control: feedback controller not initialized\n");
		return -1;
	}

	double min, max;

	/* 1. Throttle/Altitude Control */
	rc_saturate_double(&setpoint.Z_throttle, MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
	u[VEC_Z] = setpoint.Z_throttle;
	mix_add_input(u[VEC_Z], VEC_Z, mot);


	/* 2. Roll (X) Control */
	mix_check_saturation(VEC_ROLL, mot, &min, &max);
	if (max > MAX_ROLL_COMPONENT)  max = MAX_ROLL_COMPONENT;
	if (min < -MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
	u[VEC_ROLL] = setpoint.roll_throttle;
	rc_saturate_double(&u[VEC_ROLL], min, max);
	mix_add_input(u[VEC_ROLL], VEC_ROLL, mot);

	/* 2. Pitch (Y) Control */
	mix_check_saturation(VEC_PITCH, mot, &min, &max);
	if (max > MAX_PITCH_COMPONENT)  max = MAX_PITCH_COMPONENT;
	if (min < -MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
	u[VEC_PITCH] = setpoint.pitch_throttle;
	rc_saturate_double(&u[VEC_PITCH], min, max);
	mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);
	
	/* 3. Yaw (Z) Control */
	mix_check_saturation(VEC_YAW, mot, &min, &max);
	if (max > MAX_YAW_COMPONENT)  max = MAX_YAW_COMPONENT;
	if (min < -MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
	u[VEC_YAW] = setpoint.yaw_throttle;
	rc_saturate_double(&u[VEC_YAW], min, max);
	mix_add_input(u[VEC_YAW], VEC_YAW, mot);

	// for 6dof systems, add X and Y
	if (setpoint.en_6dof) {
		// X
		mix_check_saturation(VEC_X, mot, &min, &max);
		if (max > MAX_X_COMPONENT)  max = MAX_X_COMPONENT;
		if (min < -MAX_X_COMPONENT) min = -MAX_X_COMPONENT;
		u[VEC_X] = setpoint.X_throttle;
		rc_saturate_double(&u[VEC_X], min, max);
		mix_add_input(u[VEC_X], VEC_X, mot);

		// Y
		mix_check_saturation(VEC_Y, mot, &min, &max);
		if (max > MAX_Y_COMPONENT)  max = MAX_Y_COMPONENT;
		if (min < -MAX_Y_COMPONENT) min = -MAX_Y_COMPONENT;
		u[VEC_Y] = setpoint.Y_throttle;
		rc_saturate_double(&u[VEC_Y], min, max);
		mix_add_input(u[VEC_Y], VEC_Y, mot);
	}

	return 0;
}


int feedback_controller_t::march(double(&u)[MAX_INPUTS], double(&mot)[MAX_ROTORS])
{
    if (unlikely(!initialized))
    {
        printf("ERROR in reset: feedback controller not initialized\n");
        return -1;
    }


	/* Make sure controller flags are set to off when not used for proper
	switching between flight modes - we want to make sure controllers are being
	reset every time flight mode is switched.
	*/
	if (!setpoint.en_rpy_ctrl) last_en_rpy_ctrl = false;
	if (!setpoint.en_rpy_rate_ctrl) last_en_rpy_rate_ctrl = false;
	if (!setpoint.en_Z_ctrl) last_en_Z_ctrl = false;
	if (!setpoint.en_Z_rate_ctrl) last_en_Zdot_ctrl = false;
	if (!setpoint.en_XY_pos_ctrl) last_en_XY_ctrl = false;
	if (!setpoint.en_XY_vel_ctrl) last_en_XYdot_ctrl = false;
	
	// update gains if allowed
	if (settings.allow_remote_tuning) gain_tune_march();

	// run position controller if enabled
	if (setpoint.en_XY_pos_ctrl) 
	{
		xy_march(); //marches XY velocity controller as well
	}
	else if (setpoint.en_XY_vel_ctrl) //iff only using XY velocity ctrl
	{
		xy_rate_march();
	}
	
	// run altitude controller if enabled
	if (setpoint.en_Z_ctrl)
	{
		z_march(); //also marches vertical velocity controller
	}
	else if (setpoint.en_Z_rate_ctrl) //iff only using vertical velocity controller
	{
		z_rate_march();
	}

	// run attitude controllers if enabled
	if (setpoint.en_rpy_ctrl)
	{
		rpy_march(); //marches only attitude ctrl. + transition
	}
	else if (setpoint.en_rpy_trans)
	{
		rpy_transition(setpoint.roll_throttle, setpoint.pitch_throttle, setpoint.yaw_throttle);
	}

	// run attitude rate controllers if enabled
	if (setpoint.en_rpy_rate_ctrl)
	{
		rpy_rate_march(); //marches only attitude rates ctrl. + transition
	}

	// now use motor mixing matrix to get individual motor inputs in [0 1] range
	mix_all_control(u, mot);


	return 0;
}


int feedback_controller_t::reset(void)
{
    if (unlikely(!initialized))
    {
        printf("ERROR in reset: feedback controller not initialized\n");
        return -1;
    }

	rpy_reset();
	rpy_rate_reset();
	xy_reset();
	xy_rate_reset();
	z_reset();
	z_rate_reset();

    return 0;
}