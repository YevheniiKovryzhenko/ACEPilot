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
 * Last Edit:  05/27/2022 (MM/DD/YYYY)
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

/**
* @brief      Sets the default PD and I gains for the control system.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_t::set_default_gain_set(rc_filter_t &new_gain_pd, rc_filter_t &new_gain_i, double new_gain_FF)
{
	if (unlikely(rc_filter_duplicate(&def_gain_pd, new_gain_pd) == -1))
	{
		printf("Error in set_default_gain_set: failed to dublicate PD controller\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&def_gain_i, new_gain_i) == -1))
	{
		printf("Error in set_default_gain_set: failed to dublicate I controller\n");
		return -1;
	}

	def_gain_FF = gain_FF;

	return 0;
}

/**
* @brief      Sets the active PD and I gains for the control system.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_t::set_gain_set(rc_filter_t &new_gain_pd, rc_filter_t &new_gain_i, double new_gain_FF)
{
	if (unlikely(rc_filter_duplicate(&gain_pd, new_gain_pd) == -1))
	{
		printf("Error in set_gain_set: failed to dublicate PD controller\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&gain_i, new_gain_i) == -1))
	{
		printf("Error in set_gain_set: failed to dublicate I controller\n");
		return -1;
	}

	gain_FF = new_gain_FF;

	return 0;
}

/**
* @brief      Initializes the control system.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_t::init(rc_filter_t &new_gain_pd, rc_filter_t &new_gain_i, double new_gain_FF)
{
	if (unlikely(set_gain_set(new_gain_pd, new_gain_i, new_gain_FF) < 0))
	{
		printf("Error in init: failed to set controller gains\n");
		return -1;
	}
	if (unlikely(set_default_gain_set(new_gain_pd, new_gain_i, new_gain_FF) < 0))
	{
		printf("Error in init: failed to set default controller gains\n");
		return -1;
	}
	initialized = true;
	return 0;
}

/**
* @brief      Resets the control system to default gains, zeros out inputs/outputs, etc.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_t::reset(void)
{	
	gain_pd = def_gain_pd;
	gain_i = def_gain_i;
	gain_FF = def_gain_FF;

	if (unlikely(rc_filter_reset(&gain_pd) < 0))
	{
		printf("ERROR in reset: failed to reset PD control system\n");
		return -1;
	}
	if (unlikely(rc_filter_reset(&gain_i) < 0))
	{
		printf("ERROR in reset: failed to reset I control system\n");
		return -1;
	}

	return 0;
}

/**
* @brief      Marches the control system forward with new error and referece inputs.
*
* Must be initialized. Error input path goes though PID control system. Reference path is added
* directly to the output of the PID control system.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_t::march(double* out, double err_in, double ref_in)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in march: not initialized\n");
		return -1;
	}
	*out = rc_filter_march(&gain_pd, err_in)
		+ rc_filter_march(&gain_i, err_in) + gain_FF * ref_in;


	return 0;
}

/**
* @brief      Marches the control system forward with new error inputs.
*
* Must be initialized. Error input path goes though PID control system.
* Assumes no feedforward path. Out is the compined output of the system.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_t::march(double* out, double err_in)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in march: not initialized\n");
		return -1;
	}
	*out = rc_filter_march(&gain_pd, err_in)
		+ rc_filter_march(&gain_i, err_in);


	return 0;
}

/**
* @brief      Scales the control system gains using the input provided.
*
* Must be initialized. Input must be between 0 and 1.
* Scales the gain based on default gain * the input.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_t::scale_gains(double scale)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in scale_gains: not initialized\n");
		return -1;
	}
	if (unlikely(scale  < 0.0 || scale > 1.0))
	{
		printf("ERROR in scale_gains: input must be between 0.0 and 1.0\n");
		return -1;
	}

	gain_pd.gain = def_gain_pd.gain * scale;
	gain_i.gain = def_gain_i.gain * scale;
	gain_FF = def_gain_FF * scale;
	return 0;
}

/**
* @brief      Prefills PD control system input.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_t::prefill_pd_input(double in)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in prefill_pd_input: not initialized\n");
		return -1;
	}

	if (unlikely(rc_filter_prefill_inputs(&gain_pd, in)))
	{
		printf("ERROR: in prefill_pd_input: failed to prefill PD input\n");
		return -1;
	}

	return 0;
}

/**
* @brief      Prefills I control system input.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_t::prefill_i_input(double in)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in prefill_i_input: not initialized\n");
		return -1;
	}

	if (unlikely(rc_filter_prefill_inputs(&gain_i, in)))
	{
		printf("ERROR: in prefill_i_input: failed to prefill I input\n");
		return -1;
	}

	return 0;
}

/**
* @brief      Prefills PD control system output.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_t::prefill_pd_out(double out)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in prefill_pd_out: not initialized\n");
		return -1;
	}

	if (unlikely(rc_filter_prefill_inputs(&gain_pd, out)))
	{
		printf("ERROR: in prefill_pd_out: failed to prefill PD output\n");
		return -1;
	}

	return 0;
}

/**
* @brief      Prefills I control system output.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_t::prefill_i_out(double out)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in prefill_i_out: not initialized\n");
		return -1;
	}

	if (unlikely(rc_filter_prefill_inputs(&gain_i, out)))
	{
		printf("ERROR: in prefill_i_out: failed to prefill I output\n");
		return -1;
	}

	return 0;
}

/**
* @brief      Changes the active gains based on new input.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_t::set_tune_gains(PID_vars_set_t &new_input)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in set_tune_gains: not initialized\n");
		return -1;
	}

	gain_i.num.d[1] = new_input.GainN1_i;
	gain_pd.num.d[0] = new_input.GainN0_pd;
	gain_pd.num.d[1] = new_input.GainN1_pd;
	gain_pd.den.d[1] = new_input.GainD1_pd;
	gain_FF = new_input.GainFF;
	return 0;
}






/***************************************************************************
* Roll Pitch Yaw controllers
***************************************************************************/
int feedback_controller_t::rpy_init(void)
{
    // get controllers from settings
	if (unlikely(roll.init(settings.roll_controller_pd, settings.roll_controller_i, settings.roll_controller_FF) == -1))
	{
		printf("Error in rpy_init: failed to create roll controller from settings\n");
		return -1;
	}
	if (unlikely(pitch.init(settings.pitch_controller_pd, settings.pitch_controller_i, settings.pitch_controller_FF) == -1))
	{
		printf("Error in rpy_init: failed to create pitch controller from settings\n");
		return -1;
	}
	if (unlikely(yaw.init(settings.yaw_controller_pd, settings.yaw_controller_i, settings.yaw_controller_FF) == -1))
	{
		printf("Error in rpy_init: failed to create yaw controller from settings\n");
		return -1;
	}

	last_en_rpy_ctrl = false;
    return 0;
}

int feedback_controller_t::rpy_march(void)
{
	if (!last_en_rpy_ctrl)
	{
		setpoint.reset_att_all();		
		rpy_reset();

		last_en_rpy_ctrl = true;
	}

	if (settings.enable_v_gain_scaling)
	{
		// updating the gains based on battery voltage
		double scale_val = settings.v_nominal / state_estimate.v_batt_lp;
		roll.scale_gains(scale_val);
		pitch.scale_gains(scale_val);
		yaw.scale_gains(scale_val);
	}

	double err_roll, err_pitch, err_yaw;
	err_roll = setpoint.roll - state_estimate.roll;
	err_pitch = setpoint.pitch - state_estimate.pitch;
	err_yaw = setpoint.yaw - state_estimate.continuous_yaw;
	//use smooth transition if control blending is enabled:
	if (setpoint.en_rpy_trans) rpy_transition(err_roll, err_pitch, err_yaw); //zero out ff terms?

	// 1) Attitude -> Attitude Rate
	if (setpoint.is_en_rpy_FF())
	{
		roll.march(&setpoint.roll_dot, err_roll, setpoint.roll_ff);
		pitch.march(&setpoint.pitch_dot, err_pitch, setpoint.pitch_ff);
		yaw.march(&setpoint.yaw_dot, err_yaw, setpoint.yaw_ff);
	}
	else
	{
		roll.march(&setpoint.roll_dot, err_roll);
		pitch.march(&setpoint.pitch_dot, err_pitch);
		yaw.march(&setpoint.yaw_dot, err_yaw);

		setpoint.reset_att_ff();
	}

	

	if (!setpoint.is_en_rpy_rate_ctrl())
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
	roll.reset();
	pitch.reset();
	yaw.reset();	

    // prefill filters with current error (only those with D terms)
	roll.prefill_pd_input(-state_estimate.roll);
	pitch.prefill_pd_input(-state_estimate.pitch);
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
	// get controllers from settings
	if (unlikely(roll_dot.init(settings.roll_rate_controller_pd, settings.roll_rate_controller_i, settings.roll_rate_controller_FF) == -1))
	{
		printf("Error in rpy_rate_init: failed to create roll rate controller from settings\n");
		return -1;
	}
	if (unlikely(pitch_dot.init(settings.pitch_rate_controller_pd, settings.pitch_rate_controller_i, settings.pitch_rate_controller_FF) == -1))
	{
		printf("Error in rpy_rate_init: failed to create pitch rate controller from settings\n");
		return -1;
	}
	if (unlikely(yaw_dot.init(settings.yaw_rate_controller_pd, settings.yaw_rate_controller_i, settings.yaw_rate_controller_FF) == -1))
	{
		printf("Error in rpy_rate_init: failed to create yaw rate controller from settings\n");
		return -1;
	}

	last_en_rpy_rate_ctrl = false;

	return 0;
}

int feedback_controller_t::rpy_rate_march(void)
{
	if (!last_en_rpy_rate_ctrl)
	{
		setpoint.reset_att_dot_all();
		rpy_rate_reset();

		last_en_rpy_rate_ctrl = true;
	}

	if (settings.enable_v_gain_scaling)
	{
		// updating the gains based on battery voltage
		double scale_val = settings.v_nominal / state_estimate.v_batt_lp;
		roll_dot.scale_gains(scale_val);
		pitch_dot.scale_gains(scale_val);
		yaw_dot.scale_gains(scale_val);
	}

	double err_roll_dot, err_pitch_dot, err_yaw_dot;
	err_roll_dot = setpoint.roll_dot - state_estimate.roll_dot;
	err_pitch_dot = setpoint.pitch_dot - state_estimate.pitch_dot;
	err_yaw_dot = setpoint.yaw_dot - state_estimate.yaw_dot;

	//use smooth transition if control blending is enabled:
	if (setpoint.en_rpy_rate_trans) rpy_rate_transition(err_roll_dot, err_pitch_dot, err_yaw_dot);

	// Attitude rate error -> Torque cmd.
	if (setpoint.is_en_rpy_rate_FF())
	{
		roll_dot.march(&setpoint.roll_throttle, err_roll_dot, setpoint.roll_dot_ff);
		pitch_dot.march(&setpoint.pitch_throttle, err_pitch_dot, setpoint.pitch_dot_ff);
		yaw_dot.march(&setpoint.yaw_throttle, err_yaw_dot, setpoint.yaw_dot_ff);
	}
	else
	{
		roll_dot.march(&setpoint.roll_throttle, err_roll_dot);
		pitch_dot.march(&setpoint.pitch_throttle, err_pitch_dot);
		yaw_dot.march(&setpoint.yaw_throttle, err_yaw_dot);

		setpoint.reset_att_dot_ff();
	}

	last_en_rpy_rate_ctrl = true;
	return 0;
}

int feedback_controller_t::rpy_rate_reset(void)
{
	roll_dot.reset();
	pitch_dot.reset();
	yaw_dot.reset();

	// prefill filters with current error (only those with D terms)
	roll_dot.prefill_pd_input(-state_estimate.roll_dot);
	pitch_dot.prefill_pd_input(-state_estimate.pitch_dot);
	yaw_dot.prefill_pd_input(-state_estimate.yaw_dot);
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
	// get controllers from settings
	if (unlikely(x.init(settings.horiz_pos_ctrl_X_pd, settings.horiz_pos_ctrl_X_i, settings.horiz_pos_X_ctrl_FF) == -1))
	{
		printf("Error in xy_init: failed to create X position controller from settings\n");
		return -1;
	}
	if (unlikely(y.init(settings.horiz_pos_ctrl_Y_pd, settings.horiz_pos_ctrl_Y_i, settings.horiz_pos_Y_ctrl_FF) == -1))
	{
		printf("Error in xy_init: failed to create Y position controller from settings\n");
		return -1;
	}

	last_en_XY_ctrl = false;

    return 0;
}


int feedback_controller_t::xy_march(void)
{
	if (!last_en_XY_ctrl)
	{
		setpoint.reset_X();
		setpoint.reset_Y();
		setpoint.reset_X_ff();
		setpoint.reset_Y_ff();

		xy_reset();

		last_en_XY_ctrl = true;
	}

	////////////// PID for horizontal positon control /////////////
	if (settings.enable_v_gain_scaling)
	{
		// updating the gains based on battery voltage
		double scale_val = settings.v_nominal / state_estimate.v_batt_lp;
		x.scale_gains(scale_val);
		y.scale_gains(scale_val);
	}

	// Position error -> Velocity/Acceleration error
	if (setpoint.is_en_XY_pos_FF())
	{
		x.march(&setpoint.X_dot, setpoint.X - state_estimate.X, setpoint.X_ff);
		y.march(&setpoint.Y_dot, setpoint.Y - state_estimate.Y, setpoint.Y_ff);
	}
	else
	{
		x.march(&setpoint.X_dot, setpoint.X - state_estimate.X);
		y.march(&setpoint.Y_dot, setpoint.Y - state_estimate.Y);

		setpoint.reset_X_ff();
		setpoint.reset_Y_ff();
	}
	
	rc_saturate_double(&setpoint.X_dot, -MAX_XY_VELOCITY, MAX_XY_VELOCITY);
	rc_saturate_double(&setpoint.Y_dot, -MAX_XY_VELOCITY, MAX_XY_VELOCITY);
	
	if (setpoint.is_en_XY_vel_ctrl())
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
		+ cos(state_estimate.continuous_yaw) * setpoint.Y_ddot) / GRAVITY;
	setpoint.pitch = -(cos(state_estimate.continuous_yaw) * setpoint.X_ddot\
		+ sin(state_estimate.continuous_yaw) * setpoint.Y_ddot) / GRAVITY;

	rc_saturate_double(&setpoint.roll, -MAX_ROLL_SETPOINT, MAX_ROLL_SETPOINT);
	rc_saturate_double(&setpoint.pitch, -MAX_PITCH_SETPOINT, MAX_PITCH_SETPOINT);

	last_en_XY_ctrl = true;
	return 0;
}

int feedback_controller_t::xy_reset(void)
{
	x.reset();
	y.reset();
	return 0;
}



/***************************************************************************
* X Y Velocity Controller
***************************************************************************/
int feedback_controller_t::xy_rate_init(void)
{
	// get controllers from settings
	if (unlikely(x_dot.init(settings.horiz_vel_ctrl_pd_X, settings.horiz_vel_ctrl_i_X, settings.horiz_vel_X_controller_FF) == -1))
	{
		printf("Error in xy_rate_init: failed to create X velocity controller from settings\n");
		return -1;
	}
	if (unlikely(y_dot.init(settings.horiz_vel_ctrl_pd_Y, settings.horiz_vel_ctrl_i_Y, settings.horiz_vel_Y_controller_FF) == -1))
	{
		printf("Error in xy_rate_init: failed to create Y velocity controller from settings\n");
		return -1;
	}

	last_en_XYdot_ctrl = false;
	return 0;
}

int feedback_controller_t::xy_rate_march(void)
{
	if (!last_en_XYdot_ctrl)
	{
		setpoint.reset_X_dot();
		setpoint.reset_Y_dot();
		setpoint.reset_X_dot_ff();
		setpoint.reset_Y_dot_ff();
		
		xy_rate_reset();

		last_en_XYdot_ctrl = true;
	}

	if (settings.enable_v_gain_scaling)
	{
		// updating the gains based on battery voltage
		double scale_val = settings.v_nominal / state_estimate.v_batt_lp;
		x_dot.scale_gains(scale_val);
		y_dot.scale_gains(scale_val);
	}

	////////////// PID for horizontal velocity control /////////////
	if (setpoint.is_en_XY_vel_FF())
	{
		x_dot.march(&setpoint.X_ddot, setpoint.X_dot - state_estimate.X_dot, setpoint.X_dot_ff);
		y_dot.march(&setpoint.Y_ddot, setpoint.X_dot - state_estimate.X_dot, setpoint.Y_dot_ff);
	}
	else
	{
		x_dot.march(&setpoint.X_ddot, setpoint.X_dot - state_estimate.X_dot);
		y_dot.march(&setpoint.Y_ddot, setpoint.X_dot - state_estimate.X_dot);

		setpoint.reset_X_dot_ff();
		setpoint.reset_Y_dot_ff();
	}

	rc_saturate_double(&setpoint.X_ddot, -MAX_XY_ACCELERATION, MAX_XY_ACCELERATION);
	rc_saturate_double(&setpoint.Y_ddot, -MAX_XY_ACCELERATION, MAX_XY_ACCELERATION);
	
	last_en_XYdot_ctrl = true;
	return 0;
}

int feedback_controller_t::xy_rate_reset(void)
{
	x_dot.reset();
	y_dot.reset();

	// prefill filters with current error (only those with D terms)
	x_dot.prefill_pd_input(-state_estimate.X_dot);
	y_dot.prefill_pd_input(-state_estimate.Y_dot);
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
	// get controllers from settings
	if (unlikely(z.init(settings.altitude_controller_pd, settings.altitude_controller_i, settings.altitude_controller_FF) == -1))
	{
		printf("Error in z_rate_init: failed to create Altitude controller from settings\n");
		return -1;
	}

	last_en_Z_ctrl = false;

	return 0;
}


int feedback_controller_t::z_march(void)
{
	// only the first step after altitude controll is on
	if (!last_en_Z_ctrl)
	{
		setpoint.reset_Z();
		setpoint.reset_Z_ff();
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
		z.scale_gains(settings.v_nominal / state_estimate.v_batt_lp);
	}

	// Position error -> Velocity error:
	if (setpoint.is_en_Z_rate_FF())
	{
		z.march(&setpoint.Z_dot, setpoint.Z - state_estimate.Z, setpoint.Z_ff);
	}
	else
	{
		z.march(&setpoint.Z_dot, setpoint.Z - state_estimate.Z);

		setpoint.reset_Z_ff();
	}
	
	rc_saturate_double(&setpoint.Z_dot, -MAX_Z_VELOCITY, MAX_Z_VELOCITY);

	if (setpoint.is_en_Z_rate_ctrl())
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
	z.reset();
    return 0;
}


/***************************************************************************
* Z Altitude Rate Controller
***************************************************************************/
int feedback_controller_t::z_rate_init(void)
{
	// get controllers from settings
	if (unlikely(z_dot.init(settings.altitude_rate_controller_pd, settings.altitude_rate_controller_i, settings.altitude_rate_controller_FF) == -1))
	{
		printf("Error in z_rate_init: failed to create Altitude rate controller from settings\n");
		return -1;
	}

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
		z_dot.scale_gains(settings.v_nominal / state_estimate.v_batt_lp);
	}

	// Vertical velocity error -> Vertical acceleration error
	if (setpoint.is_en_Z_rate_FF())
	{
		z_dot.march(&setpoint.Z_ddot, setpoint.Z_dot - state_estimate.Z_dot, setpoint.Z_dot_ff);
	}
	else
	{
		z_dot.march(&setpoint.Z_ddot, setpoint.Z_dot - state_estimate.Z_dot);

		setpoint.reset_Z_dot_ff();
	}

	rc_saturate_double(&setpoint.Z_ddot, -MAX_Z_ACCELERATION, MAX_Z_ACCELERATION);

	last_en_Zdot_ctrl = true;
	return 0;
}

int feedback_controller_t::z_rate_reset(void)
{
	z_dot.reset();

	// prefill filters with current error (only those with D terms)
	z_dot.prefill_pd_input(-state_estimate.Z_dot);
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
		received_gain_set.GainFF = GS_RX.GainFF;



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
		roll.set_tune_gains(received_gain_set);
		break;

	case 2: //pitch
		pitch.set_tune_gains(received_gain_set);
		break;

	case 3: //yaw
		yaw.set_tune_gains(received_gain_set);
		break;

	case 4: //roll rate
		roll_dot.set_tune_gains(received_gain_set);
		break;

	case 5: //pitch rate
		pitch_dot.set_tune_gains(received_gain_set);
		break;

	case 6: //yaw rate
		yaw_dot.set_tune_gains(received_gain_set);
		break;

	case 7: //x
		x.set_tune_gains(received_gain_set);
		break;

	case 8: //y
		y.set_tune_gains(received_gain_set);
		break;

	case 9: //z
		z.set_tune_gains(received_gain_set);
		break;

	case 10: //x rate
		x_dot.set_tune_gains(received_gain_set);
		break;

	case 11: //y rate
		y_dot.set_tune_gains(received_gain_set);
		break;

	case 12: //z rate
		z_dot.set_tune_gains(received_gain_set);
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
	if (!setpoint.is_en_rpy_ctrl()) last_en_rpy_ctrl = false;
	if (!setpoint.is_en_rpy_rate_ctrl()) last_en_rpy_rate_ctrl = false;
	if (!setpoint.is_en_Z_ctrl()) last_en_Z_ctrl = false;
	if (!setpoint.is_en_Z_rate_ctrl()) last_en_Zdot_ctrl = false;
	if (!setpoint.is_en_XY_pos_ctrl()) last_en_XY_ctrl = false;
	if (!setpoint.is_en_XY_vel_ctrl()) last_en_XYdot_ctrl = false;
	
	// update gains if allowed
	if (settings.allow_remote_tuning) gain_tune_march();

	// run position controller if enabled
	if (setpoint.is_en_XY_pos_ctrl())
	{
		xy_march(); //marches XY velocity controller as well
	}
	else if (setpoint.is_en_XY_vel_ctrl()) //iff only using XY velocity ctrl
	{
		xy_rate_march();
	}
	
	// run altitude controller if enabled
	if (setpoint.is_en_Z_ctrl())
	{
		z_march(); //also marches vertical velocity controller
	}
	else if (setpoint.is_en_Z_rate_ctrl()) //iff only using vertical velocity controller
	{
		z_rate_march();
	}

	// run attitude controllers if enabled
	if (setpoint.is_en_rpy_ctrl())
	{
		rpy_march(); //marches only attitude ctrl. + transition
	}
	else if (setpoint.en_rpy_trans)
	{
		rpy_transition(setpoint.roll_throttle, setpoint.pitch_throttle, setpoint.yaw_throttle);
	}

	// run attitude rate controllers if enabled
	if (setpoint.is_en_rpy_rate_ctrl())
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