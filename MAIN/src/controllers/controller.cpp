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
 * Last Edit:  08/13/2022 (MM/DD/YYYY)
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


/*returns sign of an argument*/
template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}


/* Brief: shortcut for 2D/circular normalized bound on setpoint (not square bound)*/
inline void __get_norm_sp_bounded_2D(double& new_x_sp, double& new_y_sp, \
	double x_sp, double y_sp, double x, double y, double max_norm)
{
	double tmp_x_err = x_sp - x;
	double tmp_y_err = y_sp - y;
	double tmp_norm = sqrt(tmp_x_err * tmp_x_err + tmp_y_err * tmp_y_err);
	double tmp_x_out, tmp_y_out;

	if (tmp_norm > max_norm)
	{
		new_x_sp = tmp_x_err / tmp_norm + x / max_norm; // (x_sp - x)/nm = x_err/nm  --> x_sp/nm = (x_err + x)/nm
		new_y_sp = tmp_y_err / tmp_norm + y / max_norm;
	}
	else
	{
		new_x_sp = tmp_x_err / max_norm + x / max_norm;
		new_y_sp = tmp_y_err / max_norm + y / max_norm;
	}
	return;
}

/* Brief: shortcut for 1D normalized bound on setpoint */
inline double __get_norm_sp_bounded_1D(double x, double x_sp, double max_err)
{
	double tmp_err = x_sp - x;
	if (tmp_err >= max_err) tmp_err = 1.0;
	else if (tmp_err <= -max_err) tmp_err = -1.0;
	else tmp_err = tmp_err / max_err;
	return tmp_err + x / max_err;
}




/***************************************************************************
* Roll Pitch Yaw controllers
***************************************************************************/
int feedback_controller_t::rpy_init(void)
{
    // get controllers from settings
	if (unlikely(roll.init(settings.roll_ctrl) == -1))
	{
		printf("Error in rpy_init: failed to create roll controller from settings\n");
		return -1;
	}
	if (unlikely(pitch.init(settings.pitch_ctrl) == -1))
	{
		printf("Error in rpy_init: failed to create pitch controller from settings\n");
		return -1;
	}
	if (unlikely(yaw.init(settings.yaw_ctrl) == -1))
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
		setpoint.ATT.reset();
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

	//setpoint.ATT.x.value.saturate(-MAX_ROLL_SETPOINT, MAX_ROLL_SETPOINT);
	//setpoint.ATT.y.value.saturate(-MAX_PITCH_SETPOINT, MAX_PITCH_SETPOINT);
	double tmp_yaw = state_estimate.continuous_yaw;
	

	//double err_roll, err_pitch, err_yaw;
	//err_roll = setpoint.ATT.x.value.get() - state_estimate.roll;
	//err_pitch = setpoint.ATT.y.value.get() - state_estimate.pitch;
	//err_yaw = setpoint.ATT.z.value.get() - state_estimate.continuous_yaw;
	//use smooth transition if control blending is enabled:
	//if (setpoint.en_rpy_trans) rpy_transition(err_roll, err_pitch, err_yaw); //zero out ff terms?

	// 1) Attitude -> Attitude Rate
	double tmp_roll_out, tmp_pitch_out, tmp_yaw_out;
	roll.march_std(tmp_roll_out, __get_norm_sp_bounded_1D(0.0, setpoint.ATT.x.value.get(), MAX_ROLL_SETPOINT), state_estimate.roll / MAX_ROLL_SETPOINT);
	pitch.march_std(tmp_pitch_out, __get_norm_sp_bounded_1D(0.0, setpoint.ATT.y.value.get(), MAX_PITCH_SETPOINT), state_estimate.pitch / MAX_PITCH_SETPOINT);
	yaw.march_std(tmp_yaw_out, __get_norm_sp_bounded_1D(tmp_yaw, setpoint.ATT.z.value.get(), MAX_YAW_ERROR), tmp_yaw / MAX_YAW_ERROR);
	/*
	if (setpoint.ATT.is_en_FF())
	{
		roll.march(setpoint.ATT_dot.x.value.get_pt(), err_roll, setpoint.ATT.x.FF.get());
		pitch.march(setpoint.ATT_dot.y.value.get_pt(), err_pitch, setpoint.ATT.y.FF.get());
		yaw.march(setpoint.ATT_dot.z.value.get_pt(), err_yaw, setpoint.ATT.z.FF.get());
	}
	else
	{
		roll.march(setpoint.ATT_dot.x.value.get_pt(), err_roll);
		pitch.march(setpoint.ATT_dot.y.value.get_pt(), err_pitch);
		yaw.march(setpoint.ATT_dot.z.value.get_pt(), err_yaw);
	}	
	*/

	if (setpoint.ATT_dot.is_en())
	{
		setpoint.ATT_dot.x.value.set(tmp_roll_out * MAX_ROLL_SETPOINT);
		setpoint.ATT_dot.y.value.set(tmp_pitch_out * MAX_PITCH_SETPOINT);
		setpoint.ATT_dot.z.value.set(tmp_yaw_out * MAX_YAW_ERROR);
	}
	else
	{		
		setpoint.ATT_throttle.x.value.set(tmp_roll_out * MAX_ROLL_SETPOINT);
		setpoint.ATT_throttle.y.value.set(tmp_pitch_out * MAX_PITCH_SETPOINT);
		setpoint.ATT_throttle.z.value.set(tmp_yaw_out * MAX_YAW_ERROR);
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
	//roll_err = roll_err * (tanh(7.0 + setpoint.roll_tr / 0.06) + 1.0) / 2.0;
	//pitch_err = pitch_err * (tanh(4.9 + setpoint.pitch_tr / 0.12) + 1.0) / 2.0;
	//yaw_err = yaw_err * (tanh(4.9 + setpoint.yaw_tr / 0.12) + 1.0) / 2.0;
	return 0;
}



/***************************************************************************
* Roll Pitch Yaw rate controllers
***************************************************************************/
int feedback_controller_t::rpy_rate_init(void)
{
	// get controllers from settings
	if (unlikely(roll_dot.init(settings.roll_rate_ctrl) == -1))
	{
		printf("Error in rpy_rate_init: failed to create roll rate controller from settings\n");
		return -1;
	}
	if (unlikely(pitch_dot.init(settings.pitch_rate_ctrl) == -1))
	{
		printf("Error in rpy_rate_init: failed to create pitch rate controller from settings\n");
		return -1;
	}
	if (unlikely(yaw_dot.init(settings.yaw_rate_ctrl) == -1))
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
		setpoint.ATT_dot.reset();
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

	setpoint.ATT_dot.x.value.saturate(-MAX_ROLL_RATE, MAX_ROLL_RATE);
	setpoint.ATT_dot.y.value.saturate(-MAX_PITCH_RATE, MAX_PITCH_RATE);
	setpoint.ATT_dot.z.value.saturate(-MAX_YAW_RATE, MAX_YAW_RATE);

	/*
	double err_roll_dot, err_pitch_dot, err_yaw_dot;
	err_roll_dot = setpoint.ATT_dot.x.value.get() - state_estimate.roll_dot;
	err_pitch_dot = setpoint.ATT_dot.y.value.get() - state_estimate.pitch_dot;
	err_yaw_dot = setpoint.ATT_dot.z.value.get() - state_estimate.yaw_dot;
	*/
	//printf("yaw_dot = %f\t yaw_dot_sp =%f\t err_yaw_dot = %f\n", state_estimate.yaw_dot, setpoint.ATT_dot.z.value.get(), err_yaw_dot);

	//use smooth transition if control blending is enabled:
	//if (setpoint.en_rpy_rate_trans) rpy_rate_transition(err_roll_dot, err_pitch_dot, err_yaw_dot);

	// Attitude rate error -> Torque cmd.
	double tmp_roll_out, tmp_pitch_out, tmp_yaw_out;
	roll_dot.march_std(tmp_roll_out, __get_norm_sp_bounded_1D(0.0, setpoint.ATT_dot.x.value.get(), MAX_ROLL_RATE), state_estimate.roll_dot / MAX_ROLL_RATE);
	pitch_dot.march_std(tmp_pitch_out, __get_norm_sp_bounded_1D(0.0, setpoint.ATT_dot.y.value.get(), MAX_PITCH_RATE), state_estimate.pitch_dot / MAX_PITCH_RATE);
	yaw_dot.march_std(tmp_yaw_out, __get_norm_sp_bounded_1D(0.0, setpoint.ATT_dot.z.value.get(), MAX_YAW_RATE), state_estimate.yaw_dot / MAX_YAW_RATE);
	
	/*
	if (setpoint.ATT_dot.is_en_FF())
	{
		roll_dot.march(setpoint.ATT_throttle.x.value.get_pt(), err_roll_dot, setpoint.ATT_dot.x.FF.get());
		pitch_dot.march(setpoint.ATT_throttle.y.value.get_pt(), err_pitch_dot, setpoint.ATT_dot.y.FF.get());
		yaw_dot.march(setpoint.ATT_throttle.z.value.get_pt(), err_yaw_dot, setpoint.ATT_dot.z.FF.get());
	}
	else
	{
		roll_dot.march(setpoint.ATT_throttle.x.value.get_pt(), err_roll_dot);
		pitch_dot.march(setpoint.ATT_throttle.y.value.get_pt(), err_pitch_dot);
		yaw_dot.march(setpoint.ATT_throttle.z.value.get_pt(), err_yaw_dot);
	}
	*/
	setpoint.ATT_throttle.x.value.set(tmp_roll_out * MAX_ROLL_RATE);
	setpoint.ATT_throttle.y.value.set(tmp_pitch_out * MAX_PITCH_RATE);
	setpoint.ATT_throttle.z.value.set(tmp_yaw_out * MAX_YAW_RATE);

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
	//roll_dot_err = roll_dot_err * (tanh(7.0 + setpoint.roll_dot_tr / 0.06) + 1.0) / 2.0;
	//pitch_dot_err = pitch_dot_err * (tanh(4.9 + setpoint.pitch_dot_tr / 0.12) + 1.0) / 2.0;
	//yaw_dot_err = yaw_dot_err * (tanh(4.9 + setpoint.yaw_dot_tr / 0.12) + 1.0) / 2.0;
	return 0;
}




/***************************************************************************
* X Y Position Controller
***************************************************************************/
int feedback_controller_t::xy_init(void)
{
	// get controllers from settings
	if (unlikely(x.init(settings.X_pos_ctrl) == -1))
	{
		printf("Error in xy_init: failed to create X position controller from settings\n");
		return -1;
	}
	if (unlikely(y.init(settings.Y_pos_ctrl) == -1))
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
		setpoint.XY.reset();
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
	/*
	double tmp_x_err = setpoint.XY.x.value.get() - state_estimate.X;
	double tmp_y_err = setpoint.XY.y.value.get() - state_estimate.Y;
	rc_saturate_double(&tmp_x_err, -XYZ_MAX_ERROR, XYZ_MAX_ERROR);
	rc_saturate_double(&tmp_y_err, -XYZ_MAX_ERROR, XYZ_MAX_ERROR);
	*/
	double tmp_x_sp, tmp_y_sp, tmp_x_out, tmp_y_out;
	double tmp_x = state_estimate.X;
	double tmp_y = state_estimate.Y;

	__get_norm_sp_bounded_2D(tmp_x_sp, tmp_y_sp,\
		setpoint.XY.x.value.get(), setpoint.XY.y.value.get(),\
		tmp_x, tmp_y, XY_MAX_ERROR_NORM);

	// Position error -> Velocity/Acceleration error
	x.march_std(tmp_x_out, tmp_x_sp, tmp_x / XY_MAX_ERROR_NORM);
	y.march_std(tmp_y_out, tmp_y_sp, tmp_y / XY_MAX_ERROR_NORM);
	
	/*
	if (setpoint.XY.is_en_FF())
	{
		x.march(setpoint.XY_dot.x.value.get_pt(), tmp_x_err, setpoint.XY.x.FF.get());
		y.march(setpoint.XY_dot.y.value.get_pt(), tmp_y_err, setpoint.XY.y.FF.get());
	}
	else
	{
		x.march(setpoint.XY_dot.x.value.get_pt(), tmp_x_err);
		y.march(setpoint.XY_dot.y.value.get_pt(), tmp_y_err);
	}
	*/
	
	if (setpoint.XY_dot.is_en())
	{
		setpoint.XY_dot.x.value.set(tmp_x_out * XY_MAX_ERROR_NORM); // rescale input x
		setpoint.XY_dot.y.value.set(tmp_y_out * XY_MAX_ERROR_NORM); // rescale input y
	}
	else
	{
		setpoint.XYZ_ddot.x.value.set(tmp_x_out * XY_MAX_ERROR_NORM); // rescale input x
		setpoint.XYZ_ddot.y.value.set(tmp_y_out * XY_MAX_ERROR_NORM); // rescale input y
	}
	


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
	if (unlikely(x_dot.init(settings.X_vel_ctrl) == -1))
	{
		printf("Error in xy_rate_init: failed to create X velocity controller from settings\n");
		return -1;
	}
	if (unlikely(y_dot.init(settings.Y_vel_ctrl) == -1))
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
		setpoint.XY_dot.reset();
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
	double tmp_x_sp, tmp_y_sp, tmp_x_out, tmp_y_out;
	double tmp_x = state_estimate.X_dot;
	double tmp_y = state_estimate.Y_dot;

	__get_norm_sp_bounded_2D(tmp_x_sp, tmp_y_sp, \
		setpoint.XY_dot.x.value.get(), setpoint.XY_dot.y.value.get(), \
		tmp_x, tmp_y, MAX_XY_VELOCITY_NORM);

	// Position error -> Velocity/Acceleration error
	x_dot.march_std(tmp_x_out, tmp_x_sp, tmp_x / MAX_XY_VELOCITY_NORM);
	y_dot.march_std(tmp_y_out, tmp_y_sp, tmp_y / MAX_XY_VELOCITY_NORM);

	setpoint.XYZ_ddot.x.value.set(tmp_x_out * MAX_XY_VELOCITY_NORM);
	setpoint.XYZ_ddot.y.value.set(tmp_y_out * MAX_XY_VELOCITY_NORM);


	/*
	setpoint.XY_dot.x.value.saturate(-MAX_XY_VELOCITY, MAX_XY_VELOCITY);
	setpoint.XY_dot.y.value.saturate(-MAX_XY_VELOCITY, MAX_XY_VELOCITY);

	if (setpoint.XY_dot.is_en_FF())
	{
		x_dot.march(setpoint.XYZ_ddot.x.value.get_pt(), setpoint.XY_dot.x.value.get() - state_estimate.X_dot, setpoint.XY_dot.x.FF.get());
		y_dot.march(setpoint.XYZ_ddot.y.value.get_pt(), setpoint.XY_dot.y.value.get() - state_estimate.Y_dot, setpoint.XY_dot.y.FF.get());
	}
	else
	{
		x_dot.march(setpoint.XYZ_ddot.x.value.get_pt(), setpoint.XY_dot.x.value.get() - state_estimate.X_dot);
		y_dot.march(setpoint.XYZ_ddot.y.value.get_pt(), setpoint.XY_dot.y.value.get() - state_estimate.Y_dot);
	}
	*/
	
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
	if (unlikely(z.init(settings.Z_pos_ctrl) == -1))
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
		setpoint.Z.reset();

		if (!last_en_Zdot_ctrl)
		{
			//take stick position as nominal hover thrust but leave enough room for manual adjustements for extreme cases
			if (user_input.throttle.get() > 0.80) {
				setpoint.Z_throttle_0 = 0.80; //don't let hover thrust be too high (but account for heavy drones with T/W < 1.7)
			}
			else if (user_input.throttle.get() < 0.15) {
				setpoint.Z_throttle_0 = 0.15; //don't let hover thrust be too low if starting from the ground
			}
			else {
				setpoint.Z_throttle_0 = user_input.throttle.get(); //detect last user input
			}
		}
		
		setpoint.Z.value.set(state_estimate.Z); // set altitude setpoint to current altitude

		z_reset();   // reset the filter (works well so far, not need to prefill outputs)

		last_en_Z_ctrl = true;
	}

	if (settings.enable_v_gain_scaling)
	{
		// updating the gains based on battery voltage
		z.scale_gains(settings.v_nominal / state_estimate.v_batt_lp);
	}

	//double tmp_z_err = setpoint.Z.value.get() - state_estimate.Z;
	//rc_saturate_double(&tmp_z_err, -XYZ_MAX_ERROR, XYZ_MAX_ERROR);

	// Position error -> Velocity error:
	double tmp_out;
	double tmp_z = state_estimate.Z;
	//z.march_std(&tmp_out, setpoint.Z.value.get(), tmp_z);

	z.march_std(tmp_out, __get_norm_sp_bounded_1D(tmp_z, setpoint.Z.value.get(), XYZ_MAX_ERROR), tmp_z / XYZ_MAX_ERROR);

	/*
	if (setpoint.Z.FF.is_en())
	{
		z.march(setpoint.Z_dot.value.get_pt(), tmp_z_err, setpoint.Z.FF.get());
	}
	else
	{
		z.march(setpoint.Z_dot.value.get_pt(), tmp_z_err);
	}
	*/

	if (setpoint.Z_dot.value.is_en())
	{
		setpoint.Z_dot.value.set(tmp_out * XYZ_MAX_ERROR);
	}
	else
	{
		setpoint.XYZ_ddot.z.value.set(tmp_out * XYZ_MAX_ERROR);
	}	
	
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
	if (unlikely(z_dot.init(settings.Z_vel_ctrl) == -1))
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
		setpoint.Z_dot.reset();
		z_rate_reset();

		if (!last_en_Z_ctrl)
		{
			//take stick position as nominal hover thrust but leave enough room for manual adjustements for extreme cases
			if (user_input.throttle.get() > 0.80) {
				setpoint.Z_throttle_0 = 0.80; //don't let hover thrust be too high (but account for heavy drones with T/W < 1.7)
			}
			else if (user_input.throttle.get() < 0.15) {
				setpoint.Z_throttle_0 = 0.15; //don't let hover thrust be too low if starting from the ground
			}
			else {
				setpoint.Z_throttle_0 = user_input.throttle.get(); //detect last user input
			}
		}
		
		last_en_Zdot_ctrl = true;
	}

	if (settings.enable_v_gain_scaling)
	{
		// updating the gains based on battery voltage
		z_dot.scale_gains(settings.v_nominal / state_estimate.v_batt_lp);
	}
	
	double tmp_z = state_estimate.Z_dot;
	double tmp_out;

	z_dot.march_std(tmp_out, __get_norm_sp_bounded_1D(tmp_z, setpoint.Z_dot.value.get(), MAX_Z_VELOCITY), tmp_z / MAX_Z_VELOCITY);
	setpoint.XYZ_ddot.z.value.set(tmp_out * MAX_Z_VELOCITY);

	/*
	setpoint.Z_dot.value.saturate(-MAX_Z_VELOCITY, MAX_Z_VELOCITY);

	// Vertical velocity error -> Vertical acceleration error
	if (setpoint.Z_dot.FF.is_en())
	{
		z_dot.march(setpoint.XYZ_ddot.z.value.get_pt(), setpoint.Z_dot.value.get() - state_estimate.Z_dot, setpoint.Z_dot.FF.get());
	}
	else
	{
		z_dot.march(setpoint.XYZ_ddot.z.value.get_pt(), setpoint.Z_dot.value.get() - state_estimate.Z_dot);
	}
	*/

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
		received_gain_set.GainK = GS_RX.GainK;



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

int feedback_controller_t::XY_accel_2_attitude(void)
{
	double tmp_x = setpoint.XYZ_ddot.x.value.get();
	double tmp_y = setpoint.XYZ_ddot.y.value.get();
	double tmp_norm = sqrt(tmp_x * tmp_x + tmp_y * tmp_y);
	if (tmp_norm >= MAX_XY_ACCELERATION_NORM)
	{
		tmp_x = tmp_x / tmp_norm;
		tmp_y = tmp_y / tmp_norm;
	}
	else
	{
		tmp_x = tmp_x / MAX_XY_ACCELERATION_NORM;
		tmp_y = tmp_y / MAX_XY_ACCELERATION_NORM;
	}

	//setpoint.XYZ_ddot.x.value.saturate(-MAX_XY_ACCELERATION, MAX_XY_ACCELERATION);
	//setpoint.XYZ_ddot.y.value.saturate(-MAX_XY_ACCELERATION, MAX_XY_ACCELERATION);

	// Horizonal acceleration setpoint -> Lean Angles
	setpoint.ATT.x.value.set(\
		-sin(state_estimate.continuous_yaw) * tmp_x\
			+ cos(state_estimate.continuous_yaw) * tmp_y);
	setpoint.ATT.y.set(\
		- cos(state_estimate.continuous_yaw) * tmp_x\
			+ sin(state_estimate.continuous_yaw) * tmp_y);
	/*
	setpoint.ATT.x.value.set(\
		(-sin(state_estimate.continuous_yaw) * setpoint.XYZ_ddot.x.value.get()\
			+ cos(state_estimate.continuous_yaw) * setpoint.XYZ_ddot.y.value.get()) / GRAVITY);
	setpoint.ATT.y.set(\
		- (cos(state_estimate.continuous_yaw) * setpoint.XYZ_ddot.x.value.get()\
			+ sin(state_estimate.continuous_yaw) * setpoint.XYZ_ddot.y.value.get()) / GRAVITY);
	*/
	return 0;
}

int feedback_controller_t::Z_accel_2_throttle(void)
{
	setpoint.XYZ_ddot.z.value.saturate(-MAX_Z_ACCELERATION, MAX_Z_ACCELERATION);

	// Vertical acceleration error -> throttle	
	setpoint.POS_throttle.z.value.set((setpoint.XYZ_ddot.z.value.get() - setpoint.Z_throttle_0)\
		/ (cos(state_estimate.roll) * cos(state_estimate.pitch)));

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
		return 0;
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

	if (setpoint.POS_throttle.z.value.is_en())
	{
		/* 1. Throttle/Altitude Control */
		setpoint.POS_throttle.z.value.saturate(MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
		u[VEC_Z] = setpoint.POS_throttle.z.value.get();
		mix_add_input(u[VEC_Z], VEC_Z, mot);
	}

	if (setpoint.ATT_throttle.x.value.is_en())
	{
		/* 2. Roll (X) Control */
		mix_check_saturation(VEC_ROLL, mot, &min, &max);
		if (max > MAX_ROLL_COMPONENT)  max = MAX_ROLL_COMPONENT;
		if (min < -MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
		u[VEC_ROLL] = setpoint.ATT_throttle.x.value.get();
		rc_saturate_double(&u[VEC_ROLL], min, max);
		mix_add_input(u[VEC_ROLL], VEC_ROLL, mot);
	}

	if (setpoint.ATT_throttle.y.value.is_en())
	{
		/* 2. Pitch (Y) Control */
		mix_check_saturation(VEC_PITCH, mot, &min, &max);
		if (max > MAX_PITCH_COMPONENT)  max = MAX_PITCH_COMPONENT;
		if (min < -MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
		u[VEC_PITCH] = setpoint.ATT_throttle.y.value.get();
		rc_saturate_double(&u[VEC_PITCH], min, max);
		mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);
	}
	
	if (setpoint.ATT_throttle.z.value.is_en())
	{
		/* 3. Yaw (Z) Control */
		mix_check_saturation(VEC_YAW, mot, &min, &max);
		if (max > MAX_YAW_COMPONENT)  max = MAX_YAW_COMPONENT;
		if (min < -MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
		u[VEC_YAW] = setpoint.ATT_throttle.z.value.get();
		rc_saturate_double(&u[VEC_YAW], min, max);
		mix_add_input(u[VEC_YAW], VEC_YAW, mot);
	}

	// for 6dof systems, add X and Y
	if (setpoint.POS_throttle.x.value.is_en()) 
	{
		// X
		mix_check_saturation(VEC_X, mot, &min, &max);
		if (max > MAX_X_COMPONENT)  max = MAX_X_COMPONENT;
		if (min < -MAX_X_COMPONENT) min = -MAX_X_COMPONENT;
		u[VEC_X] = setpoint.POS_throttle.x.value.get();
		rc_saturate_double(&u[VEC_X], min, max);
		mix_add_input(u[VEC_X], VEC_X, mot);
	}
	if (setpoint.POS_throttle.y.value.is_en())
	{
		// Y
		mix_check_saturation(VEC_Y, mot, &min, &max);
		if (max > MAX_Y_COMPONENT)  max = MAX_Y_COMPONENT;
		if (min < -MAX_Y_COMPONENT) min = -MAX_Y_COMPONENT;
		u[VEC_Y] = setpoint.POS_throttle.y.value.get();
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
	if (!setpoint.ATT.is_en()) last_en_rpy_ctrl = false;
	if (!setpoint.ATT_dot.is_en()) last_en_rpy_rate_ctrl = false;
	if (!setpoint.Z.value.is_en()) last_en_Z_ctrl = false;
	if (!setpoint.Z_dot.value.is_en()) last_en_Zdot_ctrl = false;
	if (!setpoint.XY.is_en()) last_en_XY_ctrl = false;
	if (!setpoint.XY_dot.is_en()) last_en_XYdot_ctrl = false;
	
	// update gains if allowed
	if (settings.allow_remote_tuning)
	{
		if (unlikely(gain_tune_march() < 0))
		{
			printf("ERROR in march: failed to march gain tunning\n");
			settings.allow_remote_tuning = false;
			reset();
		}
	}
		

	// run position controller if enabled
	if (setpoint.XY.is_en())
	{
		if (unlikely(xy_march() < 0))
		{
			printf("ERROR in march: failed to march horizontal position control\n");
			return -1;
		}
	}
	
	// run velocity controller if enabled
	if (setpoint.XY_dot.is_en())
	{
		if (unlikely(xy_rate_march() < 0))
		{
			printf("ERROR in march: failed to march horizontal velocity control\n");
			return -1;
		}
	}

	// check if we have any of the above enabled. If so, convert acceleration setpoints into attitude
	if (setpoint.XY.is_en() || setpoint.XY_dot.is_en())
	{
		if (unlikely(XY_accel_2_attitude() < 0))
		{
			printf("ERROR in march: failed to convert acceleration into attitude setpoints\n");
			return -1;
		}
	}
	
	// run altitude controller if enabled
	if (setpoint.Z.value.is_en())
	{
		if (unlikely(z_march() < 0))
		{
			printf("ERROR in march: failed to march altitude control\n");
			return -1;
		}
	}
	
	// run vertical velocity controller if enabled
	if (setpoint.Z_dot.value.is_en())
	{
		if (unlikely(z_rate_march() < 0))
		{
			printf("ERROR in march: failed to march vertical velocity control\n");
			return -1;
		}
	}

	// check if we have any of the above vertical controllers enabled. If so, convert acceleration setpoints into throttle
	if (setpoint.Z.value.is_en() || setpoint.Z_dot.value.is_en())
	{
		if (unlikely(Z_accel_2_throttle() < 0))
		{
			printf("ERROR in march: failed to convert vertical acceleration into throttle setpoints\n");
			return -1;
		}
	}

	// run attitude controllers if enabled
	if (setpoint.ATT.is_en())
	{
		if (unlikely(rpy_march() < 0))
		{
			printf("ERROR in march: failed to march attitude control\n");
			return -1;
		}
	}
	/*
	else if (setpoint.en_rpy_trans)
	{
		rpy_transition(setpoint.ATT_throttle.x.value.get(), setpoint.ATT_throttle.y.value.get(), setpoint.ATT_throttle.z.value.get());
	}
	*/

	// run attitude rate controllers if enabled
	if (setpoint.ATT_dot.is_en())
	{
		if (unlikely(rpy_rate_march() < 0))
		{
			printf("ERROR in march: failed to march attitude rate control\n");
			return -1;
		}
	}

	// now use motor mixing matrix to get individual motor inputs in [0 1] range
	if (unlikely(mix_all_control(u, mot) < 0))
	{
		printf("ERROR in march: failed to mix all control\n");
		return -1;
	}


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