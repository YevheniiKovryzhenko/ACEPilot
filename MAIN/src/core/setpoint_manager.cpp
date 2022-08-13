/*
 * setpoint_manager.cpp
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
 *
 * Summary :
 * Setpoint manager runs at the same rate as the feedback controller
 * and is the interface between the user inputs (input manager) and
 * the feedback controller setpoint . currently it contains very
 * simply logic and runs very quickly which is why it's okay to run
 * in the feedback ISR right before the feedback controller. In the
 * future this is where go-home and other higher level autonomy will
 * live.
 *
 * This serves to allow the feedback controller to be as simple and
 * clean as possible by putting all high-level manipulation of the
 * setpoints here. Then feedback-controller only needs to march the
 * filters and zero them out when arming or enabling controllers
 *
 */

#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h> // for memset
#include <stdint.h> // for uint64_t
#include <inttypes.h> // for PRIu64

#include <rc/time.h> // for nanos
#include <rc/start_stop.h>
#include <rc/math/quaternion.h>

#include "rc_pilot_defs.h"
#include "input_manager.hpp"
#include "settings.h"
#include "state_estimator.h"
#include "flight_mode.h"
//#include "xbee_receive.h"
#include "setpoint_guidance.hpp"
#include "tools.h"
#include "feedback.hpp"
#include "state_machine.hpp"

#include "setpoint_manager.hpp"
#include "benchmark.h"
// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)



/**
* float apply_deadzone(float in, float zone)
*
* Applies a dead zone to an input stick in. in is supposed to range from -1 to 1
* the dead zone is centered around 0. zone specifies the distance from 0 the
* zone extends.
**/
inline double __deadzone(double in, double zone)
{
	if (zone <= 0.0) return in;
	if (fabs(in) <= zone) return 0.0;
	if (in > 0.0)	return ((in - zone) / (1.0 - zone));
	else		return ((in + zone) / (1.0 - zone));
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

 /*
 * Invoke defaut constructor for all the built in and exernal types in the class
 */
setpoint_t setpoint{}; // extern variable in setpoint_manager.hpp

/***********************************/
int setpoint_t::init_all_filters(void)
{
	if (unlikely(POS_throttle.init_filters() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize POS_throttle setpoints filters\n");
		return -1;
	}
	if (unlikely(ATT_throttle.init_filters() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize ATT_throttle setpoints filters\n");
		return -1;
	}
	if (unlikely(ATT_dot.init_filters() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize ATT_dot setpoints filters\n");
		return -1;
	}
	if (unlikely(ATT.init_filters() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize ATT setpoints filters\n");
		return -1;
	}
	if (unlikely(XYZ_ddot.init_filters() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize XYZ_ddot setpoints filters\n");
		return -1;
	}
	if (unlikely(Z.init_filters() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize Z setpoints filters\n");
		return -1;
	}
	if (unlikely(Z_dot.init_filters() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize Z_dot setpoints filters\n");
		return -1;
	}
	if (unlikely(XY_dot.init_filters() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize XY_dot setpoints filters\n");
		return -1;
	}
	if (unlikely(XY.init_filters() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize XY setpoints filters\n");
		return -1;
	}
	return 0;
}

/*
int setpoint_t::init_trans(void)
{
	trans_stick_int = RC_FILTER_INITIALIZER;
	if (unlikely(rc_filter_integrator(&trans_stick_int, DT) == -1))
	{
		printf("ERROR in init_trans: failed to create transition stick integrator\n");
		return -1;
	}
	rc_filter_enable_saturation(&trans_stick_int, -1.0, 1.0);

	last_en_trans = false;
	return 0;
}

int setpoint_t::init_stick_trim(void)
{
	roll_stick_int = RC_FILTER_INITIALIZER;
	pitch_stick_int = RC_FILTER_INITIALIZER;
	yaw_stick_int = RC_FILTER_INITIALIZER;

	if (unlikely(rc_filter_integrator(&roll_stick_int, DT) == -1))
	{
		printf("ERROR in init_stick_trim: failed to create roll stick trim integrator\n");
		return -1;
	}

	if (unlikely(rc_filter_integrator(&pitch_stick_int, DT) == -1))
	{
		printf("ERROR in init_stick_trim: failed to create pitch stick trim integrator\n");
		return -1;
	}

	if (unlikely(rc_filter_integrator(&yaw_stick_int, DT) == -1))
	{
		printf("ERROR in init_stick_trim: failed to create yaw stick trim integrator\n");
		return -1;
	}

	rc_filter_enable_saturation(&roll_stick_int, -1.0, 1.0);
	rc_filter_enable_saturation(&pitch_stick_int, -1.0, 1.0);
	rc_filter_enable_saturation(&yaw_stick_int, -1.0, 1.0);

	roll_stick_trim = 0.0;
	pitch_stick_trim = 0.0;
	yaw_stick_trim = 0.0;
	last_en_stick_trim = false;
	return 0;
}


int setpoint_t::init_stick_filter(void)
{
	roll_stick_lpf = RC_FILTER_INITIALIZER;
	pitch_stick_lpf = RC_FILTER_INITIALIZER;
	yaw_stick_lpf = RC_FILTER_INITIALIZER;
	roll_stick_hpf = RC_FILTER_INITIALIZER;
	pitch_stick_hpf = RC_FILTER_INITIALIZER;
	yaw_stick_hpf = RC_FILTER_INITIALIZER;

	if (unlikely(rc_filter_first_order_lowpass(&roll_stick_lpf, DT, 0.1 * DT) == -1))
	{
		printf("ERROR in init_stick_filter: failed to create roll stick low-pass filter\n");
		return -1;
	}

	if (unlikely(rc_filter_first_order_lowpass(&pitch_stick_lpf, DT, 0.1 * DT) == -1))
	{
		printf("ERROR in init_stick_filter: failed to create pitch stick low-pass filter\n");
		return -1;
	}

	if (unlikely(rc_filter_first_order_lowpass(&yaw_stick_lpf, DT, 0.1 * DT) == -1))
	{
		printf("ERROR in init_stick_filter: failed to create yaw stick high-pass filter\n");
		return -1;
	}

	if (unlikely(rc_filter_first_order_highpass(&roll_stick_hpf, DT, 6.0 * DT) == -1))
	{
		printf("ERROR in init_stick_filter: failed to create roll stick high-pass filter\n");
		return -1;
	}

	if (unlikely(rc_filter_first_order_highpass(&pitch_stick_hpf, DT, 6.0 * DT) == -1))
	{
		printf("ERROR in init_stick_filter: failed to create pitch stick high-pass filter\n");
		return -1;
	}

	if (unlikely(rc_filter_first_order_highpass(&yaw_stick_hpf, DT, 6.0 * DT) == -1))
	{
		printf("ERROR in init_stick_filter: failed to create yaw stick high-pass filter\n");
		return -1;
	}

	rc_filter_enable_saturation(&roll_stick_lpf, -1.0, 1.0);
	rc_filter_enable_saturation(&pitch_stick_lpf, -1.0, 1.0);
	rc_filter_enable_saturation(&yaw_stick_lpf, -1.0, 1.0);

	rc_filter_enable_saturation(&roll_stick_hpf, -1.0, 1.0);
	rc_filter_enable_saturation(&pitch_stick_hpf, -1.0, 1.0);
	rc_filter_enable_saturation(&yaw_stick_hpf, -1.0, 1.0);

	roll_stick_lp = 0.0;
	pitch_stick_lp = 0.0;
	yaw_stick_lp = 0.0;
	roll_stick_hp = 0.0;
	pitch_stick_hp = 0.0;
	yaw_stick_hp = 0.0;
	last_en_stick_filter_lp = false;
	last_en_stick_filter_hp = false;

	return 0;
}

int setpoint_t::update_trans(void)
{
	if (!last_en_trans)
	{
		rc_filter_reset(&trans_stick_int);

		last_en_stick_trim = false;
		last_en_stick_filter_hp = false;
		last_en_stick_filter_lp = false;
	}

	//update_stick_filter_lp(); //use low-pass to filter out the high frequency changes
	//update_stick_filter_hp(); //use high-pass to get only constant signals
	//update_stick_trim(roll_stick_lp, roll_stick_lp, yaw_stick_lp); //use filtered values for trim
	update_stick_trim(0.05 * user_input.roll.get(), 0.1 * user_input.pitch.get(), 0.05 * user_input.yaw.get());

	double manual_trim = -rc_filter_march(&trans_stick_int,\
		0.6 * (user_input.requested_flight_mode.get() - 0.5) * 2.0);
	roll_tr = manual_trim;
	pitch_tr = manual_trim;
	yaw_tr =  manual_trim;

	roll_servo_tr = manual_trim;
	pitch_servo_tr = manual_trim;
	yaw_servo_tr = manual_trim;

	last_en_trans = true;
	return 0;
}

int setpoint_t::update_stick_trim(double roll_st, double pitch_st, double yaw_st)
{
	if (!last_en_stick_trim)
	{
		roll_stick_trim = 0.0;
		pitch_stick_trim = 0.0;
		yaw_stick_trim = 0.0;

		rc_filter_reset(&roll_stick_int);
		rc_filter_reset(&pitch_stick_int);
		rc_filter_reset(&yaw_stick_int);
	}

	roll_stick_trim = rc_filter_march(&roll_stick_int, roll_st);
	pitch_stick_trim = rc_filter_march(&pitch_stick_int, pitch_st);
	yaw_stick_trim = rc_filter_march(&yaw_stick_int, yaw_st);

	last_en_stick_trim = true;
	return 0;
}

int setpoint_t::update_stick_filter_lp(void)
{
	if (!last_en_stick_filter_lp)
	{
		roll_stick_lp = 0.0;
		pitch_stick_lp = 0.0;
		yaw_stick_lp = 0.0;

		rc_filter_reset(&roll_stick_lpf);
		rc_filter_reset(&pitch_stick_lpf);
		rc_filter_reset(&yaw_stick_lpf);
	}

	roll_stick_lp = rc_filter_march(&roll_stick_lpf, user_input.roll.get());
	pitch_stick_lp = rc_filter_march(&pitch_stick_lpf, user_input.pitch.get());
	yaw_stick_lp = rc_filter_march(&yaw_stick_lpf, user_input.yaw.get());

	last_en_stick_filter_lp = true;
	return 0;
}

int setpoint_t::update_stick_filter_hp(void)
{
	if (!last_en_stick_filter_hp)
	{
		roll_stick_hp = 0.0;
		pitch_stick_hp = 0.0;
		yaw_stick_hp = 0.0;

		rc_filter_reset(&roll_stick_hpf);
		rc_filter_reset(&pitch_stick_hpf);
		rc_filter_reset(&yaw_stick_hpf);
	}

	roll_stick_hp = rc_filter_march(&roll_stick_hpf, user_input.roll.get());
	pitch_stick_hp = rc_filter_march(&pitch_stick_hpf, user_input.pitch.get());
	yaw_stick_hp = rc_filter_march(&yaw_stick_hpf, user_input.yaw.get());

	last_en_stick_filter_hp = true;
	return 0;
}
*/

/* Functions which should be called internally to update setpoints based on radio input:*/
void setpoint_t::update_yaw(void)
{
	// if throttle stick is down all the way, probably landed, so
	// keep the yaw setpoint at current yaw so it takes off straight
	double tmp_yaw = user_input.yaw.get();

	/*	
	double  vel_mag = sqrt(\
		state_estimate.X_dot * state_estimate.X_dot\
		+ state_estimate.Y_dot * state_estimate.Y_dot\
		+ state_estimate.Z_dot * state_estimate.Z_dot);

	if(user_input.throttle.get() < 0.05 && fabs(tmp_yaw) < 0.05){
		ATT.z.value.set(state_estimate.continuous_yaw);
		return;
	}
	*/
	// otherwise, scale yaw by max yaw rate in rad/s
	// and move yaw setpoint
	ATT.z.value.increment(__deadzone(tmp_yaw, 0.05) * MAX_YAW_RATE * DT);
	return;
}

void setpoint_t::update_rp(void)
{
	ATT.x.value.set(__deadzone(user_input.roll.get(), 0.05) * MAX_ROLL_SETPOINT);
	ATT.y.value.set(__deadzone(user_input.pitch.get(), 0.05) * MAX_PITCH_SETPOINT);
	return;
}

void setpoint_t::update_th(void)
{
	POS_throttle.z.value.set(-user_input.throttle.get() / \
		(cos(state_estimate.roll) * cos(state_estimate.pitch)));

	return;
}

void setpoint_t::update_rpy_rate(void)
{
	ATT_dot.x.value.set(__deadzone(user_input.roll.get(), 0.05) * MAX_ROLL_RATE);
	ATT_dot.y.value.set(__deadzone(user_input.pitch.get(), 0.05) * MAX_PITCH_RATE);

	
	double tmp_yaw = user_input.yaw.get();
	/*
	if (user_input.throttle.get() < 0.05 && fabs(tmp_yaw) < 0.05) {
		ATT_dot.z.value.set(state_estimate.yaw_dot);
		return;
	}
	*/

	ATT_dot.z.value.set(__deadzone(tmp_yaw, 0.05) * MAX_YAW_RATE);
	return;
}

void setpoint_t::update_rpy_servo(void)
{
	ATT_servo.x.value.set(__deadzone(user_input.roll.get(), 0.05));
	ATT_servo.y.value.set(__deadzone(user_input.pitch.get(), 0.05));
	ATT_servo.z.value.set(__deadzone(user_input.yaw.get(), 0.05));
	return;
}

//----Manual/Radio/Direct altitude control----//
// only run this is need an update from radio control.
// make sure setpoint doesn't go below or above controller limits
void setpoint_t::update_Z(void)
{
	double tmp_Z_dot;
	double tmp_throtle = user_input.throttle.get();

	if (tmp_throtle > Z_throttle_0 + 0.1)
	{
		tmp_Z_dot = (tmp_throtle - Z_throttle_0 - 0.1) * MAX_Z_VELOCITY;
	}
	else if (tmp_throtle < Z_throttle_0 - 0.1)
	{
		tmp_Z_dot = (tmp_throtle - Z_throttle_0 + 0.1) * MAX_Z_VELOCITY;
	}
	else
	{
		tmp_Z_dot = 0;
		return;
	}
	double tmp_Z_sp = Z.value.get() - tmp_Z_dot * DT; //negative since Z positive is defined to be down	

	if (tmp_Z_sp > state_estimate.Z + XYZ_MAX_ERROR)
	{
		Z.value.set(state_estimate.Z + XYZ_MAX_ERROR);
		return;
	}
	else if (tmp_Z_sp < state_estimate.Z - XYZ_MAX_ERROR)
	{
		Z.value.set(state_estimate.Z - XYZ_MAX_ERROR);
		return;
	}
	else
	{
		Z.value.set(tmp_Z_sp);
		return;
	}
	
	return;
}

//----Manual/Radio/Direct control of vertical velocity ----//
// only run this is need an update from radio control.
// make sure setpoint doesn't go below or above controller limits
void setpoint_t::update_Z_dot(void)
{
	double tmp_Zdot_sp;
	double tmp_throtle = user_input.throttle.get();

	if (tmp_throtle > Z_throttle_0 + 0.1)
	{
		tmp_Zdot_sp = (tmp_throtle - Z_throttle_0 - 0.1) * MAX_Z_VELOCITY;
	}
	else if (tmp_throtle < Z_throttle_0 - 0.1)
	{
		tmp_Zdot_sp = (tmp_throtle - Z_throttle_0 + 0.1) * MAX_Z_VELOCITY;
	}
	else
	{
		tmp_Zdot_sp = 0;
		return;
	}
	
	tmp_Zdot_sp = -tmp_Zdot_sp; //negative since Z positive is defined to be down	
	
	if (tmp_Zdot_sp > MAX_Z_VELOCITY)
	{
		Z_dot.value.set(MAX_Z_VELOCITY);
		return;
	}
	else if (tmp_Zdot_sp < -XYZ_MAX_ERROR)
	{
		Z_dot.value.set(-MAX_Z_VELOCITY);
		return;
	}
	else
	{
		Z_dot.value.set(tmp_Zdot_sp);
		return;
	}
	return;
}


//----Manual/Radio/Direct control of horizontal velocity ----//
void setpoint_t::update_XY_vel(void)
{	
	double tmp_yaw = state_estimate.continuous_yaw;
	double tmp_roll = __deadzone(user_input.roll.get(), 0.05);
	double tmp_pitch = __deadzone(user_input.pitch.get(), 0.05);

	double tmp_x_sp = (-tmp_pitch * cos(tmp_yaw)\
		- tmp_roll * sin(tmp_yaw));

	double tmp_y_sp = (tmp_roll * cos(tmp_yaw)\
		- tmp_pitch * sin(tmp_yaw));

	__get_norm_sp_bounded_2D(tmp_x_sp, tmp_y_sp,\
		tmp_x_sp, tmp_y_sp,\
		state_estimate.X_dot / MAX_XY_VELOCITY_NORM, state_estimate.Y_dot / MAX_XY_VELOCITY_NORM,\
		1.0);

	XY_dot.x.value.set(tmp_x_sp * MAX_XY_VELOCITY_NORM);
	XY_dot.y.value.set(tmp_y_sp * MAX_XY_VELOCITY_NORM);

	
	/*
	double tmp_roll_stick = __deadzone(user_input.roll.get(), 0.05);
	double tmp_pitch_stick = __deadzone(user_input.pitch.get(), 0.05);
	XY_dot.x.value.set((-tmp_pitch_stick * cos(tmp_yaw)\
		- tmp_roll_stick * sin(tmp_yaw))\
		* MAX_XY_VELOCITY);

	XY_dot.y.value.set((tmp_roll_stick * cos(tmp_yaw)\
		- tmp_pitch_stick * sin(tmp_yaw))\
		* MAX_XY_VELOCITY);

	*/
	return;
}

//----Manual/Radio/Direct control of horizontal position ----//
void setpoint_t::update_XY_pos(void)
{
	// X in the body frame (forward flight)
	// Y in the body frame (lateral translation to the right wing)
	double tmp_yaw = state_estimate.continuous_yaw;
	double tmp_roll = __deadzone(user_input.roll.get(), 0.05);
	double tmp_pitch = __deadzone(user_input.pitch.get(), 0.05);

	double tmp_x_sp = (-tmp_pitch * cos(tmp_yaw)\
		- tmp_roll * sin(tmp_yaw));

	double tmp_y_sp = (tmp_roll * cos(tmp_yaw)\
		- tmp_pitch * sin(tmp_yaw));

	__get_norm_sp_bounded_2D(tmp_x_sp, tmp_y_sp, \
		tmp_x_sp, tmp_y_sp, \
		state_estimate.X / XY_MAX_ERROR_NORM, state_estimate.Y / XY_MAX_ERROR_NORM, \
		1.0);

	XY.x.value.set(tmp_x_sp * XY_MAX_ERROR_NORM);
	XY.y.value.set(tmp_y_sp * XY_MAX_ERROR_NORM);


	/*
	double tmp_X_dot, tmp_Y_dot;
	// X in the body frame (forward flight)
	// make sure setpoint doesn't go too far from state in case touching something
	if(XY.x.value.get() > (state_estimate.X + XYZ_MAX_ERROR)) {
		XY.x.value.set(state_estimate.X + XYZ_MAX_ERROR);
		tmp_X_dot = 0.0;
	}
	else if(XY.x.value.get() < (state_estimate.X - XYZ_MAX_ERROR)){
		XY.x.value.set(state_estimate.X - XYZ_MAX_ERROR);
		tmp_X_dot = 0.0;
		return;
	}
	else{
		tmp_X_dot = (-user_input.pitch.get() * cos(state_estimate.continuous_yaw)\
			- user_input.roll.get() * sin(state_estimate.continuous_yaw))\
			* MAX_XY_VELOCITY;

		//apply velocity command 
		XY.x.value.increment(tmp_X_dot * DT);
	}
	
	// Y in the body frame (lateral translation)
	// make sure setpoint doesn't go too far from state in case touching something
	
	if(XY.y.value.get() > (state_estimate.Y + XYZ_MAX_ERROR)){
		XY.y.value.set(state_estimate.Y + XYZ_MAX_ERROR);
		tmp_Y_dot = 0.0;
		return;
	}
	else if(XY.y.value.get() < (state_estimate.Y - XYZ_MAX_ERROR)){
		XY.y.value.set(state_estimate.Y - XYZ_MAX_ERROR);
		tmp_Y_dot = 0.0;
		return;
	}
	else{
		tmp_Y_dot = (user_input.roll.get() * cos(state_estimate.continuous_yaw)\
			- user_input.pitch.get() * sin(state_estimate.continuous_yaw))\
			* MAX_XY_VELOCITY;

		//apply velocity command 
		XY.y.value.increment(tmp_Y_dot * DT); //Y is defined positive to the left 
	}
	*/

	return;
}



/**
* @brief      Initializes the setpoint manager.
*
* @return     0 on success, -1 on failure
*/
int setpoint_t::init(void)
{
	if(unlikely(initialized)){
		fprintf(stderr, "ERROR in setpoint_manager_init, already initialized\n");
		return -1;
	}

	if (unlikely(init_all_filters() == -1))
	{
		fprintf(stderr, "ERROR in init: failed to initialize all filters\n");
		return -1;
	}
	/*
	if (unlikely(init_stick_trim() == -1))
	{
		fprintf(stderr, "ERROR in init: failed to initialize stick trims\n");
		return -1;
	}

	if (unlikely(init_stick_filter() == -1))
	{
		fprintf(stderr, "ERROR in init: failed to initialize stick filters\n");
		return -1;
	}
	*/
	if (unlikely(setpoint_guidance.init() == -1))
	{
		fprintf(stderr, "ERROR in init, failed to initialize setpoint guidance\n");
		return -1;
	}

	reset_all();

	initialized = true;
	return 0;
}
bool setpoint_t::is_initialized(void) const
{
	return initialized;
}

/**
* @brief      Configure setpoints to reset to state_estimate.
*
* @return     0 on success, -1 on failure
*/
int setpoint_t::set_reset_sources(void)
{
	// reset is configured to set values to zero by def.
	// change the def source if non always zero:
	ATT_dot.x.value.set_def(&state_estimate.roll_dot);
	ATT_dot.y.value.set_def(&state_estimate.pitch_dot);
	ATT_dot.z.value.set_def(&state_estimate.yaw_dot);

	ATT.x.value.set_def(&state_estimate.roll);
	ATT.y.value.set_def(&state_estimate.pitch);
	ATT.z.value.set_def(&state_estimate.continuous_yaw);

	XY_dot.x.value.set_def(&state_estimate.X_dot);
	XY_dot.y.value.set_def(&state_estimate.Y_dot);
	Z_dot.value.set_def(&state_estimate.Z_dot);

	XY.x.value.set_def(&state_estimate.X);
	XY.y.value.set_def(&state_estimate.Y);
	Z.value.set_def(&state_estimate.Z);
	return 0;
}

int setpoint_t::set_reset_sources_all_defs(void)
{
	if (unlikely(set_reset_sources() < 0))
	{
		printf("ERROR in set_reset_sources_all_defs: failed to reset setpoint default sources\n");
		return -1;
	}
	return 0;
}

/**
* @brief      Reset setpoint mannager to its starting state.
*
*	Intended to be called each time the system is armed.
*
* @return     0 on success, -1 on failure
*/
int setpoint_t::reset_all(void)
{
	set_reset_sources_all_defs();

	ATT_dot.reset_all();
	ATT.reset_all();
	XY_dot.reset_all();
	Z_dot.reset_all();
	XY.reset_all();
	Z.reset_all();

	setpoint_guidance.reset();
	waypoint_state_machine.reset();
	
	return 0;
}

/**
* @brief      updates the setpoint manager, call this before feedback loop
*
* @return     0 on success, -1 on failure
*/
int setpoint_t::update(void)
{
	if (!initialized)
	{
		fprintf(stderr, "ERROR in setpoint_manager_update, not initialized yet\n");
		return -1;
	}

	if (settings.enable_dsm && !user_input.is_initialized())
	{
		fprintf(stderr, "ERROR in setpoint_manager_update, input_manager not initialized yet\n");
		return -1;
	}

	// if PAUSED or UNINITIALIZED, do nothing
	if (rc_get_state() != RUNNING) return 0;

	//if (user_input.get_flight_mode() != AUTO_FFFAFA) waypoint_state_machine.disable_update();

	// shutdown feedback on kill switch
	if (user_input.requested_arm_mode == DISARMED)
	{
		if (fstate.get_arm_state() != DISARMED)
		{
			fstate.disarm();
			//last_en_trans = false;
			setpoint_guidance.reset_Z();
			setpoint_guidance.reset_XY();
		}
		return 0;
	}

	// arm feedback when requested
	if (user_input.requested_arm_mode == ARMED) {
		if (fstate.get_arm_state() == DISARMED) fstate.arm(), \
			setpoint_guidance.reset_Z(), \
			setpoint_guidance.reset_XY();
	}
	else
	{
		fstate.reset_arming_fl();
	}
	update_setpoints(); //get manual radio updates first

	// Update the state machine if in autonomous operation
	/*	
	if (user_input.get_flight_mode() == AUTO_FFFAFA)
	{
		waypoint_state_machine.march();
		if (settings.log_benchmark) benchmark_timers.tSM = rc_nanos_since_boot();
	}
	setpoint_guidance.march();
	*/
	return 0;
}

/**
* @brief      cleans up the setpoint manager, not really necessary but here for
*             completeness
*
* @return     0 on clean exit, -1 if exit timed out
*/
int setpoint_t::cleanup(void)
{
	initialized = false;
	return 0;
}


int setpoint_t::update_setpoints(void)
{
	// Check if switching to a different flight mode:
	flight_mode_switching = flight_mode_prev != user_input.get_flight_mode();
	flight_mode_prev = user_input.get_flight_mode();

	// Perform swithcing of control scheme if nessesary:
	if (flight_mode_switching) //switch everything off
	{
		ATT_throttle.disable_all();
		ATT_throttle_servo.disable_all();

		POS_throttle.disable_all();
		POS_throttle_servo.disable_all();

		
		ATT_dot.disable_all();
		ATT_dot_servo.disable_all();
		
		ATT.disable_all();
		ATT_servo.disable_all();

		XYZ_ddot.disable_all();
		XYZ_ddot_servo.disable_all();

		Z_dot.disable_all();
		Z_dot_servo.disable_all();

		Z.disable_all();
		Z_servo.disable_all();	
		
		XY_dot.disable_all();
		XY_dot_servo.disable_all();

		XY.disable_all();
		XY_servo.disable_all();
	}


	// finally, switch between flight modes and adjust setpoint properly
	switch (user_input.get_flight_mode()) {
	case TEST_BENCH_4DOF:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();

		ATT_throttle.set(user_input.roll.get(), user_input.pitch.get(), user_input.yaw.get());
		POS_throttle.set(0.0, 0.0, -user_input.throttle.get());

		break;

	case TEST_BENCH_6DOF:
		// configure which controllers are enabled
		ATT_throttle.z.enable();
		POS_throttle.enable();

		ATT_throttle.set(0.0, 0.0, user_input.yaw.get());
		POS_throttle.set(-user_input.pitch.get(), user_input.roll.get(), -user_input.throttle.get());
		break;

	case TEST_6xSERVOS_DIRECT:
		// configure which controllers are enabled
		ATT_throttle_servo.enable();
		POS_throttle_servo.z.enable();

		ATT_throttle_servo.set(user_input.roll.get(), user_input.pitch.get(), user_input.yaw.get());
		POS_throttle_servo.set(0.0, 0.0, -user_input.throttle.get());
		break;

	case DIRECT_THROTTLE_6DOF:
		// configure which controllers are enabled
		ATT_throttle.z.enable();
		POS_throttle.enable();
		ATT.enable();
		
		POS_throttle.set(-user_input.pitch.get(), user_input.roll.get(), -user_input.throttle.get());
		update_yaw();
		break;

	case ACRO_Axxxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();		

		update_rpy_rate();
		update_th();
		break;

	case ACRO_Fxxxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT_dot.enable_FF();
		

		update_rpy_rate();
		update_th();
		break;

	case MANUAL_xAxxxx:		
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT.enable();
		
		update_rp();
		update_th();
		update_yaw();
		break;

	case MANUAL_xFxxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT.enable();
		ATT.enable_FF();

		update_rp();
		update_th();
		update_yaw();
		break;

	case MANUAL_AAxxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		
		update_rp();
		update_th();
		update_yaw();
		break;

	case MANUAL_FAxxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		ATT_dot.enable_FF();

		update_rp();
		update_th();
		update_yaw();
		break;

	case MANUAL_FFxxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		ATT_dot.enable_FF();
		ATT.enable_FF();

		update_rp();
		update_th();
		update_yaw();
		break;	

	case ALT_HOLD_AxAxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		Z_dot.enable();


		update_rpy_rate();
		update_Z_dot();
		break;

	case ALT_HOLD_FxAxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		Z_dot.enable();

		ATT_dot.enable_FF();


		update_rpy_rate();
		update_Z_dot();
		break;

	case ALT_HOLD_FxFxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		Z_dot.enable();

		ATT_dot.enable_FF();
		Z_dot.enable_FF();

		update_rpy_rate();
		update_Z_dot();
		break;

	case ALT_HOLD_AxAAxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		Z_dot.enable();
		Z.enable();

		update_rpy_rate();
		update_Z();
		break;

	case ALT_HOLD_FxAAxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		Z_dot.enable();
		Z.enable();

		ATT_dot.enable_FF();

		update_rpy_rate();
		update_Z();
		break;

	case ALT_HOLD_FxFAxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		Z_dot.enable();
		Z.enable();

		ATT_dot.enable_FF();
		Z_dot.enable_FF();

		update_rpy_rate();
		update_Z();
		break;

	case ALT_HOLD_FxFFxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		Z_dot.enable();
		Z.enable();

		ATT_dot.enable_FF();
		Z_dot.enable_FF();
		Z.enable_FF();

		update_rpy_rate();
		update_Z();
		break;

	case ALT_HOLD_xAxAxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT.enable();
		Z.enable();
		

		update_rp();
		update_Z();
		update_yaw();
		break;

	case ALT_HOLD_xFxAxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT.enable();
		Z.enable();
		ATT.enable_FF();

		update_rp();
		update_Z();
		update_yaw();
		break;

	case ALT_HOLD_xFxFxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT.enable();
		Z.enable();
		ATT.enable_FF();
		Z.enable_FF();

		update_rp();
		update_Z();
		update_yaw();
		break;

	case ALT_HOLD_AAAxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();


		update_rp();
		update_Z_dot();
		update_yaw();
		break;

	case ALT_HOLD_FAAxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();

		ATT_dot.enable_FF();


		update_rp();
		update_Z_dot();
		update_yaw();
		break;

	case ALT_HOLD_FFAxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();


		update_rp();
		update_Z_dot();
		update_yaw();
		break;

	case ALT_HOLD_FFFxxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();
		Z_dot.enable_FF();


		update_rp();
		update_Z_dot();
		update_yaw();
		break;

	case ALT_HOLD_AAAAxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z.enable();
		Z_dot.enable();

		update_rp();
		update_Z();
		update_yaw();
		break;

	case ALT_HOLD_FAAAxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z.enable();
		Z_dot.enable();

		ATT_dot.enable_FF();


		update_rp();
		update_Z();
		update_yaw();
		break;

	case ALT_HOLD_FFAAxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z.enable();
		Z_dot.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();


		update_rp();
		update_Z();
		update_yaw();
		break;

	case ALT_HOLD_FFFAxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z.enable();
		Z_dot.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();
		Z_dot.enable_FF();


		update_rp();
		update_Z();
		update_yaw();
		break;

	case ALT_HOLD_FFFFxx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z.enable();
		Z_dot.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();
		Z_dot.enable_FF();
		Z.enable_FF();


		update_rp();
		update_Z();
		update_yaw();

		break;

	case POS_CTRL_AAAAAA:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();
		Z.enable();
		XY_dot.enable();
		XY.enable();

		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();
		break;

	case POS_CTRL_FFFAAx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();
		Z.enable();
		XY_dot.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();
		Z_dot.enable_FF();


		//check validity of the velocity command, construct virtual setpoint
		update_XY_vel();
		update_Z();
		update_yaw();
		break;

	case POS_CTRL_FFFAFx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();
		Z.enable();
		XY_dot.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();
		Z_dot.enable_FF();
		XY_dot.enable_FF();


		//check validity of the velocity command, construct virtual setpoint
		update_XY_vel();
		update_Z();
		update_yaw();
		break;

	case POS_CTRL_FFFFAx:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();
		Z.enable();
		XY_dot.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();
		Z_dot.enable_FF();
		Z.enable_FF();


		//check validity of the velocity command, construct virtual setpoint
		update_XY_vel();
		update_Z();
		update_yaw();
		break;

	case POS_CTRL_FFFAAA:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();
		Z.enable();
		XY_dot.enable();
		XY.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();
		Z_dot.enable_FF();		


		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();
		break;

	case POS_CTRL_FFFAFA:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();
		Z.enable();
		XY_dot.enable();
		XY.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();
		Z_dot.enable_FF();
		XY_dot.enable_FF();


		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();
		break;

	case POS_CTRL_FFFFFF:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();
		Z.enable();
		XY_dot.enable();
		XY.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();
		Z_dot.enable_FF();
		Z.enable_FF();
		XY_dot.enable_FF();
		XY.enable_FF();
		
		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();
		break;

	case POS_CTRL_FFFAFF:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();
		Z.enable();
		XY_dot.enable();
		XY.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();
		Z_dot.enable_FF();
		XY_dot.enable_FF();
		XY.enable_FF();

		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();
		break;

	case EMERGENCY_LAND:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z.enable();
		Z_dot.enable();


		//Assign Setpoints
		setpoint_guidance.start_land();  // start landing algorithm

		update_yaw();
		break;

	case AUTO_FFFAFA:
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		ATT_dot.enable();
		ATT.enable();
		Z_dot.enable();
		Z.enable();
		XY_dot.enable();
		XY.enable();

		ATT_dot.enable_FF();
		ATT.enable_FF();
		Z_dot.enable_FF();
		XY_dot.enable_FF();

		printf("Requesting update from setpoint manager\n");
		waypoint_state_machine.request_update();

		break;

	case ZEPPELIN:
		printf("WARNING: ZEPPELIN flight mode is not longer supported\n");
		break;
		/*
		// configure which controllers are enabled
		ATT_throttle.enable();
		POS_throttle.z.enable();
		*/

		
		//en_rpy_trans = true;

		//en_6dof_servo = true;
		//en_rpy_servo_trans = true;

		//update_trans(); //update transitions, filters and trims

		//ATT_throttle.set(user_input.roll.get() + roll_stick_trim, user_input.pitch.get(), user_input.yaw.get() + yaw_stick_trim);
		//POS_throttle.z.value.set(-user_input.throttle.get());
		
		//roll_servo_throttle = user_input.roll.get() + roll_stick_trim;
		//pitch_servo_throttle = user_input.pitch.get(); // pitch_stick_hp;
		//yaw_servo_throttle = user_input.yaw.get() + yaw_stick_trim;
		
		//X_servo_throttle = user_input.yaw.get() + yaw_stick_trim;
		//Y_servo_throttle = pitch_servo_tr;
		//Z_servo_throttle = 0.0;
		
		
		//break;
		

	default: // should never get here
		fprintf(stderr, "ERROR in setpoint_manager thread, unknown flight mode\n");
		break;

	} // end switch(user_input.get_flight_mode())
	return 0;
}



