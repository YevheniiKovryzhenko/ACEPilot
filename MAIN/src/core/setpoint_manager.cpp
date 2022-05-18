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
 * Last Edit:  05/18/2022 (MM/DD/YYYY)
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

 /*
 * Invoke defaut constructor for all the built in and exernal types in the class
 */
setpoint_t setpoint{}; // extern variable in setpoint_manager.hpp

/***********************************/
int setpoint_t::init_trans(void)
{
	trans_stick_int = RC_FILTER_INITIALIZER;
	if (unlikely(rc_filter_integrator(&trans_stick_int, DT) == -1))
	{
		printf("\nERROR in init_trans: failed to create transition stick integrator");
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
		printf("\nERROR in init_stick_trim: failed to create roll stick trim integrator");
		return -1;
	}

	if (unlikely(rc_filter_integrator(&pitch_stick_int, DT) == -1))
	{
		printf("\nERROR in init_stick_trim: failed to create pitch stick trim integrator");
		return -1;
	}

	if (unlikely(rc_filter_integrator(&yaw_stick_int, DT) == -1))
	{
		printf("\nERROR in init_stick_trim: failed to create yaw stick trim integrator");
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
		printf("\nERROR in init_stick_filter: failed to create roll stick low-pass filter");
		return -1;
	}

	if (unlikely(rc_filter_first_order_lowpass(&pitch_stick_lpf, DT, 0.1 * DT) == -1))
	{
		printf("\nERROR in init_stick_filter: failed to create pitch stick low-pass filter");
		return -1;
	}

	if (unlikely(rc_filter_first_order_lowpass(&yaw_stick_lpf, DT, 0.1 * DT) == -1))
	{
		printf("\nERROR in init_stick_filter: failed to create yaw stick high-pass filter");
		return -1;
	}

	if (unlikely(rc_filter_first_order_highpass(&roll_stick_hpf, DT, 6.0 * DT) == -1))
	{
		printf("\nERROR in init_stick_filter: failed to create roll stick high-pass filter");
		return -1;
	}

	if (unlikely(rc_filter_first_order_highpass(&pitch_stick_hpf, DT, 6.0 * DT) == -1))
	{
		printf("\nERROR in init_stick_filter: failed to create pitch stick high-pass filter");
		return -1;
	}

	if (unlikely(rc_filter_first_order_highpass(&yaw_stick_hpf, DT, 6.0 * DT) == -1))
	{
		printf("\nERROR in init_stick_filter: failed to create yaw stick high-pass filter");
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
	update_stick_trim(0.05 * user_input.get_roll_stick(), 0.1 * user_input.get_pitch_stick(), 0.05 * user_input.get_yaw_stick());

	double manual_trim = -rc_filter_march(&trans_stick_int,\
		0.6 * (user_input.get_mode_stick() - 0.5) * 2.0);
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

	roll_stick_lp = rc_filter_march(&roll_stick_lpf, user_input.get_roll_stick());
	pitch_stick_lp = rc_filter_march(&pitch_stick_lpf, user_input.get_pitch_stick());
	yaw_stick_lp = rc_filter_march(&yaw_stick_lpf, user_input.get_yaw_stick());

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

	roll_stick_hp = rc_filter_march(&roll_stick_hpf, user_input.get_roll_stick());
	pitch_stick_hp = rc_filter_march(&pitch_stick_hpf, user_input.get_pitch_stick());
	yaw_stick_hp = rc_filter_march(&yaw_stick_hpf, user_input.get_yaw_stick());

	last_en_stick_filter_hp = true;
	return 0;
}


/* Functions which should be called internally to update setpoints based on radio input:*/
void setpoint_t::update_yaw(void)
{
	// if throttle stick is down all the way, probably landed, so
	// keep the yaw setpoint at current yaw so it takes off straight
	double  vel_mag = sqrt(\
		state_estimate.X_dot * state_estimate.X_dot\
		+ state_estimate.Y_dot * state_estimate.Y_dot\
		+ state_estimate.Z_dot * state_estimate.Z_dot);
	if(user_input.get_thr_stick() < 0.05 && vel_mag < 0.1){
		yaw = state_estimate.continuous_yaw;
		yaw_dot = 0.0;
		return;
	}
	// otherwise, scale yaw_rate by max yaw rate in rad/s
	// and move yaw setpoint
	yaw_dot = user_input.get_yaw_stick() * MAX_YAW_RATE;
	yaw += yaw_dot*DT;
	return;
}

void setpoint_t::update_rp(void)
{
	roll = user_input.get_roll_stick();
	pitch = user_input.get_pitch_stick();
	
	return;
}

void setpoint_t::update_th(void)
{
	Z_throttle = -user_input.get_thr_stick() / \
		(cos(state_estimate.roll) * cos(state_estimate.pitch));
	return;
}

void setpoint_t::update_rpy_rate(void)
{
	roll_dot = user_input.get_roll_stick() * MAX_ROLL_RATE;
	pitch_dot = user_input.get_pitch_stick() * MAX_PITCH_RATE;
	yaw_dot = user_input.get_yaw_stick() * MAX_YAW_RATE;

	return;
}

void setpoint_t::update_rpy_servo(void)
{
	roll_servo = user_input.get_roll_stick();
	pitch_servo = user_input.get_pitch_stick();
	yaw_servo = user_input.get_yaw_stick();

	return;
}

//----Manual/Radio/Direct control----//
// only run this is need an update from radio control.
// make sure setpoint doesn't go too far below current altitude since we
// can't sink into the ground
void setpoint_t::update_Z(void)
{
	

	double tmp_Z_dot;

	if	(user_input.get_thr_stick() > Z_throttle_0+0.1)
	{
		tmp_Z_dot = (user_input.get_thr_stick() - Z_throttle_0) * settings.max_Z_velocity;
	}
	else if (user_input.get_thr_stick() < Z_throttle_0-0.1)
	{
		tmp_Z_dot = (user_input.get_thr_stick() - Z_throttle_0) * settings.max_Z_velocity;
	}
	else
	{
		tmp_Z_dot = 0;
		return;
	}
	Z -= tmp_Z_dot*DT; //neagtive since Z positive is defined to be down
	
	return;
}


//----Manual/Radio/Direct control----//
void setpoint_t::update_XY_pos(void)
{
	double tmp_X_dot, tmp_Y_dot;
	// X in the body frame (forward flight)
	// make sure setpoint doesn't go too far from state in case touching something
	if(X > (state_estimate.X + XYZ_MAX_ERROR)){
		X = state_estimate.X + XYZ_MAX_ERROR;
		tmp_X_dot = 0.0;
	}
	else if(X < (state_estimate.X - XYZ_MAX_ERROR)){
		X = state_estimate.X - XYZ_MAX_ERROR;
		tmp_X_dot = 0.0;
		return;
	}
	else{
		tmp_X_dot = (-user_input.get_pitch_stick() * cos(state_estimate.continuous_yaw)\
			- user_input.get_roll_stick() * sin(state_estimate.continuous_yaw))\
			* settings.max_XY_velocity;

		//apply velocity command 
		X += tmp_X_dot * DT;
	}
	
	// Y in the body frame (lateral translation)
	// make sure setpoint doesn't go too far from state in case touching something
	
	if(Y > (state_estimate.Y + XYZ_MAX_ERROR)){
		Y = state_estimate.Y + XYZ_MAX_ERROR;
		tmp_Y_dot = 0.0;
		return;
	}
	else if(Y < (state_estimate.Y - XYZ_MAX_ERROR)){
		Y = state_estimate.Y - XYZ_MAX_ERROR;
		tmp_Y_dot = 0.0;
		return;
	}
	else{
		tmp_Y_dot = (user_input.get_roll_stick() * cos(state_estimate.continuous_yaw)\
			- user_input.get_pitch_stick() * sin(state_estimate.continuous_yaw))\
			* settings.max_XY_velocity;

		//apply velocity command 
		Y += tmp_Y_dot * DT; //Y is defined positive to the left
	}
	

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
		fprintf(stderr, "\nERROR in setpoint_manager_init, already initialized");
		return -1;
	}

	if (unlikely(init_trans() == -1))
	{
		fprintf(stderr, "\nERROR in init: failed to initialize transition functions");
		return -1;
	}

	if (unlikely(init_stick_trim() == -1))
	{
		fprintf(stderr, "\nERROR in init: failed to initialize stick trims");
		return -1;
	}

	if (unlikely(init_stick_filter() == -1))
	{
		fprintf(stderr, "\nERROR in init: failed to initialize stick filters");
		return -1;
	}

	if (unlikely(setpoint_guidance.init() == -1))
	{
		fprintf(stderr, "\nERROR in init, failed to initialize setpoint guidance");
		return -1;
	}

	reset_all();

	initialized = true;
	return 0;
}
bool setpoint_t::is_initialized(void)
{
	return initialized;
}


/**
* @brief      Set setpoints to input value.
*
* @return     0 on success, -1 on failure
*/
int setpoint_t::set_roll_dot(double val)
{
	roll_dot = val;
	return 0;
}
int setpoint_t::set_pitch_dot(double val)
{
	pitch_dot = val;
	return 0;
}
int setpoint_t::set_yaw_dot(double val)
{
	yaw_dot = val;
	return 0;
}
int setpoint_t::set_roll_dot_ff(double val)
{
	roll_dot_ff = val;
	return 0;
}
int setpoint_t::set_pitch_dot_ff(double val)
{
	pitch_dot_ff = val;
	return 0;
}
int setpoint_t::set_yaw_dot_ff(double val)
{
	yaw_dot_ff = val;
	return 0;
}

int setpoint_t::set_roll(double val)
{
	roll = val;
	return 0;
}
int setpoint_t::set_pitch(double val)
{
	pitch = val;
	return 0;
}
int setpoint_t::set_yaw(double val)
{
	yaw = val;
	return 0;
}
int setpoint_t::set_roll_ff(double val)
{
	roll_ff = val;
	return 0;
}
int setpoint_t::set_pitch_ff(double val)
{
	pitch_ff = val;
	return 0;
}
/*
int setpoint_t::set_yaw_ff(double val)
{
	yaw_ff = val;
	return 0;
}
*/

int setpoint_t::set_X_dot(double val)
{
	X_dot = val;
	return 0;
}
int setpoint_t::set_Y_dot(double val)
{
	Y_dot = val;
	return 0;
}
int setpoint_t::set_Z_dot(double val)
{
	Z_dot = val;
	return 0;
}
int setpoint_t::set_X_dot_ff(double val)
{
	X_dot_ff = val;
	return 0;
}
int setpoint_t::set_Y_dot_ff(double val)
{
	Y_dot_ff = val;
	return 0;
}
int setpoint_t::set_Z_dot_ff(double val)
{
	Z_dot_ff = val;
	return 0;
}

int setpoint_t::set_X(double val)
{
	X = val;
	return 0;
}
int setpoint_t::set_Y(double val)
{
	Y = val;
	return 0;
}
int setpoint_t::set_Z(double val)
{
	Z = val;
	return 0;
}
/*
int setpoint_t::set_X_ff(double val)
{
	X_ff = val;
	return 0;
}
int setpoint_t::set_Y_ff(double val)
{
	Y_ff = val;
	return 0;
}
int setpoint_t::set_Z_ff(double val)
{
	Z_ff = val;
	return 0;
}
*/


/**
* @brief      Reset setpoints to state_estimate.
*
* @return     0 on success, -1 on failure
*/
int setpoint_t::reset_roll_dot(void)
{
	return set_roll_dot(state_estimate.roll_dot);
}
int setpoint_t::reset_pitch_dot(void)
{
	return set_pitch_dot(state_estimate.pitch_dot);
}
int setpoint_t::reset_yaw_dot(void)
{
	return set_yaw_dot(state_estimate.yaw_dot);
}
int setpoint_t::reset_roll_dot_ff(void)
{
	return set_roll_dot_ff(0.0);
}
int setpoint_t::reset_pitch_dot_ff(void)
{
	return set_pitch_dot_ff(0.0);
}
int setpoint_t::reset_yaw_dot_ff(void)
{
	return set_yaw_dot_ff(0.0);
}

int setpoint_t::reset_att_dot(void)
{
	reset_roll_dot();
	reset_pitch_dot();
	reset_yaw_dot();

	return 0;
}
int setpoint_t::reset_att_dot_ff(void)
{
	reset_roll_dot_ff();
	reset_pitch_dot_ff();
	reset_yaw_dot_ff();

	return 0;
}
int setpoint_t::reset_att_dot_all(void)
{
	reset_att_dot();
	reset_att_dot_ff();

	return 0;
}


int setpoint_t::reset_roll(void)
{
	return set_roll(state_estimate.roll);
}
int setpoint_t::reset_pitch(void)
{
	return set_pitch(state_estimate.pitch);
}
int setpoint_t::reset_yaw(void)
{
	return set_yaw(state_estimate.continuous_yaw);
}
int setpoint_t::reset_roll_ff(void)
{
	return set_roll_ff(0.0);
}
int setpoint_t::reset_pitch_ff(void)
{
	return set_pitch_ff(0.0);
}
/*
int setpoint_t::reset_yaw_ff(void)
{
	return set_yaw(0.0);
}
*/

int setpoint_t::reset_att(void)
{
	reset_roll();
	reset_pitch();
	reset_yaw();

	return 0;
}
int setpoint_t::reset_att_ff(void)
{
	reset_roll_ff();
	reset_pitch_ff();
	//reset_yaw_ff();

	return 0;
}
int setpoint_t::reset_att_all(void)
{
	reset_att();
	reset_att_ff();

	return 0;
}

int setpoint_t::reset_X_dot(void)
{
	return set_X_dot(state_estimate.X_dot);
}
int setpoint_t::reset_Y_dot(void)
{
	return set_Y_dot(state_estimate.Y_dot);
}
int setpoint_t::reset_Z_dot(void)
{
	return set_Z_dot(state_estimate.Z_dot);
}
int setpoint_t::reset_X_dot_ff(void)
{
	return set_X_dot_ff(0.0);
}
int setpoint_t::reset_Y_dot_ff(void)
{
	return set_Y_dot_ff(0.0);
}
int setpoint_t::reset_Z_dot_ff(void)
{
	return set_Z_dot_ff(0.0);
}

int setpoint_t::reset_pos_dot(void)
{
	reset_X_dot();
	reset_Y_dot();
	reset_Z_dot();

	return 0;
}
int setpoint_t::reset_pos_dot_ff(void)
{
	reset_X_dot_ff();
	reset_Y_dot_ff();
	reset_Z_dot_ff();

	return 0;
}
int setpoint_t::reset_pos_dot_all(void)
{
	reset_pos_dot();
	reset_pos_dot_ff();

	return 0;
}

int setpoint_t::reset_X(void)
{
	return set_X(state_estimate.X);
}
int setpoint_t::reset_Y(void)
{
	return set_Y(state_estimate.Y);
}
int setpoint_t::reset_Z(void)
{
	return set_Z(state_estimate.Z);
}
/*
int setpoint_t::reset_X_ff(void)
{
	return set_X_ff(0.0);
}
int setpoint_t::reset_Y_ff(void)
{
	return set_Y_ff(0.0);
}
int setpoint_t::reset_Z_ff(void)
{
	return set_Z_ff(0.0);
}
*/
int setpoint_t::reset_pos(void)
{
	reset_X();
	reset_Y();
	reset_Z();

	return 0;
}
/*
int setpoint_t::reset_pos_ff(void)
{
	reset_X_ff();
	reset_Y_ff();
	reset_Z_ff();

	return 0;
}
*/
int setpoint_t::reset_pos_all(void)
{
	reset_pos();
	//reset_pos_ff();

	return 0;
}

int setpoint_t::reset_all(void)
{
	reset_pos_all();
	reset_pos_dot_all();
	reset_att_all();
	reset_att_dot_all();
}


int setpoint_t::update_setpoints(void)
{
	// finally, switch between flight modes and adjust setpoint properly
	switch (user_input.flight_mode) {


	case TEST_BENCH_4DOF:
		// configure which controllers are enabled
		en_6dof = false;
		en_rpy_rate_ctrl = false;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = false;
		en_rpy_trans = false;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;

		roll_throttle = user_input.get_roll_stick();
		pitch_throttle = user_input.get_pitch_stick();
		yaw_throttle = user_input.get_yaw_stick();
		X_throttle = 0.0;
		Y_throttle = 0.0;
		Z_throttle = -user_input.get_thr_stick();

		break;

	case TEST_BENCH_6DOF:
		// configure which controllers are enabled
		en_6dof = true;
		en_rpy_rate_ctrl = false;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = false;
		en_rpy_trans = false;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;


		roll_throttle = 0.0;
		pitch_throttle = 0.0;
		yaw_throttle = user_input.get_yaw_stick();
		X_throttle = -user_input.get_pitch_stick();
		Y_throttle = user_input.get_roll_stick();
		Z_throttle = -user_input.get_thr_stick();
		break;

	case TEST_6xSERVOS_DIRECT:
		// configure which controllers are enabled
		en_6dof = false;
		en_rpy_rate_ctrl = false;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = false;
		en_rpy_trans = false;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;


		roll_throttle = 0.0;
		pitch_throttle = 0.0;
		yaw_throttle = 0.0;
		X_throttle = 0.0;
		Y_throttle = 0.0;
		Z_throttle = 0.0;


		//servos:
		roll_servo_throttle = user_input.get_roll_stick();	//map [-1 1] into [0 1]
		pitch_servo_throttle = user_input.get_pitch_stick();	//map [-1 1] into [0 1]
		yaw_servo_throttle = user_input.get_yaw_stick();		//map [-1 1] into [0 1]
		Z_servo_throttle = -user_input.get_thr_stick();
		break;

	case ACRO:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = false;
		en_rpy_trans = false;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;

		update_rpy_rate();
		update_th();
		break;

	case MANUAL_S:
		en_6dof = false;
		en_rpy_rate_ctrl = false;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = true;
		en_rpy_trans = false;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;

		roll = user_input.get_roll_stick();
		pitch = user_input.get_pitch_stick();
		Z_throttle = -user_input.get_thr_stick() / \
			(cos(state_estimate.roll) * cos(state_estimate.pitch));

		update_yaw();
		break;

	case MANUAL_F:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = true;
		en_rpy_trans = false;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;


		update_rp();
		update_th();
		update_yaw();
		break;

	case DIRECT_THROTTLE_6DOF:
		en_6dof = true;
		en_rpy_rate_ctrl = false;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = true;
		en_rpy_trans = false;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;


		X_throttle = -user_input.get_pitch_stick();
		Y_throttle = user_input.get_roll_stick();
		Z_throttle = -user_input.get_thr_stick();
		update_yaw();
		break;

	case ALT_HOLD_SS:
		en_6dof = false;
		en_rpy_rate_ctrl = false;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = true;
		en_rpy_trans = false;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;

		update_rp();
		update_Z();
		update_yaw();
		break;

	case ALT_HOLD_FS:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = true;
		en_rpy_trans = false;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;


		update_rp();
		update_Z();
		update_yaw();
		break;

	case ALT_HOLD_FF:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = true;
		en_rpy_trans = false;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = true;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;


		update_rp();
		update_Z();
		update_yaw();
		break;

	case POSITION_CONTROL_SSS:
		en_6dof = false;
		en_rpy_rate_ctrl = false;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = true;
		en_rpy_trans = false;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = true;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;


		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();
		break;

	case POSITION_CONTROL_FSS:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = true;
		en_rpy_trans = false;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = true;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;


		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();
		break;

	case POSITION_CONTROL_FFS:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = true;
		en_rpy_trans = false;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = true;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = true;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;


		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();
		break;

	case POSITION_CONTROL_FFF:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = true;
		en_rpy_trans = false;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = true;
		en_XY_vel_ctrl = true;
		en_XY_pos_ctrl = true;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;


		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();
		break;

	case EMERGENCY_LAND:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = true;
		en_rpy_trans = false;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = true;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;


		//Assign Setpoints
		roll = 0;
		pitch = 0;

		setpoint_guidance.start_land();  // start landing algorithm

		update_yaw();
		break;

	case AUTONOMOUS:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = true;
		en_rpy_trans = false;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = true;
		en_XY_vel_ctrl = true;
		en_XY_pos_ctrl = true;

		en_6dof_servo = false;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = false;


		waypoint_state_machine.enable_update();

		break;

	case ZEPPELIN:
		en_6dof = false;
		en_rpy_rate_ctrl = false;
		en_rpy_rate_trans = false;
		en_rpy_ctrl = false;
		en_rpy_trans = true;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		en_6dof_servo = true;
		en_rpy_rate_servo_ctrl = false;
		en_rpy_rate_servo_trans = false;
		en_rpy_servo_ctrl = false;
		en_rpy_servo_trans = true;

		update_trans(); //update transitions, filters and trims

		roll_throttle = user_input.get_roll_stick() + roll_stick_trim;
		pitch_throttle = user_input.get_pitch_stick(); // pitch_stick_hp;
		yaw_throttle = user_input.get_yaw_stick() + yaw_stick_trim;
		Z_throttle = -user_input.get_thr_stick();
		
		roll_servo_throttle = user_input.get_roll_stick() + roll_stick_trim;
		pitch_servo_throttle = user_input.get_pitch_stick(); // pitch_stick_hp;
		yaw_servo_throttle = user_input.get_yaw_stick() + yaw_stick_trim;
		
		X_servo_throttle = user_input.get_yaw_stick() + yaw_stick_trim;
		Y_servo_throttle = pitch_servo_tr;
		Z_servo_throttle = 0.0;
		
		
		break;

	default: // should never get here
		fprintf(stderr, "ERROR in setpoint_manager thread, unknown flight mode\n");
		break;

	} // end switch(user_input.flight_mode)
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

	if (!user_input.is_initialized())
	{
		fprintf(stderr, "ERROR in setpoint_manager_update, input_manager not initialized yet\n");
		return -1;
	}

	// if PAUSED or UNINITIALIZED, do nothing
	if(rc_get_state()!=RUNNING) return 0;

	if (user_input.flight_mode != AUTONOMOUS) waypoint_state_machine.disable_update();

	// shutdown feedback on kill switch
	if (user_input.requested_arm_mode == DISARMED)
	{
		if (fstate.get_arm_state() != DISARMED) 
		{
			fstate.disarm();
			last_en_trans = false;
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
	update_setpoints(); //get manual radio updates first

	// Update the state machine if in autonomous operation
	if (user_input.flight_mode == AUTONOMOUS)
	{
		waypoint_state_machine.march();
		if (settings.log_benchmark) benchmark_timers.tSM = rc_nanos_since_boot();
	}
	setpoint_guidance.march();

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
