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
 * Last Edit:  08/16/2020 (MM/DD/YYYY)
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
#include "xbee_receive.h"
#include "setpoint_guidance.hpp"
#include "tools.h"
#include "feedback.hpp"
#include "state_machine.hpp"

#include "setpoint_manager.hpp"

 /*
 * Invoke defaut constructor for all the built in and exernal types in the class
 */
setpoint_t setpoint{}; // extern variable in setpoint_manager.hpp

/***********************************/
/* Functions which should be called exterally to update setpoints based on radio input:*/
void setpoint_t::update_yaw(void)
{
	// if throttle stick is down all the way, probably landed, so
	// keep the yaw setpoint at current yaw so it takes off straight
	if(user_input.get_thr_stick() < 0.1){
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
	if(initialized){
		fprintf(stderr, "\nERROR in setpoint_manager_init, already initialized");
		return -1;
	}

	if (setpoint_guidance.init() == -1)
	{
		fprintf(stderr, "\nERROR in setpoint_manager_init, failed to initialize setpoint guidance");
		return -1;
	}

	initialized = true;
	return 0;
}
bool setpoint_t::is_initialized(void)
{
	return initialized;
}


int setpoint_t::update_setpoints(void)
{
	if (user_input.flight_mode != AUTONOMOUS) waypoint_state_machine.disable_update();

	// finally, switch between flight modes and adjust setpoint properly
	switch (user_input.flight_mode) {


	case TEST_BENCH_4DOF:
		// configure which controllers are enabled
		en_6dof = false;
		en_rpy_rate_ctrl = false;
		en_rpy_ctrl = false;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		roll_throttle = user_input.get_roll_stick();
		pitch_throttle = user_input.get_pitch_stick();
		yaw_throttle = user_input.get_yaw_stick();
		X_throttle = 0.0;
		Y_throttle = 0.0;
		Z_throttle = -user_input.get_thr_stick();
		// TODO add these two throttle modes as options to settings, I use a radio
		// with self-centering throttle so having 0 in the middle is safest
		// Z_throttle = -(user_input.thr_stick+1.0)/2.0;

		break;

	case TEST_BENCH_6DOF:
		// configure which controllers are enabled
		en_6dof = true;
		en_rpy_rate_ctrl = false;
		en_rpy_ctrl = false;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;


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
		en_rpy_ctrl = false;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;


		roll_throttle = 0.0;
		pitch_throttle = 0.0;
		yaw_throttle = 0.0;
		X_throttle = 0.0;
		Y_throttle = 0.0;
		Z_throttle = 0.0;


		//servos:
		roll_servo_throttle = (user_input.get_roll_stick() + 1.0) / 2.0;	//map [-1 1] into [0 1]
		pitch_servo_throttle = (user_input.get_pitch_stick() + 1.0) / 2.0;	//map [-1 1] into [0 1]
		yaw_servo_throttle = (user_input.get_yaw_stick() + 1.0) / 2.0;		//map [-1 1] into [0 1]
		Z_servo_throttle = user_input.get_thr_stick();
		break;

	case ACRO:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_ctrl = false;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		roll_dot = user_input.get_roll_stick() * MAX_ROLL_RATE;
		pitch_dot = user_input.get_pitch_stick() * MAX_PITCH_RATE;
		yaw_dot = user_input.get_yaw_stick() * MAX_YAW_RATE;
		Z_throttle = -user_input.get_thr_stick() / \
			(cos(state_estimate.roll) * cos(state_estimate.pitch));
		break;

	case MANUAL_S:
		en_6dof = false;
		en_rpy_rate_ctrl = false;
		en_rpy_ctrl = true;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		roll = user_input.get_roll_stick();
		pitch = user_input.get_pitch_stick();
		Z_throttle = -user_input.get_thr_stick() / \
			(cos(state_estimate.roll) * cos(state_estimate.pitch));

		update_yaw();
		break;

	case MANUAL_F:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_ctrl = true;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		roll = user_input.get_roll_stick();
		pitch = user_input.get_pitch_stick();
		Z_throttle = -user_input.get_thr_stick() / \
			(cos(state_estimate.roll) * cos(state_estimate.pitch));

		update_yaw();
		break;

	case DIRECT_THROTTLE_6DOF:
		en_6dof = true;
		en_rpy_rate_ctrl = false;
		en_rpy_ctrl = true;
		en_Z_ctrl = false;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		X_throttle = -user_input.get_pitch_stick();
		Y_throttle = user_input.get_roll_stick();
		Z_throttle = -user_input.get_thr_stick();
		update_yaw();
		break;

	case ALT_HOLD_SS:
		en_6dof = false;
		en_rpy_rate_ctrl = false;
		en_rpy_ctrl = true;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		roll = user_input.get_roll_stick();
		pitch = user_input.get_pitch_stick();

		update_Z();
		update_yaw();
		break;

	case ALT_HOLD_FS:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_ctrl = true;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		roll = user_input.get_roll_stick();
		pitch = user_input.get_pitch_stick();

		update_Z();
		update_yaw();
		break;

	case ALT_HOLD_FF:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_ctrl = true;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = true;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = false;

		roll = user_input.get_roll_stick();
		pitch = user_input.get_pitch_stick();

		update_Z();
		update_yaw();
		break;

	case POSITION_CONTROL_SSS:
		en_6dof = false;
		en_rpy_rate_ctrl = false;
		en_rpy_ctrl = true;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = true;

		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();

		//printf("\nM2  X=%f Y=%f Z=%f with altitude of %f \n",X,Y,Z,state_estimate.Z);
		break;

	case POSITION_CONTROL_FSS:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_ctrl = true;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = false;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = true;

		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();

		//printf("\nM2  X=%f Y=%f Z=%f with altitude of %f \n",X,Y,Z,state_estimate.Z);
		break;

	case POSITION_CONTROL_FFS:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_ctrl = true;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = true;
		en_XY_vel_ctrl = false;
		en_XY_pos_ctrl = true;

		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();

		//printf("\nM2  X=%f Y=%f Z=%f with altitude of %f \n",X,Y,Z,state_estimate.Z);
		break;

	case POSITION_CONTROL_FFF:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_ctrl = true;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = true;
		en_XY_vel_ctrl = true;
		en_XY_pos_ctrl = true;

		//check validity of the velocity command, construct virtual setpoint
		update_XY_pos();
		update_Z();
		update_yaw();

		//printf("\nM2  X=%f Y=%f Z=%f with altitude of %f \n",X,Y,Z,state_estimate.Z);
		break;

	case AUTONOMOUS:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_ctrl = true;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = true;
		en_XY_vel_ctrl = true;
		en_XY_pos_ctrl = true;

		waypoint_state_machine.enable_update();

		break;

	case EMERGENCY_LAND:
		en_6dof = false;
		en_rpy_rate_ctrl = true;
		en_rpy_ctrl = true;
		en_Z_ctrl = true;
		en_Z_rate_ctrl = true;
		en_XY_vel_ctrl = true;
		en_XY_pos_ctrl = true;

		//Assign Setpoints
		roll = 0;
		pitch = 0;

		setpoint_guidance.start_land();  // start landing algorithm

		update_yaw();
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

	// shutdown feedback on kill switch
	if (user_input.requested_arm_mode == DISARMED)
	{
		if (fstate.get_arm_state() != DISARMED) fstate.disarm(), \
			setpoint_guidance.reset_Z(), \
			setpoint_guidance.reset_XY();
		return 0;
	}

	// arm feedback when requested
	if (user_input.requested_arm_mode == ARMED) {
		if (fstate.get_arm_state() == DISARMED) fstate.arm(), \
			setpoint_guidance.reset_Z(), \
			setpoint_guidance.reset_XY();
	}
	update_setpoints();
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
