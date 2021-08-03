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
 * Last Edit:  08/2/2020 (MM/DD/YYYY)
 *
 * Summary :
 * Setpoint manager runs at the same rate as the feedback controller
 * and is the interface between the user inputs (input manager) and
 * the feedback controller setpoint. currently it contains very
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
#include "path.h"
#include "state_estimator.h"
#include "flight_mode.h"
#include "xbee_receive.h"
#include "trajectories_common.hpp"
#include "tools.h"
#include "feedback.hpp"

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
		tmp_X_dot = -user_input.get_pitch_stick() * settings.max_XY_velocity;

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
		tmp_Y_dot = user_input.get_roll_stick() * settings.max_XY_velocity;

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
		fprintf(stderr, "ERROR in setpoint_manager_init, already initialized\n");
		return -1;
	}

	initialized = true;
	return 0;
}
bool setpoint_t::is_initialized(void)
{
	return initialized;
}

/**
* @brief   Externally order setpoint manager to follow new path from path_file
*
* @return  0 on success, -1 if unsuccessful
*/
int setpoint_t::set_new_path(const char* file_name)
{
    if (path_load_from_file(file_name) == -1)
    {
        fprintf(stderr, "ERROR: could not load new path file\n");
        return -1;
    }

    reset_waypoint_counter();
    return 0;
}



/**
* @brief   Logic for starting to follow path, reset time and waypoint counter
*/
void setpoint_t::start_waypoint_counter()
{
    // If this is the first time in autonomous mode and armed, save the current time
    if (!auto_armed_set)
    {
        // If the system is armed and autonomous mode is set, record time in
        // time_auto_set
        auto_armed_set = true;
        time_auto_set = rc_nanos_since_boot();

		path.waypoints_init.x		= X;
        path.waypoints_init.y		= Y;
        path.waypoints_init.z		= Z;
        path.waypoints_init.roll	= roll;
        path.waypoints_init.pitch	= pitch;
        if (fabs(yaw - state_estimate.continuous_yaw) >= M_PI/18.0)
        {
            yaw = state_estimate.continuous_yaw;
            printf("\nWARNING: High yaw error, overwriting setpoint");
		}
		
        path.waypoints_init.yaw = yaw;
    }

    waypoint_time = finddt_s(time_auto_set);
	return;
}

void setpoint_t::stop_waypoint_counter()
{
    // If the system is disarmed or out of auto reset the auto_armed_set flag
    // and change the current waytpoint to zero
    auto_armed_set = false;
    cur_waypoint_num = 0;
	return;
}

void setpoint_t::reset_waypoint_counter()
{
    stop_waypoint_counter();
    start_waypoint_counter();
	return;
}


/**
* @brief Update the setpoint for the next waypoint
*/
void setpoint_t::update_setpoint_from_waypoint()
{
    start_waypoint_counter();
    // Break out of function if the current waypoint is the last point in the path
    //printf("\n cur_waypoint_num = %" PRId64 "", cur_waypoint_num);
    if (cur_waypoint_num == path.len)
    {
        return;
    }

    // Parse waypoint flag
    //printf("\n path.waypoints[cur_waypoint_num].flag = %d",
    //    path.waypoints[cur_waypoint_num].flag);
    switch ((int)path.waypoints[cur_waypoint_num].flag)
    {
        case TIME_TRANSITION_FLAG:
            // Check if there are additional waypoints and advnace control
            // to the next waytoint if it is time to do so.  If there are no additional waypoints,
            // keep controlling to the previous point
            if (cur_waypoint_num < (path.len - 1) &&
                waypoint_time >= path.waypoints[cur_waypoint_num + 1].t)
            {
                ++cur_waypoint_num;

				// Set the desired x, y, and z if allowed
                if (en_waypoint_update)
                {
					X		= path.waypoints_init.x		+ path.waypoints[cur_waypoint_num].x;
					Y		= path.waypoints_init.y		+ path.waypoints[cur_waypoint_num].y;
					Z		= path.waypoints_init.z		+ path.waypoints[cur_waypoint_num].z;
					roll	= path.waypoints_init.roll	+ path.waypoints[cur_waypoint_num].roll;
					pitch	= path.waypoints_init.pitch + path.waypoints[cur_waypoint_num].pitch;
					yaw		= path.waypoints_init.yaw	+ path.waypoints[cur_waypoint_num].yaw;
				}
                
            }
            if (cur_waypoint_num >= (path.len - 1))
            {
                //printf("\nDisable wp");
                en_waypoint_update = false;
			}
            break;
        case POS_TRANSITION_FLAG:
            // TODO: determine position error and compare to convergence tolerance
            //       (? who sets/determines/stores convergence tolerance ?)
            assert(0);
            break;
        default:
            fprintf(stderr, "ERROR: unrecognized waypoint flag\n");
    }
    //printf("\n waypoint_time = %f time = %f",
    //    waypoint_time, path.waypoints[cur_waypoint_num + 1].t);
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
		if (fstate.get_arm_state() != DISARMED) fstate.disarm();
		return 0;
	}

	// arm feedback when requested
	if (user_input.requested_arm_mode == ARMED) {
		if (fstate.get_arm_state() == DISARMED) fstate.arm();
	}


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
