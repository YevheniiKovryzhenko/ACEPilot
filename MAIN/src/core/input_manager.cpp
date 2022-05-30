/*
 * input_mannager.cpp
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
 * Last Edit:  05/29/2022 (MM/DD/YYYY)
 *
 * Functions to start and stop the input manager thread which is the translation
 * beween control inputs from DSM to the user_input struct which is read by the
 * setpoint manager.
 */

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <math.h> // for fabs

#include <rc/start_stop.h>
#include <rc/pthread.h>
#include <rc/time.h>
#include <rc/dsm.h>
#include <rc/math/other.h>

#include "setpoint_manager.hpp"
#include "settings.h"
#include "rc_pilot_defs.h"
#include "flight_mode.h"
#include "state_estimator.h"
#include "rc_pilot_defs.h"
#include "thread_defs.h"
#include "feedback.hpp"

#include "input_manager.hpp"

 // preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

double stick_t::get(void)
{
	return value;
}
int stick_t::set(double in)
{
	value = in;
	return 0;
}
int stick_t::reset(void)
{
	value = 0.0;
	return 0;
}


 /*
 * Invoke defaut constructor for all the built in and exernal types in the class
 */
user_input_t user_input{}; // extern variable in input_manager.hpp

/**
* float apply_deadzone(float in, float zone)
*
* Applies a dead zone to an input stick in. in is supposed to range from -1 to 1
* the dead zone is centered around 0. zone specifies the distance from 0 the
* zone extends.
**/
static double __deadzone(double in, double zone)
{
	if(zone<=0.0) return in;
	if(fabs(in)<=zone) return 0.0;
	if(in>0.0)	return ((in-zone)/(1.0-zone));
	else		return ((in+zone)/(1.0-zone));
}

static bool in_arming_position()
{
	return rc_dsm_ch_normalized(settings.dsm_thr_ch) < 0.1 &&
		   rc_dsm_ch_normalized(settings.dsm_pitch_ch) < -0.9 &&
		   ((rc_dsm_ch_normalized(settings.dsm_yaw_ch) > 0.9 &&
			 rc_dsm_ch_normalized(settings.dsm_roll_ch) < -0.9) ||
			(rc_dsm_ch_normalized(settings.dsm_yaw_ch) < -0.9 &&
			 rc_dsm_ch_normalized(settings.dsm_roll_ch) > 0.9));
}

static bool in_centered_position()
{
	return fabs(rc_dsm_ch_normalized(settings.dsm_thr_ch)) < 0.1 &&
		   fabs(rc_dsm_ch_normalized(settings.dsm_pitch_ch)) < 0.1 &&
		   fabs(rc_dsm_ch_normalized(settings.dsm_yaw_ch)) < 0.1 &&
		   fabs(rc_dsm_ch_normalized(settings.dsm_roll_ch)) < 0.1;
}

typedef struct disconnect_timer_t
{
	uint64_t time_start;
	int active;
} disconnect_timer_t;

void* input_manager(__attribute__((unused)) void* ptr)
{
	user_input.update();
	return NULL;
}

void __new_dsm_data_callback()
{
	user_input.new_dsm_data_callback();
}

void __dsm_disconnect_callback(void)
{
	user_input.dsm_disconnect_callback();
}

// returns true on success
static bool attempt_arming()
{
	// already armed, just return. Should never do this in normal operation though
	if (user_input.requested_arm_mode == ARMED) return 0;

	while(!fstate.is_initialized()){
		rc_usleep(100000);
		if (rc_get_state() == EXITING) return false;
	}
	// wait for level
	while (fabs(state_estimate.roll) > ARM_TIP_THRESHOLD ||
		   fabs(state_estimate.pitch) > ARM_TIP_THRESHOLD) 
	{
		rc_usleep(100000);
		if (rc_get_state() == EXITING) return false;
	}
	// wait for kill switch to be switched to ARMED
	while (user_input.get_arm_switch() == DISARMED) 
	{
		rc_usleep(100000);
		if (rc_get_state() == EXITING) return false;
	}
	// wait for throttle down-left/down-right or down-right/down-left
	while (!in_arming_position()) 
	{
		rc_usleep(100000);
		if (rc_get_state() == EXITING || user_input.get_arm_switch() == DISARMED) return false;
	}
	// wait for 3 seconds to pass
	for (int i = 0; i < 30; i++) 
	{
		if (!in_arming_position()) return false;
		rc_usleep(100000);
		if (rc_get_state() == EXITING || user_input.get_arm_switch() == DISARMED) return false;
	}

	// wait for sticks to return to center
	while (!in_centered_position()) 
	{
		rc_usleep(100000);
		if (rc_get_state() == EXITING || user_input.get_arm_switch() == DISARMED) return false;
	}

	// final check of kill switch and level before arming
	if (user_input.get_arm_switch() == DISARMED ||
		fabs(state_estimate.roll) > ARM_TIP_THRESHOLD ||
		fabs(state_estimate.pitch) > ARM_TIP_THRESHOLD) {
		return false;
	}
	return true;
}

/**
 * @brief      blocking function that returns after arming sequence is complete
 *
 * @return     0 if successful or already armed, -1 if exiting or problem
 */
int user_input_t::wait_for_arming_sequence(void)
{
	// already armed, just return. Should never do this in normal operation though
	if(requested_arm_mode == ARMED) return 0;

	while (rc_get_state() != EXITING) {
		if (attempt_arming()) {
			return 0;
		}
	}

	return -1;
}

void user_input_t::dsm_disconnect_callback(void)
{
	reset_sticks();
	input_active = 0;
	set_arm_switch(DISARMED);
	requested_arm_mode = DISARMED;
	fprintf(stderr, "LOST DSM CONNECTION\n");
}

int user_input_t::new_dsm_data_callback(void)
{
	double new_kill;

	// Read normalized (+-1) inputs from RC radio stick and multiply by
	// polarity setting so positive stick means positive setpoint
	double new_thr = rc_dsm_ch_normalized(settings.dsm_thr_ch) * settings.dsm_thr_pol;
	double new_roll = rc_dsm_ch_normalized(settings.dsm_roll_ch) * settings.dsm_roll_pol;
	double new_pitch = rc_dsm_ch_normalized(settings.dsm_pitch_ch) * settings.dsm_pitch_pol;
	double new_yaw = __deadzone(rc_dsm_ch_normalized(settings.dsm_yaw_ch) * settings.dsm_yaw_pol, YAW_DEADZONE);
	double new_mode = rc_dsm_ch_normalized(settings.dsm_mode_ch) * settings.dsm_mode_pol;


	// kill mode behaviors
	switch (settings.dsm_kill_mode) {
	case DSM_KILL_DEDICATED_SWITCH:
		new_kill = rc_dsm_ch_normalized(settings.dsm_kill_ch) * settings.dsm_kill_pol;
		// determine the kill state
		if (new_kill <= 0.1) {
			set_arm_switch(DISARMED);
			requested_arm_mode = DISARMED;
		}
		else {
			set_arm_switch(ARMED);
		}
		break;

	case DSM_KILL_NEGATIVE_THROTTLE:
		if (new_thr <= -1.1) {
			set_arm_switch(DISARMED);
			requested_arm_mode = DISARMED;
		}
		else    set_arm_switch(ARMED);
		break;

	default:
		fprintf(stderr, "ERROR in input manager, unhandled settings.dsm_kill_mode\n");
		return 0;
	}

	// saturate the sticks to avoid possible erratic behavior
	// throttle can drop below -1 so extend the range for thr
	rc_saturate_double(&new_thr, -1.0, 1.0);
	rc_saturate_double(&new_roll, -1.0, 1.0);
	rc_saturate_double(&new_pitch, -1.0, 1.0);
	rc_saturate_double(&new_yaw, -1.0, 1.0);
	rc_saturate_double(&new_mode, -1.0, 1.0);

	// pick flight mode
	switch (settings.num_dsm_modes) {
	case 1:
		flight_mode = settings.flight_mode_1;
		break;
	case 2:
		// switch will either range from -1 to 1 or 0 to 1.
		// in either case it's safe to use +0.5 as the cutoff
		if (new_mode < 0.5 * settings.dsm_mode_pol) flight_mode = settings.flight_mode_2;
		else flight_mode = settings.flight_mode_1;
		break;
	case 3:
		// 3-position switch will have the positions 0, 0.5, 1 when
		// calibrated correctly. checking +- 0.5 is a safe cutoff
		if (new_mode < 0.45 * settings.dsm_mode_pol) flight_mode = settings.flight_mode_3;
		else if (new_mode > 0.55 * settings.dsm_mode_pol) flight_mode = settings.flight_mode_1;
		else flight_mode = settings.flight_mode_2;
		break;
	default:
		fprintf(stderr, "ERROR, in input_manager, num_dsm_modes must be 1,2 or 3\n");
		fprintf(stderr, "selecting flight mode 1\n");
		flight_mode = settings.flight_mode_1;
		break;
	}

	//If we need MOCAP and it is no longer available and the user has indicated
	//they want an emergency landing in case of dropouts, 
	//then manage switching into and out of to OPEN_LOOP_DESCENT
	if (mode_needs_mocap(flight_mode) &&
		settings.enable_mocap_dropout_emergency_land)
	{
		//Compute time since last MOCAP packet was received (in milliseconds)
		// Each value here must be cast to a double before diving. 
		// Although I expect this to implicitly cast, the testing shows that OPEN_LOOP_DESCENT "randomly" triggers at inappropriate times
		double ms_since_mocap = ((double)rc_nanos_since_boot() - (double)state_estimate.mocap_timestamp_ns) / 1e6;

		//If MOCAP has been out for too long, then enable emergency landing
		if (!is_emergency_land_active() &&
			ms_since_mocap >= settings.mocap_dropout_timeout_ms &&
			requested_arm_mode == ARMED)
		{
			//Enable Emergency landing. 
			//This extra check is done so that we don't exit emergency landing if MOCAP becomes available suddenly
			emergency_land(true);

			fprintf(stderr, "ENABLE EMERGENCY LANDING MODE\n");
			fprintf(stderr, "\tMOCAP has been gone for %lfms which exeeds the limit %lfms\n", ms_since_mocap, settings.mocap_dropout_timeout_ms);
		}

		//Force flight mode to be EMERGENCY_LAND as long as emergency landing is enables
		if (user_input.is_emergency_land_active())
		{
			flight_mode = EMERGENCY_LAND;
		}

	}
	else
	{

		//Turn off emergency landing if we enter a mode that does NOT need MOCAP
		user_input.emergency_land(false);
	}

	// fill in sticks
	if (requested_arm_mode == ARMED) {
		throttle.set(new_thr);
		roll.set(new_roll);
		pitch.set(new_pitch);
		yaw.set(new_yaw);
		requested_flight_mode.set(new_mode);
		requested_arm_mode = get_arm_switch();
	}
	else {
		// during arming sequence keep sticks zeroed
		user_input.reset_sticks();
	}

	if (user_input.input_active == 0) {
		user_input.input_active = 1; // flag that connection has come back online
		printf("DSM CONNECTION ESTABLISHED\n");
	}
	return 0;
}

void user_input_t::reset_sticks(void)
{
	throttle.reset();
	roll.reset();
	pitch.reset();
	yaw.reset();
	requested_flight_mode.reset();
}

int user_input_t::update(void)
{
	disconnect_timer_t disconnect_timer;
	disconnect_timer.active = 0;

	user_input.set_initialized(true);
	// wait for first packet
	while (rc_get_state() != EXITING) {
		if (user_input.input_active) break;
		rc_usleep(1000000 / INPUT_MANAGER_HZ);
	}

	// not much to do since the DSM callbacks to most of it. Later some
	// logic to handle other inputs such as mavlink/bluetooth/wifi
	while (rc_get_state() != EXITING) {
		// if the core got disarmed, wait for arming sequence
		if (user_input.requested_arm_mode != ARMED) {
			wait_for_arming_sequence();
			// user may have pressed the pause button or shut down while waiting
			// check before continuing
			if (rc_get_state() != RUNNING) continue;
			else {
				user_input.requested_arm_mode = ARMED;
				//printf("\n\nDSM ARM REQUEST\n\n");
			}
		}
		// check if disconnected
		if (user_input.input_active == 0)
		{
			// check if timer started
			if (disconnect_timer.active == 0)
			{
				// if timer has not started, set start time to
				// current time and set the timer to active
				disconnect_timer.time_start = rc_nanos_since_boot();
				disconnect_timer.active = 1;
			}
			else
			{
				// if the timer has started, check if more that dsm_timout seconds
				// (dsm_timout_ms * 1e6) have elapsed
				if (rc_nanos_since_boot() >
					(disconnect_timer.time_start + settings.dsm_timeout_ms * 1e6))
				{
					// if too much time has elapsed, perform disconnect actions
					dsm_disconnect_callback();
					disconnect_timer.active = 0;  // deactivate timer for next disconnect
				}
			}
		}
		else
		{
			// if user input is reestablished, turn the timer off
			disconnect_timer.active = 0;
		}
		// wait
		rc_usleep(1000000 / INPUT_MANAGER_HZ);
	}
	return 0;
}

int user_input_t::init_all_filters(void)
{
	if (unlikely(throttle.filters.init_all() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize throttle stick filters\n");
		return -1;
	}
	if (unlikely(roll.filters.init_all() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize roll stick filters\n");
		return -1;
	}
	if (unlikely(pitch.filters.init_all() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize pitch stick filters\n");
		return -1;
	}
	if (unlikely(yaw.filters.init_all() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize yaw stick filters\n");
		return -1;
	}
	if (unlikely(requested_flight_mode.filters.init_all() == -1))
	{
		printf("ERROR in init_all_filters: failed to initialize requested_flight_mode stick filters\n");
		return -1;
	}
	
	return 0;
}

int user_input_t::input_manager_init(void)
{
	en_emergency_land = false;

	initialized = false;
	
	
	if (unlikely(init_all_filters() == -1))
	{
		fprintf(stderr, "ERROR in init: failed to initialize all filters\n");
		return -1;
	}
	
	int i;
	// start dsm hardware
	if (unlikely(rc_dsm_init() == -1))
	{
		fprintf(stderr, "ERROR in input_manager_init, failed to initialize dsm\n");
		return -1;
	}
	rc_dsm_set_disconnect_callback(__dsm_disconnect_callback);
	rc_dsm_set_callback(__new_dsm_data_callback);
	
	// start thread
	if(rc_pthread_create(&thread, &input_manager, NULL,
				SCHED_FIFO, INPUT_MANAGER_PRI)==-1){
		fprintf(stderr, "ERROR in input_manager_init, failed to start thread\n");
		return -1;
	}
	// wait for thread to start
	for (i = 0; i < 50; i++) {
		if (user_input.is_initialized()) return 0;
		rc_usleep(50000);
	}
	fprintf(stderr, "ERROR in input_manager_init, timeout waiting for thread to start\n");
	return -1;
}


/* Reading/Writing Control Sticks From Radio
* No other functions shold be changing sticks other
* than input mannager. Reading can be done externally.
*/
flight_mode_t user_input_t::get_flight_mode(void)
{
	return flight_mode;
}

/* Arm Stick */
arm_state_t user_input_t::get_arm_switch(void)
{
	return arm_switch;
}
void user_input_t::set_arm_switch(arm_state_t val)
{
	arm_switch = val;
	return;
}

/* Initialization */
bool user_input_t::is_initialized(void)
{
	return initialized;
}

void user_input_t::set_initialized(bool val)
{
	initialized = val;
	return;
}

/* Emergency Landning */
bool user_input_t::is_emergency_land_active(void)
{
	return en_emergency_land;
}

void user_input_t::emergency_land(bool val)
{
	en_emergency_land = val;
	return;
}

/* Safe Exit */
int user_input_t::input_manager_cleanup()
{

	if(initialized==0){
		fprintf(stderr, "WARNING in input_manager_cleanup, was never initialized\n");
		return -1;
	}
	// wait for the thread to exit
	if(rc_pthread_timed_join(thread, NULL, INPUT_MANAGER_TOUT)==1){
		fprintf(stderr,"WARNING: in input_manager_cleanup, thread join timeout\n");
		return -1;
	}
	// stop dsm
	rc_dsm_cleanup();
	return 0;
}
