/*
 * feedback.cpp
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
 * Last Edit:  07/29/2020 (MM/DD/YYYY)
 *
 * Summary :
 * Here lies the heart and soul of the operation. feedback_init(void) pulls
 * in the control constants from settings module and sets up the discrete
 * controllers. From then on out, feedback_march(void) should be called by the
 * IMU interrupt at feedback_hz until the program is shut down.
 * feedback_march(void) will monitor the setpoint which is constantly being
 * changed by setpoint_manager(void).
 *
 */

#include <stdint.h> // for uint64_t
#include <stdio.h>
#include <math.h>

#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/quaternion.h>
#include <rc/math/other.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/servo.h>
#include <rc/time.h>

#include "state_estimator.h"
#include "rc_pilot_defs.h"
#include "setpoint_manager.hpp"
#include "settings.h"
#include "mix.h"
#include "thrust_map.h"
#include "xbee_receive.h"
#include "tools.h"
#include "controller.hpp"
#include "log_manager.hpp"
#include "input_manager.hpp"
#include "trajectories_common.hpp"
#include "servos.hpp"

#include "feedback.hpp"

 // preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

 /*
 * Invoke defaut constructor for all the built in and exernal types in the class
 */
feedback_state_t fstate{}; // extern variable in feedback.hpp


int feedback_state_t::send_motor_stop_pulse(void)
{
	int i;
	if (unlikely(settings.num_rotors > 8))
	{
		printf("ERROR: set_motors_to_idle: too many rotors\n");
		return -1;
	}
	for (i = 0; i < settings.num_rotors; i++) {
		m[i] = -0.1;
		rc_servo_send_esc_pulse_normalized(i + 1, -0.1);
	}
	return 0;
}


/**
* @brief      This is how outside functions should stop the flight controller.
*
*             It would be reasonable to set motors to 0 here, but since this
*             function can be called from anywhere that might produce
*             conflicts. Instead the interrupt service routine will do this on
*             the next loop after disarming to maintain timing of pulses to the
*             motors
*
* @return     0 on success, -1 on failure
*/
int feedback_state_t::disarm(void)
{
	arm_state = DISARMED;
	// set LEDs
	rc_led_set(RC_LED_RED, 1);
	rc_led_set(RC_LED_GREEN, 0);

	if (settings.enable_servos)
	{
		if (unlikely(sstate.disarm() == -1))
		{
			printf("\nERROR in disarm: failed to disarm servos");
			return -1;
		}
	}

	if (settings.log_only_while_armed)
	{
		log_entry.cleanup();
	}
	return 0;
}


/**
* @brief      This is how outside functions should start the flight controller.
*
* @return     0 on success, -1 on failure
*/
int feedback_state_t::arm(void)
{
	if (unlikely(!initialized))
	{
		printf("\nERROR in arm: feedback state not initialized");
		return -1;
	}

	//printf("\n Arming!\n");
	if (unlikely(arm_state == ARMED)) {
		printf("WARNING: trying to arm when controller is already armed\n");
		return 0;
	}
	// start a new log file every time controller is armed, this may take some
	// time so do it before touching anything else
	if (settings.enable_logging)
	{
		if (unlikely(log_entry.init() == -1))
		{
			printf("\nERROR in arm: failed to start new log entry");
			return -1;
		}
	}


	if (settings.enable_servos)
	{
		if (unlikely(sstate.arm() == -1))
		{
			printf("\nERROR in arm: failed to arm servos");
			return -1;
		}
	}
	
	// get the current time
	arm_time_ns = rc_nanos_since_boot();
	// reset the index
	loop_index = 0;

	controller.reset();
	
	printf("\n WARNING: Waking up the ESCs....");
	do
	{
		send_motor_stop_pulse();
	} while (finddt_s(arm_time_ns) < settings.arm_time_s);
	printf("\n Done! Wake up time was: %f ", finddt_s(arm_time_ns));
	
	// set LEDs
	rc_led_set(RC_LED_RED,0);
	rc_led_set(RC_LED_GREEN,1);
	// last thing is to flag as armed
	arm_state = ARMED;
	return 0;
}


/**
* @brief      Initial setup of all feedback controllers. Should only be called
*             once on program start.
*
* @param      setpoint  pointer to global setpoint struct
* @param      settings  pointer to global settings struct
*
* @return     0 on success, -1 on failure
*/
int feedback_state_t::init(void)
{
	if (unlikely(initialized))
	{
		printf("\nWARNING in init: feedback state already initialized");
		return 0;
	}

	arm_time_ns = 0;

	if (unlikely(controller.init() == -1))
	{
		printf("\nERROR in init: failed to initialize the controllers");
		return -1;
	}
	
	// make sure everything is disarmed then start the ISR
	if (unlikely(disarm() == -1))
	{
		printf("\nERROR in init: failed to disarm");
		return -1;
	}

	if (settings.enable_servos)
	{
		// make sure everything is disarmed then start the ISR
		if (unlikely(sstate.disarm() == -1))
		{
			printf("\nERROR in init: failed to disarm servos");
			return -1;
		}
	}
	
	

	initialized = true;
	return 0;
}

int feedback_state_t::update_gains(void)
{
	/*Update gains using xbee. Use xbeeMsg.GainCH for swithing
	between channels, assume 0 is the default mode of operation
	with the original gains.
	*/
	/*
	if(xbeeMsg.GainCH == 1){
		D_roll.num.d[0] = xbeeMsg.GainN0;
		D_roll.num.d[1] = xbeeMsg.GainN1;
		D_roll.num.d[2] = xbeeMsg.GainN2;
		D_roll.den.d[0] = xbeeMsg.GainD0;
		D_roll.den.d[1] = xbeeMsg.GainD1;
		D_roll.den.d[2] = xbeeMsg.GainD2;

		}
	else if(xbeeMsg.GainCH == 2){
		D_pitch.num.d[0] = xbeeMsg.GainN0;
		D_pitch.num.d[1] = xbeeMsg.GainN1;
		D_pitch.num.d[2] = xbeeMsg.GainN2;
		D_pitch.den.d[0] = xbeeMsg.GainD0;
		D_pitch.den.d[1] = xbeeMsg.GainD1;
		D_pitch.den.d[2] = xbeeMsg.GainD2;

		}
	else if(xbeeMsg.GainCH == 3){
		D_yaw.num.d[0] = xbeeMsg.GainN0;
		D_yaw.num.d[1] = xbeeMsg.GainN1;
		D_yaw.num.d[2] = xbeeMsg.GainN2;
		D_yaw.den.d[0] = xbeeMsg.GainD0;
		D_yaw.den.d[1] = xbeeMsg.GainD1;
		D_yaw.den.d[2] = xbeeMsg.GainD2;

		}
	else if(xbeeMsg.GainCH == 4){
		D_Z.num.d[0] = xbeeMsg.GainN0;
		D_Z.num.d[1] = xbeeMsg.GainN1;
		D_Z.num.d[2] = xbeeMsg.GainN2;
		D_Z.den.d[0] = xbeeMsg.GainD0;
		D_Z.den.d[1] = xbeeMsg.GainD1;
		D_Z.den.d[2] = xbeeMsg.GainD2;

	}
	else if(xbeeMsg.GainCH == 5){
		D_X_4.num.d[0] = xbeeMsg.GainN0;
		D_X_4.num.d[1] = xbeeMsg.GainN1;
		D_X_4.num.d[2] = xbeeMsg.GainN2;
		D_X_4.den.d[0] = xbeeMsg.GainD0;
		D_X_4.den.d[1] = xbeeMsg.GainD1;
		D_X_4.den.d[2] = xbeeMsg.GainD2;

	}
	else if(xbeeMsg.GainCH == 6){
		D_Y_4.num.d[0] = xbeeMsg.GainN0;
		D_Y_4.num.d[1] = xbeeMsg.GainN1;
		D_Y_4.num.d[2] = xbeeMsg.GainN2;
		D_Y_4.den.d[0] = xbeeMsg.GainD0;
		D_Y_4.den.d[1] = xbeeMsg.GainD1;
		D_Y_4.den.d[2] = xbeeMsg.GainD2;

	}
	else{
		//use original pid gains defined in the settings file:
	}
	*/
	return 0;
}



/**
* @brief      marches feedback controller forward one step
*
* This is called AFTER state_estimator_march and actually sends signals to the
* motors. This can as is still safely called when Disarmed.
*
* @return     0 on success, -1 on failure
*/
int feedback_state_t::march(void)
{
	if (unlikely(!initialized))
	{
		printf("\nERROR in march: feedback state not initialized");
		disarm();
		return -1;
	}


	int i;
	double tmp_u[MAX_INPUTS], tmp_mot[MAX_ROTORS], tmp_s[MAX_INPUTS], tmp_servos[MAX_SERVOS];

	// Disarm if rc_state is somehow paused without disarming the controller.
	// This shouldn't happen if other threads are working properly.
	if (unlikely(rc_get_state() != RUNNING && arm_state == ARMED))
	{
		disarm();
		printf("\n rc_state is somehow paused \n");
	}

	// check for a tipover
	if (unlikely(fabs(state_estimate.roll) > TIP_ANGLE || fabs(state_estimate.pitch) > TIP_ANGLE)) 
	{
		disarm();
		printf("\n TIPOVER DETECTED \n");
	}

	// if not running or not armed, keep the motors in an idle state
	if (rc_get_state() != RUNNING || arm_state == DISARMED)
	{
		send_motor_stop_pulse();
		return 0;
	}

	update_setpoints();

	if (settings.enable_dynamic_gains)
	{
		update_gains(); // work in progress
	}

	// We are about to start marching the individual SISO controllers forward.
	// Start by zeroing out the motors signals then add from there.
	for (i = 0; i < MAX_ROTORS; i++) tmp_mot[i] = 0.0;
	for (i = 0; i < MAX_INPUTS; i++) tmp_u[i] = 0.0;

	if (settings.enable_servos)
	{
		for (i = 0; i < MAX_SERVOS; i++) tmp_servos[i] = 0.0;
		for (i = 0; i < MAX_INPUTS; i++) tmp_s[i] = 0.0;

		//we need to march a dedicated servo controller here
		/*
		if (servo_controller.march(tmp_s, tmp_servos) == -1)
		{
			printf("\nERROR in march: failed to march the servo controller");
			disarm();
		}
		*/
	}
	

	if (controller.march(tmp_u, tmp_mot) == -1)
	{
		printf("\nERROR in march: failed to march the controller");
		disarm();
	}

	/***************************************************************************
	* Send ESC motor signals immediately at the end of the control loop
	***************************************************************************/
	for (i = 0; i < settings.num_rotors; i++)
	{
		rc_saturate_double(&tmp_mot[i], 0.0, 1.0);
		m[i] = map_motor_signal(tmp_mot[i]);

		// final saturation just to take care of possible rounding errors
		// this should not change the values and is probably excessive
		rc_saturate_double(&m[i], 0.0, 1.0);

		// finally send pulses!
		rc_servo_send_esc_pulse_normalized(i + 1, m[i]);
		
	}

	if (settings.enable_servos)
	{
		for (int i = 0; i < settings.num_servos; i++)
		{
			rc_saturate_double(&tmp_servos[i], 0.0, 1.0);
			//need to do control allocation here:
			//tmp_s[i] = map_servo_signal(tmp_servos[i]);

			// final saturation just to take care of possible rounding errors
			// this should not change the values and is probably excessive
			rc_saturate_double(&tmp_s[i], 0.0, 1.0);
			//send mapped into [0 1] control signals to servos:
			sstate.march(i, tmp_s[i]);
		}
	}

	/***************************************************************************
	* Final cleanup, timing, and indexing
	***************************************************************************/
	// Load control inputs into cstate for viewing by outside threads
	for (i = 0; i < 6; i++) u[i] = tmp_u[i];
	// keep track of loops since arming
		loop_index++;
	// log us since arming, mostly for the log
	last_step_ns = rc_nanos_since_boot();

	return 0;
}

int feedback_state_t::zero_out_ff(void)
{
	setpoint.roll_dot_ff = 0;
	setpoint.pitch_dot_ff = 0;
	setpoint.yaw_dot_ff = 0;
	setpoint.roll_ff = 0;
	setpoint.pitch_ff = 0;
	setpoint.Z_dot_ff = 0;
	setpoint.X_dot_ff = 0;
	setpoint.Y_dot_ff = 0;
	return 0;
}

int feedback_state_t::update_setpoints(void)
{
	// Zero out feedforward terms so unexpected things don't happen
	zero_out_ff();


	// finally, switch between flight modes and adjust setpoint properly
	switch (user_input.flight_mode) {


	case TEST_BENCH_4DOF:
		// configure which controllers are enabled
		setpoint.en_6dof = 0;
		setpoint.en_rpy_ctrl = 0;
		setpoint.en_Z_ctrl = 0;
		setpoint.en_XY_vel_ctrl = 0;
		setpoint.en_XY_pos_ctrl = 0;

		setpoint.roll_throttle = user_input.get_roll_stick();
		setpoint.pitch_throttle = user_input.get_pitch_stick();
		setpoint.yaw_throttle = user_input.get_yaw_stick();
		setpoint.Z_throttle = -user_input.get_thr_stick();
		// TODO add these two throttle modes as options to settings, I use a radio
		// with self-centering throttle so having 0 in the middle is safest
		// setpoint.Z_throttle = -(user_input.thr_stick+1.0)/2.0;
		break;

	case TEST_BENCH_6DOF:
		setpoint.en_6dof = 1;
		setpoint.en_rpy_ctrl = 0;
		setpoint.en_Z_ctrl = 0;
		setpoint.en_XY_vel_ctrl = 0;
		setpoint.en_XY_pos_ctrl = 0;

		setpoint.X_throttle = -user_input.get_pitch_stick();
		setpoint.Y_throttle = user_input.get_roll_stick();
		setpoint.roll_throttle = 0.0;
		setpoint.pitch_throttle = 0.0;
		setpoint.yaw_throttle = user_input.get_yaw_stick();
		setpoint.Z_throttle = -user_input.get_thr_stick();
		break;

	case MANUAL_S:
		setpoint.en_6dof = 0;
		setpoint.en_rpy_ctrl = 1;
		setpoint.en_Z_ctrl = 0;
		setpoint.en_XY_vel_ctrl = 0;
		setpoint.en_XY_pos_ctrl = 0;

		setpoint.roll = user_input.get_roll_stick();
		setpoint.pitch = user_input.get_pitch_stick();
		setpoint.Z_throttle = -user_input.get_thr_stick() /\
			(cos(state_estimate.roll) * cos(state_estimate.pitch));
		setpoint.update_yaw();
		break;

	case DIRECT_THROTTLE_6DOF:
		setpoint.en_6dof = 1;
		setpoint.en_rpy_ctrl = 1;
		setpoint.en_Z_ctrl = 0;
		setpoint.en_XY_vel_ctrl = 0;
		setpoint.en_XY_pos_ctrl = 0;

		setpoint.X_throttle = -user_input.get_pitch_stick();
		setpoint.Y_throttle = user_input.get_roll_stick();
		setpoint.Z_throttle = -user_input.get_thr_stick();
		setpoint.update_yaw();
		break;

	case AUTONOMOUS:
		setpoint.en_6dof = 0;
		setpoint.en_rpy_ctrl = 1;
		setpoint.en_Z_ctrl = 1;
		setpoint.en_XY_vel_ctrl = 0;
		setpoint.en_XY_pos_ctrl = 1;

		//Test functions:
		setpoint.en_AUTO_LIFTOFF_HOWER_TEST = 0;
		setpoint.en_XY_CIRC_TEST = 0;
		setpoint.en_XY_SQUARE_TEST = 0;

		if (setpoint.en_AUTO_LIFTOFF_HOWER_TEST) AUTO_LIFTOFF_HOWER_TEST();	//run automated position guide (only liftoff, hover and landing)
		if (setpoint.en_XY_CIRC_TEST) AUTO_XY_CIRC_TEST();  // run automated position guide (circular)
		if (setpoint.en_XY_SQUARE_TEST) AUTO_XY_SQUARE_TEST();  // run automated position guide (square)

		setpoint.update_setpoint_from_waypoint();
		//printf("\nsp.x = %f and st.x = %f, sp.y = %f and st.y = %f, sp.z = %f, st.z = %f sp.yaw=%f and st.yaw = %f\n",
		//    setpoint.X, state_estimate.X, setpoint.Y, state_estimate.Y, setpoint.Z,
		//    state_estimate.Z, setpoint.yaw, state_estimate.yaw);

		break;
	case ALT_HOLD_SS:
		setpoint.en_6dof = 0;
		setpoint.en_rpy_ctrl = 1;
		setpoint.en_Z_ctrl = 1;
		setpoint.en_XY_vel_ctrl = 0;
		setpoint.en_XY_pos_ctrl = 0;

		setpoint.roll = user_input.get_roll_stick();
		setpoint.pitch = user_input.get_pitch_stick();
		setpoint.update_Z();
		setpoint.update_yaw();
		break;

	

	case POSITION_CONTROL_SSS:
		setpoint.en_6dof = 0;
		setpoint.en_rpy_ctrl = 1;
		setpoint.en_Z_ctrl = 1;
		setpoint.en_XY_vel_ctrl = 0;
		setpoint.en_XY_pos_ctrl = 1;

		setpoint.X_dot = -user_input.get_pitch_stick() * settings.max_XY_velocity;
		setpoint.Y_dot = user_input.get_roll_stick() * settings.max_XY_velocity;

		//check validity of the velocity command, construct virtual setpoint
		setpoint.update_XY_pos();
		setpoint.update_Z();
		setpoint.update_yaw();

		//printf("\nM2  setpoint.X=%f setpoint.Y=%f setpoint.Z=%f with altitude of %f \n",setpoint.X,setpoint.Y,setpoint.Z,state_estimate.Z);
		break;

	case EMERGENCY_LAND:
		// 1) Enable PID Loops based on flight mode
		setpoint.en_6dof = 0;
		setpoint.en_rpy_rate_ctrl = 1;
		setpoint.en_rpy_ctrl = 1;
		setpoint.en_Z_ctrl = 0;
		setpoint.en_XY_pos_ctrl = 0;

		// 2) Assign Setpoints
		setpoint.roll = 0;
		setpoint.pitch = 0;

		setpoint.en_Z_land	= 1;  // start landing algorithm
		setpoint.V_max_land = settings.V_max_land;  // landing speed
		AUTO_LAND();

		setpoint.update_yaw();
		break;
	default: // should never get here
		fprintf(stderr, "ERROR in setpoint_manager thread, unknown flight mode\n");
		break;

	} // end switch(user_input.flight_mode)
	return 0;
}




/* Externally used functions to get information about the current feedback state */
bool feedback_state_t::is_initialized(void)
{
	return initialized;
}

double feedback_state_t::get_m(int i)
{
	return m[i];
}
double feedback_state_t::get_u(int i)
{
	return u[i];
}

arm_state_t feedback_state_t::get_arm_state(void)
{
	return arm_state;
}

uint64_t feedback_state_t::get_loop_index(void)
{
	return loop_index;
}
uint64_t feedback_state_t::get_last_step_ns(void)
{
	return last_step_ns;
}


/**
* @brief      Cleanup the feedback controller, freeing memory
*
* @return     0 on success, -1 on failure
*/
int feedback_state_t::cleanup(void)
{
	send_motor_stop_pulse();

	initialized = 0;
	return 0;
}