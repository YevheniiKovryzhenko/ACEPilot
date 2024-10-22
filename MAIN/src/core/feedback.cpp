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
 * Last Edit:  09/17/2022 (MM/DD/YYYY)
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

#include "state_estimator.hpp"
#include "rc_pilot_defs.hpp"
#include "setpoint_manager.hpp"
#include "settings.hpp"
#include "mix.hpp"
#include "thrust_map.hpp"
#include "tools.h"
#include "controller.hpp"
#include "log_manager.hpp"
#include "input_manager.hpp"
#include "servos.hpp"
#include "thread_defs.hpp"

#include "feedback.hpp"

 // preposessor macros
#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif // !unlikely

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif // !likely

 /*
 * Invoke defaut constructor for all the built in and exernal types in the class
 */
feedback_state_t fstate{}; // extern variable in feedback.hpp


static void* __arm_thread_func(__attribute__((unused)) void* ptr)
{
	while (rc_get_state() != EXITING)
	{
		char tmp = fstate.update_arm_thread();
		if (tmp < 0)
		{
			printf("ERROR in __arm_thread_func: failed to update main thread\n");
			return NULL;
		}
		else if (tmp > 0) return NULL; //completed execution
	}
	return NULL;
}

char feedback_state_t::send_motor_stop_pulse(void)
{
	int i;
	if (unlikely(settings.num_rotors > 8))
	{
		printf("ERROR in send_motor_stop_pulse: too many rotors\n");
		return -1;
	}
	for (i = 0; i < settings.num_rotors; i++) {
		m[i] = -0.1;
		if (unlikely(rc_servo_send_esc_pulse_normalized(i + 1, -0.1) < 0))
		{
			fprintf(stderr, "ERROR in send_motor_stop_pulse: failed to send stop pulse to motor %i\n", i + 1);
			return -1;
		}
	}
	return 0;
}

char feedback_state_t::update_arm_thread(void)
{
	if (finddt_s(arm_time_ns) < settings.arm_time_s)
	{
		started_arming_fl = true;
		if (unlikely(send_motor_stop_pulse() < 0))
		{
			fprintf(stderr, "ERROR in update_arm_thread: failed to send motor stop pulse\n");
			return -1;
		}
	}
	else
	{
		started_arming_fl = false;
		if (settings.warnings_en) printf("WARNING: ARMED!\n");

		// set LEDs
		rc_led_set(RC_LED_RED, 0);
		rc_led_set(RC_LED_GREEN, 1);
		// last thing is to flag as armed
		arm_state = ARMED;

		return 1;
	}

	double tmp = finddt_s(arm_thread_time);
	arm_thread_time = rc_nanos_since_boot();
	if (tmp < 1.0 / ARM_FEEDBACK_HZ)
	{
		rc_usleep(1000000 * (1.0 / ARM_FEEDBACK_HZ - tmp));
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
	reset_arming_fl();
	// set LEDs
	rc_led_set(RC_LED_RED, 1);
	rc_led_set(RC_LED_GREEN, 0);

	if (settings.enable_servos && sstate.is_armed())
	{
		if (unlikely(sstate.disarm() == -1))
		{
			printf("ERROR in disarm: failed to disarm servos\n");
			return -1;
		}
	}

	if (settings.enable_logging)
	{
		if (settings.log_only_while_armed)
		{
			log_entry.request_shutdown();
		}
		else
		{
			log_entry.request_reset();
		}		
	}
	return 0;
}
/* externally resets arming flags */
int feedback_state_t::reset_arming_fl(void)
{
	started_arming_fl = false;
	return 0;
}//does not disarm 

/**
* @brief      This is how outside functions should start the flight controller.
*
* @return     0 on success, -1 on failure
*/
int feedback_state_t::arm(void)
{
	if (unlikely(!initialized))
	{
		if (settings.warnings_en) printf("ERROR in arm: feedback state not initialized\n");
		return -1;
	}	
	//printf("\n Arming!\n");
	if (unlikely(arm_state == ARMED)) {
		if (settings.warnings_en) printf("WARNING: trying to arm when controller is already armed\n");
		return 0;
	}

	if (!started_arming_fl)
	{
		// start a new log file every time controller is armed, this may take some
		// time so do it before touching anything else
		if (settings.enable_logging)
		{
			if (settings.log_only_while_armed)
			{
				if (unlikely(log_entry.start() == -1))
				{
					printf("ERROR in arm: failed to start log manager thread\n");
					return -1;
				}
			}
			else
			{
				if (unlikely(log_entry.request_reset() == -1))
				{
					printf("ERROR in arm: failed to request reset for log manager\n");
					return -1;
				}
			}			
		}


		if (settings.enable_servos)
		{
			if (unlikely(sstate.arm() == -1))
			{
				printf("ERROR in arm: failed to arm servos\n");
				return -1;
			}
		}

		// get the current time
		arm_time_ns = rc_nanos_since_boot();
		// reset the index
		loop_index = 0;

		if (settings.warnings_en) printf("WARNING: Waking up the ESCs....\n");
		/* Starting arming Thread */
		if (unlikely(thread.start(__arm_thread_func) < 0))
		{
			fprintf(stderr, "ERROR in init: failed to start arming thread\n");
			return -1;
		}
	}
	

	setpoint.reset_all(); //reset setpoints
	controller.reset(); //reset control system

	/*
	if (finddt_s(arm_time_ns) < settings.arm_time_s)
	{
		started_arming_fl = true;
		send_motor_stop_pulse();
		return 0;
	}
	started_arming_fl = false;
	if (settings.warnings_en) printf("WARNING: ARMED!\n");

	// set LEDs
	rc_led_set(RC_LED_RED,0);
	rc_led_set(RC_LED_GREEN,1);
	// last thing is to flag as armed
	arm_state = ARMED;
	*/
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
		printf("WARNING in init: feedback state already initialized\n");
		return 0;
	}

	arm_time_ns = rc_nanos_since_boot();

	if (unlikely(controller.init() == -1))
	{
		printf("ERROR in init: failed to initialize the controllers\n");
		return -1;
	}
	
	// make sure everything is disarmed then start the ISR
	if (unlikely(disarm() == -1))
	{
		printf("ERROR in init: failed to disarm\n");
		return -1;
	}

	if (unlikely(thread.init(ARM_FEEDBACK_PRI, FIFO) < 0))
	{
		fprintf(stderr, "ERROR in init: failed to start arm thread\n");
		return -1;
	}

	if (settings.enable_servos)
	{
		if (!sstate.is_initialized())
		{
			if (unlikely(sstate.init(settings.servo_i2c_driver_id) == -1))
			{
				printf("ERROR in init: failed to init servos\n");
				return -1;
			}
		}
		if (sstate.is_armed())
		{
			// make sure everything is disarmed then start the ISR
			if (unlikely(sstate.disarm() == -1))
			{
				printf("ERROR in init: failed to disarm servos\n");
				return -1;
			}
		}
	}
	
	
	reset_arming_fl();
	initialized = true;
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
		printf("ERROR in march: feedback state not initialized\n");
		disarm();
		return -1;
	}


	int i;
	double tmp_u[MAX_INPUTS], tmp_mot[MAX_ROTORS], tmp_s[MAX_SERVO_INPUTS], tmp_servos[MAX_SERVOS];

	// Disarm if rc_state is somehow paused without disarming the controller.
	// This shouldn't happen if other threads are working properly.
	if (unlikely(rc_get_state() != RUNNING && arm_state == ARMED))
	{
		disarm();
		printf("\n rc_state is somehow paused \n");
	}

	// check for a tipover
	if (unlikely(fabs(state_estimate.get_roll()) > TIP_ANGLE || fabs(state_estimate.get_pitch()) > TIP_ANGLE)) 
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

	// We are about to start marching the individual SISO controllers forward.
	// Start by zeroing out the motors signals then add from there.
	for (i = 0; i < MAX_ROTORS; i++) tmp_mot[i] = 0.0;
	for (i = 0; i < MAX_INPUTS; i++) tmp_u[i] = 0.0;

	if (settings.enable_servos)
	{
		for (i = 0; i < MAX_SERVOS; i++) tmp_servos[i] = 0.0;
		for (i = 0; i < MAX_SERVO_INPUTS; i++) tmp_s[i] = 0.0;

		//we need to march a dedicated servo controller here
		if (sstate.march_controller(tmp_u, tmp_servos) == -1)
		{
			printf("ERROR in march: failed to march the servo controller\n");
			disarm();
		}
	}
	

	if (controller.march(tmp_u, tmp_mot) == -1)
	{
		printf("ERROR in march: failed to march the controller\n");
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
		//rc_saturate_double(&m[i], 0.0, 1.0);

		// finally send pulses!
		rc_servo_send_esc_pulse_normalized(i + 1, m[i]);
		
	}
	if (settings.enable_servos)
	{
		for (int i = 0; i < settings.num_servos; i++)
		{
			//rc_saturate_double(&tmp_servos[i], 0.0, 1.0);
			//need to do control allocation here:
			tmp_s[i] = tmp_servos[i]; //assume direct 1-1 map
			//printf("\nServo CMND: %f", tmp_s[i]);
			// final saturation just to take care of possible rounding errors
			// this should not change the values and is probably excessive
			rc_saturate_double(&tmp_s[i], -1.0, 1.0);
			//send mapped into [0 1] control signals to servos:
			//sstate.march(i, tmp_s[i]);
			sstate.march_with_centering(i, tmp_s[i]);
		}
	}
	/***************************************************************************
	* Final cleanup, timing, and indexing
	***************************************************************************/
	// Load control inputs into cstate for viewing by outside threads
	for (i = 0; i < MAX_INPUTS; i++) u[i] = tmp_u[i];
	// keep track of loops since arming
		loop_index++;
	// log us since arming, mostly for the log
	last_step_ns = rc_nanos_since_boot();

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
	if (thread.stop(ARM_FEEDBACK_TOUT) < 0)
	{
		printf("WARNING in cleanup: failed to close arm thread\n");
	}
	send_motor_stop_pulse();

	initialized = 0;
	return 0;
}
