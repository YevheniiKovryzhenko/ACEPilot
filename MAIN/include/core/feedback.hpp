/*
 * feedback.hpp
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

#ifndef FEEDBACK_H
#define FEEDBACK_H
#include <stdbool.h>

#include "controller.hpp"

/**
	* This is the state of the feedback loop. contains most recent values
	* reported by the feedback controller. Should only be written to by the
	* feedback controller after initialization.
	*/
class feedback_state_t
{
private:
	bool initialized;			///< set to 1 after feedback_init(void)
	double u[MAX_INPUTS];		///< siso controller outputs
	double m[MAX_ROTORS];		///< signals sent to motors after mapping

	arm_state_t arm_state;		///< actual arm state as reported by feedback controller
	uint64_t arm_time_ns;		///< time since boot when controller was armed
	uint64_t loop_index;		///< increases every time feedback loop runs
	uint64_t last_step_ns;		///< last time controller has finished a step

	feedback_controller_t controller;

	int send_motor_stop_pulse(void);

	int update_gains(void);

	//int update_setpoints(void);

	int zero_out_ff(void);

public:

	/* Externally used functions to get information about the current feedback state */
	double get_m(int i);
	double get_u(int i);

	arm_state_t get_arm_state(void);
	uint64_t get_loop_index(void);
	uint64_t get_last_step_ns(void);

	/**
	* @brief      Initial setup of all feedback controllers. Should only be called
	*             once on program start.
	*
	* @param      setpoint  pointer to global setpoint struct
	* @param      settings  pointer to global settings struct
	*
	* @return     0 on success, -1 on failure
	*/
	int init(void);
	bool is_initialized(void);


	/**
	* @brief      marches feedback controller forward one step
	*
	* This is called AFTER state_estimator_march and actually sends signals to the
	* motors. This can as is still safely called when Disarmed.
	*
	* @return     0 on success, -1 on failure
	*/
	int march(void);

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
	int disarm(void);

	/**
	* @brief      This is how outside functions should start the flight controller.
	*
	* @return     0 on success, -1 on failure
	*/
	int arm(void);


	/**
	* @brief      Cleanup the feedback controller, freeing memory
	*
	* @return     0 on success, -1 on failure
	*/
	int cleanup(void);
};

extern feedback_state_t fstate;


#endif // FEEDBACK_H
