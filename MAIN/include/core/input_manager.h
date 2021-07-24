/**
 * <input_manager.h>
 *
 * Functions to start and stop the input manager thread which is the translation
 * beween control inputs from DSM to the user_input struct which is read by the
 * setpoint manager. TODO: Allow other inputs such as mavlink
 */

#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H


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

#include "setpoint_manager.h"
#include "settings.h"
#include "rc_pilot_defs.h"
#include "flight_mode.h"
#include "state_estimator.h"
#include "rc_pilot_defs.h"
#include "thread_defs.h"
#include "feedback.h"

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * Represents current command by the user. This is populated by the
	 * input_manager thread which decides to read from mavlink or DSM depending on
	 * what it is receiving.
	 */
	typedef struct user_input_t {
		int initialized;				///< set to 1 after input_manager_init(void)
		flight_mode_t flight_mode;		///< this is the user commanded flight_mode.
		int input_active;				///< nonzero indicates some user control is coming in
		arm_state_t requested_arm_mode;	///< set to ARMED after arming sequence is entered.
		arm_state_t arm_switch; 		///< actual position of the physical switch

		// All sticks scaled from -1 to 1
		double thr_stick;		///< positive forward
		double yaw_stick;		///< positive to the right, CW yaw
		double roll_stick;		///< positive to the right
		double pitch_stick;		///< positive forward
	} user_input_t;

	extern user_input_t user_input;

	/**
	 * @brief      Starts an input manager thread.
	 *
	 *             Watch for new DSM data and translate into local user mode. Used
	 *             in input_manager.c
	 *
	 * @return     0 on success, -1 on failure
	 */
	int input_manager_init(void);

	/**
	 * @brief      Waits for the input manager thread to exit
	 *
	 *             This should only be called after the program flow state is set to
	 *             EXITING as that's the only thing that will cause the thread to
	 *             exit on its own safely. Used in input_manager.c
	 *
	 * @return     0 on clean exit, -1 if exit timed out.
	 */
	int input_manager_cleanup(void);

#ifdef __cplusplus
}
#endif
#endif // INPUT_MANAGER_H
