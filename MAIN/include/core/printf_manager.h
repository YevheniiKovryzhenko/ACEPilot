/**
 * <printf_manager.h>
 *
 * @brief      Functions to start and stop the printf mnaager which is a
 *             separate thread printing data to the console for debugging.
 */


#ifndef PRINTF_MANAGER_H
#define PRINTF_MANAGER_H
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>
#include <rc/encoder.h>
#include <signal.h>

#include "rc_pilot_defs.h"
#include "flight_mode.h"
#include "input_manager.h"
#include "setpoint_manager.h"
#include "feedback.h"
#include "state_estimator.h"
#include "thread_defs.h"
#include "settings.h"
#include "xbee_receive.h"


#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * @brief      Start the printf_manager thread which should be the only thing
	 *             printing to the screen besides error messages from other threads.
	 *
	 * @return     0 on success, -1 on failure
	 */
	int printf_init(void);


	/**
	 * @brief      Waits for the printf manager thread to exit.
	 *
	 * @return     0 on clean exit, -1 on exit time out/force close
	 */
	int printf_cleanup(void);


	/**
	 * @brief      Only used by printf_manager right now, but could be useful
	 * elsewhere.
	 *
	 * @param[in]  mode  The mode
	 *
	 * @return     0 on success or -1 on error
	 */
	int print_flight_mode(flight_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif //PRINTF_MANAGER_H
