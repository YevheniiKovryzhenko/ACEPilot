/**
 * <log_manager.h>
 *
 * @brief      Functions to start, stop, and interact with the log manager
 *             thread.
 *
 */

#ifndef LOG_MANAGER_H
#define LOG_MANAGER_H


#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <string.h>
 // to allow printf macros for multi-architecture portability
#define __STDC_FORMAT_MACROS 
#include <inttypes.h>

#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>
#include <rc/encoder.h>

#include "rc_pilot_defs.h"
#include "thread_defs.h"
#include "settings.h"
#include "setpoint_manager.h"
#include "feedback.h"
#include "state_estimator.h"
#include "signal.h"
#include "xbee_receive.h"
#include "tools.h"


#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * Struct containing all possible values that could be writen to the log. For
	 * each log entry you wish to create, fill in an instance of this and pass to
	 * add_log_entry(void). You do not need to populate all parts of the struct.
	 * Currently feedback.c populates all values and log_manager.c only writes the
	 * values enabled in the settings file.
	 */
	typedef struct log_entry_t {
		/** @name index, always printed */
		///@{
		uint64_t loop_index; // timing
		// B: counter for the autonomous mode
		int64_t counter;
		uint64_t last_step_ns;
		// B rev counter of the four motors  
		int64_t rev1;
		int64_t rev2;
		int64_t rev3;
		int64_t rev4;

		///@}

		/** @name sensors */
		///@{
		double	v_batt;
		double	alt;
		double	gyro_roll;
		double	gyro_pitch;
		double	gyro_yaw;
		double	accel_X;
		double	accel_Y;
		double	accel_Z;
		///@}

		/** @name state estimate */
		///@{
		double	roll;
		double	pitch;
		double	yaw;
		double  yaw_cont;
		double	X;
		double	Y;
		double	Z;
		double	Xdot;
		double	Ydot;
		double	Zdot;
		double	xp;
		double	yp;
		double	zp;

		///@}xbee

		double xb;
		double yb;
		double zb;

		///@}

		/** @name setpoint */
		///@{
		double	sp_roll;
		double	sp_pitch;
		double	sp_yaw;
		double	sp_X;
		double	sp_Y;
		double	sp_Z;
		double	sp_Xdot;
		double	sp_Ydot;
		double	sp_Zdot;
		///@}

		/** @name orthogonal control outputs */
		///@{
		double	u_roll;
		double	u_pitch;
		double	u_yaw;
		double	u_X;
		double	u_Y;
		double	u_Z;
		///@}

		/** @name motor signals */
		///@{
		double	mot_1;
		double	mot_2;
		double	mot_3;
		double	mot_4;
		double	mot_5;
		double	mot_6;
		double	mot_7;
		double	mot_8;
		///@}

	} log_entry_t;



	/**
	 * @brief      creates a new csv log file and starts the background thread.
	 *
	 * @return     0 on success, -1 on failure
	 */
	int log_manager_init(void);


	/**
	 * @brief      quickly add new data to local buffer
	 *
	 * This is called after feedback_march after signals have been sent to
	 * the motors.
	 *
	 * @return     0 on success, -1 on failure
	 */
	int log_manager_add_new();

	/**
	 * @brief      Finish writing remaining data to log and close thread.
	 *
	 *             Used in log_manager.c
	 *
	 * @return     0 on sucess and clean exit, -1 on exit timeout/force close.
	 */
	int log_manager_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif // LOG_MANAGER_H
