/*
 * log_manager.hpp
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
 * Last Edit:  09/03/2022 (MM/DD/YYYY)
 * 
 * Class to start, stop, and interact with the log manager.
 */



#ifndef LOG_MANAGER_H
#define LOG_MANAGER_H
#include <stdbool.h>
#include "thread_gen.hpp"
#include "state_estimator.hpp"
#include "barometer_gen.hpp"
#include "IMU_9DOF_gen.hpp"
#include "mocap_gen.hpp"
#include "KF.hpp"
#include "EKF.hpp"
#include "EKF2.hpp"

#define MAX_LOG_FILES	500

/**
* Struct containing all possible values that could be writen to the log. For
* each log entry you wish to create, fill in an instance of this and pass to
* add_log_entry(void). You do not need to populate all parts of the struct.
* Currently feedback.c populates all values and log_manager.c only writes the
* values enabled in the settings file.
*/
class log_entry_t {
private:
	bool initialized;
	bool file_open;
	uint64_t num_entries_skipped;
	uint64_t num_entries;	// number of entries logged so far
	FILE* log_fd;          ///< file descriptor for the log file

	bool logging_enabled;
	bool new_data_available;
	bool request_reset_fl;
	thread_gen_t thread;

	/** @name index, always printed */
	///@{
	uint64_t loop_index; // timing
	uint64_t last_step_ns;
	//uint64_t imu_time_ns;
	uint64_t log_time_ns;

	///@}

	/* State estimator */
	state_estimator_log_entry_t state_estimator_entry{};

	/** @name sensors */
	///@{
	voltage_sensor_log_entry_t battery_entry{};
	barometer_log_entry_t bmp_entry{};
	IMU_9DOF_log_entry_t imu_entry{};
	///@}

	/*** @name mocap data */
	///@{
	mocap_log_entry_t mocap_entry{};
	///@}

	/*** @name gps data */
	///@{
	double gps_lon;        // longitude in degree decimal
	double gps_lat;        // latitude in degree decimal
	double gps_gpsAlt;     // altitude in m (from GPS)
	double gps_ned_x;      // ned x coordinates
	double gps_ned_y;      // ned y coordinates
	double gps_ned_z;      // ned z coordinates
	double gps_spd;        // speed in m/s
	int gps_fix;           // fix type
	uint8_t gps_sat;       // number of satellites
	double gps_headingNc;  // heading (not tilt compensated) in degrees
	double gps_cog;        // course over ground
	double gps_gpsVsi;     // vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
	double gps_hdop;       // horizontal dilution of precision
	double gps_vdop;       // vertical dilution of precision
	uint8_t gps_year;
	uint8_t gps_month;
	uint8_t gps_day;
	uint8_t gps_hour;
	uint8_t gps_minute;
	uint8_t gps_second;
	uint64_t gps_time_received_ns;
	///@}

	/* Filters */
	KF_log_entry_t KF_altitude_entry{};
	EKF_log_entry_t EKF1_entry{};
	EKF2_log_entry_t EKF2_entry{};

	/** @name throttles */
	///@{
	double X_throttle;
	double Y_throttle;
	double Z_throttle;
	double X_throttle_ff;
	double Y_throttle_ff;
	double Z_throttle_ff;
	double roll_throttle;
	double pitch_throttle;
	double yaw_throttle;
	double roll_throttle_ff;
	double pitch_throttle_ff;
	double yaw_throttle_ff;
	///@}

	/** @name attitude setpoints */
	///@{
	double sp_roll_dot;
	double sp_pitch_dot;
	double sp_yaw_dot;
	double sp_roll_dot_ff;
	double sp_pitch_dot_ff;
	double sp_yaw_dot_ff;
	double sp_roll;
	double sp_pitch;
	double sp_yaw;
	double sp_roll_ff;
	double sp_pitch_ff;
	double sp_yaw_ff;
	///@}

	/** @name XYZ setpoints */
	///@{
	double sp_X;
	double sp_Y;
	double sp_Z;
	double sp_X_ff;
	double sp_Y_ff;
	double sp_Z_ff;
	double sp_Xdot;
	double sp_Ydot;
	double sp_Zdot;
	double sp_Xdot_ff;
	double sp_Ydot_ff;
	double sp_Zdot_ff;
	double sp_Xddot;
	double sp_Yddot;
	double sp_Zddot;
	double sp_Xddot_ff;
	double sp_Yddot_ff;
	double sp_Zddot_ff;
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

	/** @name dsm connection valid */
	///@{
	int dsm_con;
	///@}

	/** @name flight mode */
	///@{
	int flight_mode;
	///@}

	/** @name imu_isr() Benchmarking Timers */
	///@{
	uint64_t tIMU_END, tSM, tCOMMS, tMOCAP, tGPS, tPNI, tNAV, tGUI, tCTR, tLOG, tNTP;
	///@}

	/**
	 * @brief      writes header for into a log file
	 *
	 *
	 * @return     0 on success, -1 on failure
	 */
	int write_header(void);

	int write_log_entry(void);

	void construct_new_entry(void);

	/**
	 * @brief      resets log mannager, creates a new csv log file and starts the background thread.
	 *
	 * @return     0 on success, -1 on failure
	 */
	int reset(void);


	/**
	 * @brief      quickly add new data to local buffer
	 *
	 * This is called after feedback_march after signals have been sent to
	 * the motors.
	 *
	 * @return     0 on success, -1 on failure
	 */
	int add_new(void);

public:	
	/**
	 * @brief      notifies the thread that new data is available
	 *
	 *
	 * @return     0 on success, -1 on failure
	 */
	void data_available(void);

	/**
	 * @brief      notifies the thread that reset is requested
	 *
	 *
	 * @return     0 on success, -1 on failure
	 */
	int request_reset(void);

	/**
	 * @brief      main update loop of the thread
	 *
	 *
	 * @return     0 on success, -1 on failure
	 */
	int update(void);

	/**
	 * @brief      creates a new csv log file and starts the background thread. only call once.
	 *
	 * @return     0 on success, -1 on failure
	 */
	int init(void);
	

	/**
	 * @brief      Finish writing remaining data to log and close thread.
	 *
	 *             Used in log_manager.c
	 *
	 * @return     0 on sucess and clean exit, -1 on exit timeout/force close.
	 */
	int cleanup(void);
};

// Single place in memory to hold log_entries (no function passing)
extern log_entry_t log_entry;

#endif // LOG_MANAGER_H
