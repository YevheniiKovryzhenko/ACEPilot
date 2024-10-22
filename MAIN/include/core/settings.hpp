/*
 * settings.hpp
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
 * Functions to read the json settings file
 */

#ifndef SETTINGS_HPP
#define SETTINGS_HPP
#include <stdbool.h>
#include <stdio.h>
#include <string.h>	// FOR str_cmp()
#include <fcntl.h>	// for F_OK
#include <unistd.h>	// for access()
 //#include <json-c/json.h> //if unavailable run: sudo apt install libjson-c-dev libjson-c3
#include <json.h>

#include <rc/math/filter.h>
#include <rc/mpu.h>

#include "mix.hpp"
#include "flight_mode.hpp"
#include "rc_pilot_defs.hpp"
#include "thrust_map.hpp"

#include "mocap_gen.hpp"
#include "IMU_9DOF_gen.hpp"
#include "voltage_sensor_gen.hpp"
#include "mix_servos.hpp"

/** @name feedback control struct */
///@{
typedef struct controller_settings_t {
	rc_filter_t pd;
	rc_filter_t i;
	double FF;
	double K;
}controller_settings_t;


/**
 * Configuration settings read from the json settings file and passed to most
 * threads as they initialize.
 */
typedef struct settings_t{
	/** @name File details */
	char name[128]; ///< string declaring the name of the settings file
	///@}

	/**@name warings */
	///@{
	bool warnings_en;
	bool delay_warnings_en;
	bool telem_warnings_en;
	///@}

	/** @name physical parameters */
	///@{
	int num_rotors;
	double arm_time_s;
	rotor_layout_t layout;
	servo_layout_t servo_layout;
	thrust_map_t thrust_map;
	voltage_sensor_settings_t battery;
	IMU_9DOF_gen_settings_t IMU0;
	IMU_9DOF_gen_settings_t IMU1;
	mocap_settings_t mocap;
	bool enable_encoders;
	bool enable_gps;
	///@}

	/** @name physical parameters */
	///@{
	bool enable_servos;
	bool en_servo_ch_pool_sat;
	int servo_i2c_driver_id;
	int num_servos;
	double servos_arm_time_s;
	///@}

	//True if you want to automatically enter OPEN_LOOP_DESCENT if using a mode that
	//requires MOCAP and mocap has been unavailable for 'mocap_dropout_timeout_ms' ms
	//OPEN_LOOP_DESCENT commands 0 roll, 0 pitch, and throttle of 'dropout_z_throttle'
	//One can exit this mode by switching to a controller mode that doesn't require MOCAP
	bool enable_mocap_dropout_emergency_land;
	double mocap_dropout_timeout_ms;

	/** @name flight modes */
	///@{
	int num_dsm_modes;
	flight_mode_t flight_mode_1;
	flight_mode_t flight_mode_2;
	flight_mode_t flight_mode_3;
	///@}


	/** @name dsm radio config */
	///@{
	bool enable_dsm;
	int dsm_thr_ch;
	int dsm_thr_pol;
	int dsm_roll_ch;
	int dsm_roll_pol;
	int dsm_pitch_ch;
	int dsm_pitch_pol;
	int dsm_yaw_ch;
	int dsm_yaw_pol;
	int dsm_mode_ch;
	int dsm_mode_pol;
	dsm_kill_mode_t dsm_kill_mode;
	int dsm_kill_ch;
	int dsm_kill_pol;
	///@}

	/** @name printf settings */
	///@{
	bool printf_arm;
	bool printf_altitude;
	bool printf_battery;
	bool printf_rpy;
	bool printf_sticks;
	bool printf_setpoint;
	bool printf_setpoint_xy;
	bool printf_setpoint_xy_dot;
	bool printf_setpoint_z;
	bool printf_setpoint_z_dot;
	bool printf_setpoint_att;
	bool printf_setpoint_att_dot;
	bool printf_u;
	bool printf_motors;
	bool printf_mode;
	bool printf_mocap;
	bool printf_gain_tunning;
	bool printf_tracking;
	bool printf_gps;
	///@}

	/** @name log settings */
	///@{
	bool enable_logging;
	int log_every_n_entry;
	bool log_only_while_armed;
	bool log_sensors;
	bool log_state;
	bool log_gps;

	bool log_setpoints;
	bool log_throttles;
	bool log_throttles_ff;
	bool log_attitude_rate_setpoint;
	bool log_attitude_rate_setpoint_ff;
	bool log_attitude_setpoint;
	bool log_attitude_setpoint_ff;
	bool log_acceleration_setpoint;
	bool log_acceleration_setpoint_ff;
	bool log_velocity_setpoint;
	bool log_velocity_setpoint_ff;
	bool log_position_setpoint;
	bool log_position_setpoint_ff;
	
	bool log_dsm;
	bool log_flight_mode;
	bool log_benchmark;
	bool log_control_u;
	bool log_motor_signals;
	bool log_encoders;
	///@}

	/** @name mavlink stuff */
	///@{
	char dest_ip[24];
	uint8_t my_sys_id;
	uint16_t mav_port;

	/** @name feedback controllers */
	///@{
	controller_settings_t roll_rate_ctrl;
	controller_settings_t pitch_rate_ctrl;
	controller_settings_t yaw_rate_ctrl;
	controller_settings_t roll_ctrl;
	controller_settings_t pitch_ctrl;
	controller_settings_t yaw_ctrl;
	controller_settings_t X_vel_ctrl;
	controller_settings_t Y_vel_ctrl;
	controller_settings_t Z_vel_ctrl;
	controller_settings_t X_pos_ctrl;
	controller_settings_t Y_pos_ctrl;
	controller_settings_t Z_pos_ctrl;

	///@}

	/** @name dsm connection */
	///@{
	int dsm_timeout_ms;
	///@}

	/** @name waypoint folder name and path*/
    ///@{
    char wp_folder[100];
    ///@}

    /** @name waypoint filenames for takeoff, guided, and landing */
    ///@{
    char wp_takeoff_filename[100];
    char wp_guided_filename[100];
    char wp_landing_filename[100];
    ///@}

    /** @name serial ports the Xbee and other hardware it is connected to */
    ///@{
    char serial_port_1[50];
    int serial_baud_1;
    char serial_port_2[50];
    int serial_baud_2;
	char serial_port_gps[50];
	int serial_baud_gps;
    ///@}

	/** @name parameters for autonomous flight modes*/
	///@{
	double hover_throttle;
	double V_max_land;
	double height_takeoff;
	double t_takeoff;
	double XY_start_delay_s;
	double XY_waypt_delay_s;
	double max_XY_velocity;
	double max_Z_velocity;
	double square_X_offset;
	double square_Y_offset;
	double square_X_time_s;
	double square_Y_time_s;
	double turn_radius;
	double turn_period;
	double turn_time_s;
	bool turn_dir_yaw_cw;
	///@}

	//** @name remote tuning parameters */
	///@{
	bool allow_remote_tuning;
	///@}
}settings_t;

/**
 * settings are external, so just include this header and read from it
 */
extern settings_t settings;

/**
 * @brief      Populates the settings and controller structs with the settings file.
 *
 * @return     0 on success, -1 on failure
 */
int settings_load_from_file(char* path);


/**
 * @brief      Only used in debug mode. Prints settings to console
 *
 * @return     0 on success, -1 on failure
 */
int settings_print(void);

#endif // SETTINGS_H
