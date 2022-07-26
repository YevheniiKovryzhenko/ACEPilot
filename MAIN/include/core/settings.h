/**
 * <settings.h>
 *
 * @brief      Functions to read the json settings file
 */

#ifndef SETTINGS_H
#define SETTINGS_H
#include <stdbool.h>
#include <stdio.h>
#include <string.h>	// FOR str_cmp()
#include <fcntl.h>	// for F_OK
#include <unistd.h>	// for access()
 //#include <json-c/json.h> //if unavailable run: sudo apt install libjson-c-dev libjson-c3
#include <json.h>

#include <rc/math/filter.h>
#include <rc/mpu.h>

#include "mix.h"
#include "flight_mode.h"
#include "rc_pilot_defs.h"
#include "thrust_map.h"
//#include "mix_servos.hpp"
#include "servo_mix_defs.h"

#ifdef __cplusplus
extern "C"
{
#endif



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
	int dof;
	thrust_map_t thrust_map;
	double v_nominal;
	bool enable_v_gain_scaling;
	bool enable_magnetometer; // we suggest leaving as 0 (mag OFF)
	bool enable_mocap;	//enable mocap serial link
	bool use_mocap_yaw;
	bool use_mocap_pitch;
	bool use_mocap_roll;
	bool use_mocap_yaw_rate;
	bool use_mocap_pitch_rate;
	bool use_mocap_roll_rate;
	bool enable_encoders;
	bool enable_gps;
	bool enable_ext_mag;
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
	bool printf_int_mag;
	bool printf_ext_mag;
	bool printf_rev;
	bool printf_counter;
	///@}

	/** @name log settings */
	///@{
	bool enable_logging;
	int log_every_n_entry;
	bool log_only_while_armed;
	bool log_sensors;
	bool log_state;
	bool log_mocap;
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
	bool log_ext_mag;
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
	rc_filter_t roll_rate_controller_pd;
	rc_filter_t roll_rate_controller_i;
	double roll_rate_controller_FF;
	double roll_rate_controller_K;
	rc_filter_t pitch_rate_controller_pd;
	rc_filter_t pitch_rate_controller_i;
	double pitch_rate_controller_FF;
	double pitch_rate_controller_K;
	rc_filter_t yaw_rate_controller_pd;
	rc_filter_t yaw_rate_controller_i;
	double yaw_rate_controller_FF;
	double yaw_rate_controller_K;

	rc_filter_t roll_controller_pd;
	rc_filter_t pitch_controller_pd;
	rc_filter_t yaw_controller_pd;
	rc_filter_t roll_controller_i;
	rc_filter_t pitch_controller_i;
	rc_filter_t yaw_controller_i;
	double roll_controller_FF;
	double pitch_controller_FF;
	double yaw_controller_FF;
	double roll_controller_K;
	double pitch_controller_K;
	double yaw_controller_K;

	rc_filter_t altitude_rate_controller_pd;
	rc_filter_t altitude_rate_controller_i;
	double altitude_rate_controller_FF;
	double altitude_rate_controller_K;
	rc_filter_t altitude_controller_pd;
	rc_filter_t altitude_controller_i;
	double altitude_controller_FF;
	double altitude_controller_K;

	rc_filter_t horiz_vel_ctrl_pd_X;
	rc_filter_t horiz_vel_ctrl_i_X;
	double horiz_vel_X_controller_FF;
	double horiz_vel_X_controller_K;
	rc_filter_t horiz_vel_ctrl_pd_Y;
	rc_filter_t horiz_vel_ctrl_i_Y;
	double horiz_vel_Y_controller_FF;
	double horiz_vel_Y_controller_K;

	rc_filter_t horiz_pos_ctrl_X_pd;
	rc_filter_t horiz_pos_ctrl_Y_pd;
	rc_filter_t horiz_pos_ctrl_X_i;
	rc_filter_t horiz_pos_ctrl_Y_i;
	double horiz_pos_X_ctrl_FF;
	double horiz_pos_Y_ctrl_FF;
	double horiz_pos_X_ctrl_K;
	double horiz_pos_Y_ctrl_K;

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

#ifdef __cplusplus
}
#endif

#endif // SETTINGS_H
