/*
 * settings.cpp
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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>	// FOR str_cmp()
#include <fcntl.h>	// for F_OK
#include <unistd.h>	// for access()
#include <json.h>

#include <rc/math/filter.h>
#include <rc/mpu.h>

#include "flight_mode.hpp"
#include "thrust_map.hpp"
#include "mix.hpp"
#include "rc_pilot_defs.hpp"
#include "feedback_filter_gen.hpp"
#include "settings_gen.hpp"
#include "settings.hpp"


// json object respresentation of the whole settings file
static json_object* jobj;

// primary settings struct declared as extern in header is defined ONCE here
settings_t settings;

// if anything goes wrong set this flag back to 0
static int was_load_successful = 0;

////////////////////////////////////////////////////////////////////////////////
/// MACROS FOR PARSING JSON TYPES
////////////////////////////////////////////////////////////////////////////////

// macro for reading a boolean
#define PARSE_BOOL(name) \
if(json_object_object_get_ex(jobj, #name, &tmp)==0){\
	fprintf(stderr,"ERROR parsing settings file, can't find " #name "\n");\
	return -1;\
}\
if(json_object_is_type(tmp, json_type_boolean)==0){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be a boolean\n");\
	return -1;\
}\
settings.name = json_object_get_boolean(tmp);\


// macro for reading an integer
#define PARSE_INT(name) \
if(json_object_object_get_ex(jobj, #name, &tmp)==0){ \
	fprintf(stderr,"ERROR parsing settings file, can't find " #name "\n");\
	return -1;\
}\
if(json_object_is_type(tmp, json_type_int)==0){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be an int\n");\
	return -1;\
}\
settings.name = json_object_get_int(tmp);\


// macro for reading a bound integer
#define PARSE_INT_MIN_MAX(name,min,max) \
if(json_object_object_get_ex(jobj, #name, &tmp)==0){ \
	fprintf(stderr,"ERROR parsing settings file, can't find " #name "\n");\
	return -1;\
}\
if(json_object_is_type(tmp, json_type_int)==0){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be an int\n");\
	return -1;\
}\
settings.name = json_object_get_int(tmp);\
if(settings.name<min || settings.name>max){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be between min and max\n");\
	return -1;\
}\


// macro for reading a polarity which should be +-1
#define PARSE_POLARITY(name) \
if(json_object_object_get_ex(jobj, #name, &tmp)==0){ \
	fprintf(stderr,"ERROR parsing settings file, can't find " #name "\n");\
	return -1;\
}\
if(json_object_is_type(tmp, json_type_int)==0){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be an int\n");\
	return -1;\
}\
settings.name = json_object_get_int(tmp);\
if(settings.name!=-1 && settings.name!=1){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be -1 or 1\n");\
	return -1;\
}\


// macro for reading a floating point number
#define PARSE_DOUBLE_MIN_MAX(name,min,max)\
if(parse_double(jobj, #name, settings.name) < 0){\
	fprintf(stderr,"ERROR: could not parse " #name "\n");\
	return -1;\
}\
if(settings.name<min || settings.name>max){\
	fprintf(stderr,"ERROR " #name " should be between min and max\n");\
	return -1;\
}\


// macro for reading a string
#define PARSE_STRING(name) \
if(json_object_object_get_ex(jobj, #name, &tmp)==0){ \
	fprintf(stderr,"ERROR parsing settings file, can't find " #name "\n");\
	return -1;\
}\
if(json_object_is_type(tmp, json_type_string)==0){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be a string\n");\
	return -1;\
}\
strcpy(settings.name, json_object_get_string(tmp));

// macro for reading feedback controller
#define PARSE_CONTROLLER(name) \
if(parse_feedback_filter_gen(jobj, #name, settings.name) < 0){\
	fprintf(stderr,"ERROR: could not parse " #name "\n");\
	return -1;\
}\

#define PARSE_BATTERY(name)\
if(parse_voltage_sensor_gen_settings(jobj, #name, settings.name) < 0){\
	fprintf(stderr,"ERROR: could not parse " #name "\n");\
	return -1;\
}\

#define PARSE_MOCAP(name)\
if(parse_mocap_gen_settings(jobj, #name, settings.name) < 0){\
	fprintf(stderr,"ERROR: could not parse " #name "\n");\
	return -1;\
}\

#define PARSE_IMU_9DOF(name)\
if(parse_IMU_9DOF_gen_settings(jobj, #name, settings.name) < 0){\
	fprintf(stderr,"ERROR: could not parse " #name "\n");\
	return -1;\
}\


////////////////////////////////////////////////////////////////////////////////
/// functions for parsing enums
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      pulls rotor layout out of json object into settings struct
 *
 * @return     0 on success, -1 on failure
 */
static int __parse_layout(void)
{
	struct json_object* tmp = NULL;
	char* tmp_str = NULL;
	if (json_object_object_get_ex(jobj, "layout", &tmp) == 0) {
		fprintf(stderr, "ERROR: can't find layout in settings file\n");
		return -1;
	}
	if (json_object_is_type(tmp, json_type_string) == 0) {
		fprintf(stderr, "ERROR: layout should be a string\n");
		return -1;
	}
	tmp_str = (char*)json_object_get_string(tmp);
	if (strcmp(tmp_str, "LAYOUT_6DOF_ROTORBITS") == 0) {
		settings.num_rotors = 6;
		settings.layout = LAYOUT_6DOF_ROTORBITS;
	}
	else if (strcmp(tmp_str, "LAYOUT_4X") == 0) {
		settings.num_rotors = 4;
		settings.layout = LAYOUT_4X;
	}
	else if (strcmp(tmp_str, "LAYOUT_4PLUS") == 0) {
		settings.num_rotors = 4;
		settings.layout = LAYOUT_4PLUS;
	}
	else if (strcmp(tmp_str, "LAYOUT_4H") == 0) {
		settings.num_rotors = 4;
		settings.layout = LAYOUT_4H;
	}
	else if (strcmp(tmp_str, "LAYOUT_6X") == 0) {
		settings.num_rotors = 6;
		settings.layout = LAYOUT_6X;
	}
	else if (strcmp(tmp_str, "LAYOUT_8X") == 0) {
		settings.num_rotors = 8;
		settings.layout = LAYOUT_8X;
	}
	else if (strcmp(tmp_str, "LAYOUT_4X_ZEPPELIN") == 0) {
		settings.num_rotors = 4;
		settings.layout = LAYOUT_4X_ZEPPELIN;
	}
	else {
		fprintf(stderr, "ERROR: invalid layout string\n");
		return -1;
	}
	return 0;
}

/**
 * @brief      pulls servo layout out of json object into settings struct
 *
 * @return     0 on success, -1 on failure
 */
static int __parse_servo_layout(void)
{
	struct json_object* tmp = NULL;
	char* tmp_str = NULL;
	if (json_object_object_get_ex(jobj, "servo_layout", &tmp) == 0) {
		fprintf(stderr, "ERROR: can't find servo_layout in settings file\n");
		return -1;
	}
	if (json_object_is_type(tmp, json_type_string) == 0) {
		fprintf(stderr, "ERROR: layout should be a string\n");
		return -1;
	}
	tmp_str = (char*)json_object_get_string(tmp);
	if (strcmp(tmp_str, "LAYOUT_6xDIRECT_TEST") == 0) {
		settings.num_servos = 6;
		settings.servo_layout = LAYOUT_6xDIRECT_TEST;
	}
	else if (strcmp(tmp_str, "LAYOUT_16xZEPPELIN") == 0) {
		settings.num_servos = 16;
		settings.servo_layout = LAYOUT_16xZEPPELIN;
	}
	else {
		fprintf(stderr, "ERROR: invalid layout string\n");
		return -1;
	}
	return 0;
}


static int __parse_thrust_map(void)
{
	struct json_object* tmp = NULL;
	char* tmp_str = NULL;
	if (json_object_object_get_ex(jobj, "thrust_map", &tmp) == 0) {
		fprintf(stderr, "ERROR: can't find thrust_map in settings file\n");
		return -1;
	}
	if (json_object_is_type(tmp, json_type_string) == 0) {
		fprintf(stderr, "ERROR: thrust map should be a string\n");
		return -1;
	}
	tmp_str = (char*)json_object_get_string(tmp);
	if (strcmp(tmp_str, "LINEAR_MAP") == 0) {
		settings.thrust_map = LINEAR_MAP;
	}
	else if (strcmp(tmp_str, "MN1806_1400KV_4S") == 0) {
		settings.thrust_map = MN1806_1400KV_4S;
	}
	else if (strcmp(tmp_str, "RS2205_2600KV_3S") == 0) {
		settings.thrust_map = RS2205_2600KV_3S;
	}
	else if (strcmp(tmp_str, "RS2212_920_4S") == 0)
	{
		settings.thrust_map = RS2212_920_4S;
	}
	else if (strcmp(tmp_str, "RS2212_920_3S") == 0)
	{
		settings.thrust_map = RS2212_920_3S;
	}
	else if (strcmp(tmp_str, "TEAM2_PROP") == 0) {
		settings.thrust_map = TEAM2_PROP;
	}
	else if (strcmp(tmp_str, "F20_2300KV_2S") == 0) {
		settings.thrust_map = F20_2300KV_2S;
	}
	else if (strcmp(tmp_str, "RX2206_4S") == 0) {
		settings.thrust_map = RX2206_4S;
	}
	else {
		fprintf(stderr, "ERROR: invalid thrust_map string\n");
		return -1;
	}
	return 0;
}

/**
 * @brief      parses a json_object and fills in the flight mode.
 *
 * @param      jobj  The jobj to parse
 * @param      mode  pointer to write mode out to
 *
 * @return     returns 0 on success or -1 on failure
 */
static int __parse_flight_mode(json_object* jobj_str, flight_mode_t* mode)
{
	char* tmp_str = NULL;
	if (json_object_is_type(jobj_str, json_type_string) == 0) {
		fprintf(stderr, "ERROR: flight_mode should be a string\n");
		return -1;
	}
	tmp_str = (char*)json_object_get_string(jobj_str);
	if (strcmp(tmp_str, "TEST_BENCH_4DOF") == 0) {
		*mode = TEST_BENCH_4DOF;
	}
	else if (strcmp(tmp_str, "TEST_BENCH_6DOF") == 0) {
		*mode = TEST_BENCH_6DOF;
	}
	else if (strcmp(tmp_str, "TEST_6xSERVOS_DIRECT") == 0) {
		*mode = TEST_6xSERVOS_DIRECT;
	}
	else if (strcmp(tmp_str, "DIRECT_THROTTLE_6DOF") == 0) {
		*mode = DIRECT_THROTTLE_6DOF;
	}
	else if (strcmp(tmp_str, "ACRO_Axxxxx") == 0) {
		*mode = ACRO_Axxxxx;
	}
	else if (strcmp(tmp_str, "ACRO_Fxxxxx") == 0) {
		*mode = ACRO_Fxxxxx;
	}
	else if (strcmp(tmp_str, "MANUAL_xAxxxx") == 0) {
		*mode = MANUAL_xAxxxx;
	}
	else if (strcmp(tmp_str, "MANUAL_xFxxxx") == 0) {
		*mode = MANUAL_xFxxxx;
	}
	else if (strcmp(tmp_str, "MANUAL_AAxxxx") == 0) {
		*mode = MANUAL_AAxxxx;
	}
	else if (strcmp(tmp_str, "MANUAL_FAxxxx") == 0) {
		*mode = MANUAL_FAxxxx;
	}
	else if (strcmp(tmp_str, "MANUAL_FFxxxx") == 0) {
		*mode = MANUAL_FFxxxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_AxAxxx") == 0) {
		*mode = ALT_HOLD_AxAxxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FxAxxx") == 0) {
		*mode = ALT_HOLD_FxAxxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FxFxxx") == 0) {
		*mode = ALT_HOLD_FxFxxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_xAxAxx") == 0) {
		*mode = ALT_HOLD_xAxAxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_xFxAxx") == 0) {
		*mode = ALT_HOLD_xFxAxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_xFxFxx") == 0) {
		*mode = ALT_HOLD_xFxFxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_AxAAxx") == 0) {
		*mode = ALT_HOLD_AxAAxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FxAAxx") == 0) {
		*mode = ALT_HOLD_FxAAxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FxFAxx") == 0) {
		*mode = ALT_HOLD_FxFAxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FxFFxx") == 0) {
		*mode = ALT_HOLD_FxFFxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_AAAxxx") == 0) {
		*mode = ALT_HOLD_AAAxxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FAAxxx") == 0) {
		*mode = ALT_HOLD_FAAxxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FFAxxx") == 0) {
		*mode = ALT_HOLD_FFAxxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FFFxxx") == 0) {
		*mode = ALT_HOLD_FFFxxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_AAxAxx") == 0) {
		*mode = ALT_HOLD_AAxAxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FAxAxx") == 0) {
		*mode = ALT_HOLD_FAxAxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FFxAxx") == 0) {
		*mode = ALT_HOLD_FFxAxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FFxFxx") == 0) {
		*mode = ALT_HOLD_FFxFxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_AAAAxx") == 0) {
		*mode = ALT_HOLD_AAAAxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FAAAxx") == 0) {
		*mode = ALT_HOLD_FAAAxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FFAAxx") == 0) {
		*mode = ALT_HOLD_FFAAxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FFFAxx") == 0) {
		*mode = ALT_HOLD_FFFAxx;
	}
	else if (strcmp(tmp_str, "ALT_HOLD_FFFFxx") == 0) {
		*mode = ALT_HOLD_FFFFxx;
	}
	else if (strcmp(tmp_str, "POS_CTRL_AAAAAA") == 0) {
		*mode = POS_CTRL_AAAAAA;
	}
	else if (strcmp(tmp_str, "POS_CTRL_FFFAAx") == 0) {
		*mode = POS_CTRL_FFFAAx;
	}
	else if (strcmp(tmp_str, "POS_CTRL_FFFAFx") == 0) {
		*mode = POS_CTRL_FFFAFx;
	}
	else if (strcmp(tmp_str, "POS_CTRL_FFFFAx") == 0) {
		*mode = POS_CTRL_FFFFAx;
	}
	else if (strcmp(tmp_str, "POS_CTRL_FFFAAA") == 0) {
		*mode = POS_CTRL_FFFAAA;
	}
	else if (strcmp(tmp_str, "POS_CTRL_FFFAFA") == 0) {
		*mode = POS_CTRL_FFFAFA;
	}
	else if (strcmp(tmp_str, "POS_CTRL_FFFAFF") == 0) {
		*mode = POS_CTRL_FFFAFF;
	}
	else if (strcmp(tmp_str, "POS_CTRL_FFFFFF") == 0) {
		*mode = POS_CTRL_FFFFFF;
	}
	else if (strcmp(tmp_str, "EMERGENCY_LAND") == 0) {
		*mode = EMERGENCY_LAND;
	}
	else if (strcmp(tmp_str, "AUTO_FFFAFA") == 0) {
		*mode = AUTO_FFFAFA;
	}
	else if (strcmp(tmp_str, "ZEPPELIN") == 0) {
		*mode = ZEPPELIN;
	}
	else {
		fprintf(stderr, "ERROR: invalid flight mode\n");
		return -1;
	}
	return 0;
}



static int __parse_kill_mode(void)
{
	struct json_object* tmp = NULL;
	char* tmp_str = NULL;
	if (json_object_object_get_ex(jobj, "dsm_kill_mode", &tmp) == 0) {
		fprintf(stderr, "ERROR: can't find dsm_kill_mode in settings file\n");
		return -1;
	}
	if (json_object_is_type(tmp, json_type_string) == 0) {
		fprintf(stderr, "ERROR: dsm_kill_mode should be a string\n");
		return -1;
	}
	tmp_str = (char*)json_object_get_string(tmp);
	if (strcmp(tmp_str, "DSM_KILL_DEDICATED_SWITCH") == 0) {
		settings.dsm_kill_mode = DSM_KILL_DEDICATED_SWITCH;
	}
	else if (strcmp(tmp_str, "DSM_KILL_NEGATIVE_THROTTLE") == 0) {
		settings.dsm_kill_mode = DSM_KILL_NEGATIVE_THROTTLE;
	}
	else {
		fprintf(stderr, "ERROR: invalid dsm_kill_mode string\n");
		return -1;
	}
	return 0;
}

int settings_load_from_file(char* path)
{
	struct json_object* tmp = NULL; // temp object
	struct json_object* ctrl_json = NULL; // for parsing controllers
	//struct json_object* subctrl_json = NULL; //for parsing controllers (sublevel)

	was_load_successful = 0;

#ifdef DEBUG
	fprintf(stderr, "beginning of load_settings_from_file\n");
	fprintf(stderr, "about to check access of fly settings file\n");
#endif

	// read in file contents
	if (access(path, F_OK) != 0) {
		fprintf(stderr, "ERROR: settings file missing\n");
		return -1;
	}
	else {
#ifdef DEBUG
		printf("about to read json from file\n");
#endif
		jobj = json_object_from_file(path);
		if (jobj == NULL) {
			fprintf(stderr, "ERROR, failed to read settings from disk\n");
			return -1;
		}
	}

#ifdef DEBUG
	settings_print();
#endif

	// START PARSING

	PARSE_STRING(name);
#ifdef DEBUG
	fprintf(stderr, "name: %s\n", settings.name);
#endif
	PARSE_BOOL(warnings_en);
#ifdef DEBUG
	fprintf(stderr, "warnings: %d\n", settings.warnings_en);
#endif
	PARSE_BOOL(delay_warnings_en);
	PARSE_BOOL(telem_warnings_en);


	// PHYSICAL PARAMETERS
	// layout populates num_rotors, layout, and dof
	if (__parse_layout() == -1) return -1; // parse_layout also fill in num_rotors and dof
#ifdef DEBUG
	fprintf(stderr, "layout:%d,%d\n", settings.layout, settings.num_rotors);
#endif
	if (__parse_servo_layout() == -1) return -1;
	if (__parse_thrust_map() == -1) return -1;
#ifdef DEBUG
	fprintf(stderr, "thrust_map: %d\n", settings.thrust_map);
#endif
	PARSE_BATTERY(battery)
#ifdef DEBUG
		fprintf(stderr, "v_nominal: %f\n", settings.v_nominal);
#endif
	PARSE_IMU_9DOF(IMU0);
	PARSE_IMU_9DOF(IMU1);
	PARSE_MOCAP(mocap);
	PARSE_BOOL(enable_encoders);
	PARSE_BOOL(enable_gps);

	//Servo settings
	PARSE_BOOL(enable_servos);
	PARSE_BOOL(en_servo_ch_pool_sat);
	PARSE_INT(servo_i2c_driver_id);
	PARSE_DOUBLE_MIN_MAX(servos_arm_time_s, 0.0, 120.0);

	PARSE_DOUBLE_MIN_MAX(arm_time_s, 0.0, 120.0);

	// EMERGENCY LANDING SETTINGS
	PARSE_BOOL(enable_mocap_dropout_emergency_land);
	PARSE_DOUBLE_MIN_MAX(mocap_dropout_timeout_ms, 0, 10000);
	if (settings.enable_mocap_dropout_emergency_land)
	{
		printf("Mocap dropout emergency landing ENABLED.\tDropout timeout: %0.1lfms.\n",
			settings.mocap_dropout_timeout_ms);
	}
#ifdef DEBUG
	fprintf(stderr, "enable_mocap_dropout_emergency_land: %d\n", settings.enable_mocap_dropout_emergency_land);
	fprintf(stderr, "mocap_dropout_timeout_ms: %lf\n", settings.mocap_dropout_timeout_ms);
#endif   

	// FLIGHT MODES
	PARSE_INT_MIN_MAX(num_dsm_modes, 1, 3);
	if (json_object_object_get_ex(jobj, "flight_mode_1", &tmp) == 0) {
		fprintf(stderr, "ERROR: can't find flight_mode_1 in settings file\n");
		return -1;
	}
	if (__parse_flight_mode(tmp, &settings.flight_mode_1)) return -1;
#ifdef DEBUG
	fprintf(stderr, "flight_mode_1: %d\n", settings.flight_mode_1);
#endif
	if (json_object_object_get_ex(jobj, "flight_mode_2", &tmp) == 0) {
		fprintf(stderr, "ERROR: can't find flight_mode_2 in settings file\n");
		return -1;
	}
	if (__parse_flight_mode(tmp, &settings.flight_mode_2)) return -1;
#ifdef DEBUG
	fprintf(stderr, "flight_mode_2: %d\n", settings.flight_mode_2);
#endif
	if (json_object_object_get_ex(jobj, "flight_mode_3", &tmp) == 0) {
		fprintf(stderr, "ERROR: can't find flight_mode_3 in settings file\n");
		return -1;
	}
	if (__parse_flight_mode(tmp, &settings.flight_mode_3)) return -1;
#ifdef DEBUG
	fprintf(stderr, "flight_mode_3: %d\n", settings.flight_mode_3);
#endif
	printf("---\n"); // Just a visual break between above settings and the ones below

	// DSM RADIO CONFIG
	PARSE_BOOL(enable_dsm);
	PARSE_INT_MIN_MAX(dsm_thr_ch, 1, 9);
	PARSE_POLARITY(dsm_thr_pol);
	PARSE_INT_MIN_MAX(dsm_roll_ch, 1, 9);
	PARSE_POLARITY(dsm_roll_pol);
	PARSE_INT_MIN_MAX(dsm_pitch_ch, 1, 9);
	PARSE_POLARITY(dsm_pitch_pol);
	PARSE_INT_MIN_MAX(dsm_yaw_ch, 1, 9);
	PARSE_POLARITY(dsm_yaw_pol);
	PARSE_INT_MIN_MAX(dsm_mode_ch, 1, 9);
	PARSE_POLARITY(dsm_mode_pol);
	if (__parse_kill_mode() == -1) return -1;
#ifdef DEBUG
	fprintf(stderr, "kill_mode: %d\n", settings.dsm_kill_mode);
#endif
	PARSE_INT_MIN_MAX(dsm_kill_ch, 1, 9);
	PARSE_POLARITY(dsm_kill_pol);

	// PRINTF OPTIONS
	PARSE_BOOL(printf_arm);
	PARSE_BOOL(printf_altitude);
	PARSE_BOOL(printf_battery);
	PARSE_BOOL(printf_rpy);
	PARSE_BOOL(printf_sticks);

	PARSE_BOOL(printf_setpoint);
	PARSE_BOOL(printf_setpoint_xy);
	PARSE_BOOL(printf_setpoint_xy_dot);
	PARSE_BOOL(printf_setpoint_z);
	PARSE_BOOL(printf_setpoint_z_dot);
	PARSE_BOOL(printf_setpoint_att);
	PARSE_BOOL(printf_setpoint_att_dot);

	PARSE_BOOL(printf_u);
	PARSE_BOOL(printf_motors);
	PARSE_BOOL(printf_mode);
	PARSE_BOOL(printf_mocap);
	PARSE_BOOL(printf_gain_tunning);
	PARSE_BOOL(printf_tracking);
	PARSE_BOOL(printf_gps);


	// LOGGING
	PARSE_BOOL(enable_logging);
	PARSE_INT(log_every_n_entry);
	PARSE_BOOL(log_only_while_armed);
	PARSE_BOOL(log_sensors);
	PARSE_BOOL(log_state);
	PARSE_BOOL(log_gps);

	PARSE_BOOL(log_setpoints);
	PARSE_BOOL(log_throttles);
	PARSE_BOOL(log_throttles_ff);
	PARSE_BOOL(log_attitude_rate_setpoint);
	PARSE_BOOL(log_attitude_rate_setpoint_ff);
	PARSE_BOOL(log_attitude_setpoint);
	PARSE_BOOL(log_attitude_setpoint_ff);
	PARSE_BOOL(log_acceleration_setpoint);
	PARSE_BOOL(log_acceleration_setpoint_ff);
	PARSE_BOOL(log_velocity_setpoint);
	PARSE_BOOL(log_velocity_setpoint_ff);
	PARSE_BOOL(log_position_setpoint);
	PARSE_BOOL(log_position_setpoint_ff);

	PARSE_BOOL(log_dsm);
	PARSE_BOOL(log_flight_mode);
	PARSE_BOOL(log_benchmark);
	PARSE_BOOL(log_control_u);
	PARSE_BOOL(log_motor_signals);
	PARSE_BOOL(log_encoders);

	// MAVLINK
	PARSE_STRING(dest_ip);
	PARSE_INT(my_sys_id);
	PARSE_INT(mav_port);

	// DSM CONNECTION TIMER
	PARSE_INT(dsm_timeout_ms)

		// WAYPOINT FILES
		PARSE_STRING(wp_folder);
	PARSE_STRING(wp_takeoff_filename);
	PARSE_STRING(wp_guided_filename);
	PARSE_STRING(wp_landing_filename);

	// AUTOMATED FLIGHT
	PARSE_DOUBLE_MIN_MAX(V_max_land, 0.0, 1000000);
	PARSE_DOUBLE_MIN_MAX(height_takeoff, 0.0, 100000000);
	PARSE_DOUBLE_MIN_MAX(t_takeoff, 0.0, 10000000);
	PARSE_DOUBLE_MIN_MAX(max_XY_velocity, 0.0, 100000000);
	PARSE_DOUBLE_MIN_MAX(max_Z_velocity, 0.0, 10000000);
	PARSE_DOUBLE_MIN_MAX(hover_throttle, 0.0, 1.0);
	PARSE_DOUBLE_MIN_MAX(XY_start_delay_s, 0.0, 100.0);
	PARSE_DOUBLE_MIN_MAX(XY_waypt_delay_s, 0.0, 100.0);
	PARSE_DOUBLE_MIN_MAX(square_X_offset, -10000.0, 10000.0);
	PARSE_DOUBLE_MIN_MAX(square_Y_offset, -10000.0, 10000.0);
	PARSE_DOUBLE_MIN_MAX(square_X_time_s, 0.0, 1000.0);
	PARSE_DOUBLE_MIN_MAX(square_Y_time_s, 0.0, 1000.0);
	PARSE_DOUBLE_MIN_MAX(turn_radius, 0.0, 100.0);
	PARSE_DOUBLE_MIN_MAX(turn_period, 0.0, 100.0);
	PARSE_DOUBLE_MIN_MAX(turn_time_s, 0.0, 100.0);
	PARSE_BOOL(turn_dir_yaw_cw);

	// SERIAL PORTS
	PARSE_STRING(serial_port_1);
	PARSE_INT(serial_baud_1);
	PARSE_STRING(serial_port_2);
	PARSE_INT(serial_baud_2);
	PARSE_STRING(serial_port_gps);
	PARSE_INT(serial_baud_gps);

	// FEEDBACK CONTROLLERS
	PARSE_CONTROLLER(roll_rate_ctrl);
	PARSE_CONTROLLER(pitch_rate_ctrl);
	PARSE_CONTROLLER(yaw_rate_ctrl);
	PARSE_CONTROLLER(roll_ctrl);
	PARSE_CONTROLLER(pitch_ctrl);
	PARSE_CONTROLLER(yaw_ctrl);
	PARSE_CONTROLLER(X_vel_ctrl);
	PARSE_CONTROLLER(Y_vel_ctrl);
	PARSE_CONTROLLER(Z_vel_ctrl);
	PARSE_CONTROLLER(X_pos_ctrl);
	PARSE_CONTROLLER(Y_pos_ctrl);
	PARSE_CONTROLLER(Z_pos_ctrl);

	//REMOTE TUNING
	PARSE_BOOL(allow_remote_tuning);

	json_object_put(jobj);	// free memory
	was_load_successful = 1;
	return 0;
}




int settings_print(void)
{
	if(jobj == NULL){
		fprintf(stderr,"ERROR: NULL object passed to settings_print\n");
		return -1;
	}
	printf("settings:\n\n");
	printf("%s", json_object_to_json_string_ext(jobj, \
			JSON_C_TO_STRING_SPACED | JSON_C_TO_STRING_PRETTY));
	printf("\n");
	return 0;
}