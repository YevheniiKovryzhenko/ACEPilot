/*
 * accel_gen.hpp
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
 * Summary :
 * This contains the nessesary framework for operating Accelerometer.
 */

#ifndef ACCEL_GEN_HPP
#define ACCEL_GEN_HPP
#include <json.h>
#include <rc/time.h>
#include "coordinate_frames_gen.hpp"
#include "signal_filter_gen.hpp"

/* Accelerometer */
typedef struct accel_settings_t
{
	coordinate_frames_gen_t frame_type;// = ENU; //physical placemet of the sensor
	signal_filter_gen_settings_t filter[3]; //filter settings
	bool enable_logging;
	bool log_raw;
}accel_settings_t;

/* General class for all accelerometer instances */
class accel_gen_t
{
private:
	bool initialized = false;
	uint64_t time = rc_nanos_since_boot();
	bool updated = false;

	accel_settings_t settings;  //settings for the accelerometer

	double raw[3] = { 0.0 , 0.0, 0.0 };
	double raw_NED[3] = { 0.0 , 0.0, 0.0 };
	double filtered_NED[3] = { 0.0 , 0.0, 0.0 };

	// accelerometer filters
	signal_filter3D_gen_t lp{};
public:
	/* Initialization */
	char init(void);
	char init(accel_settings_t new_acc_settings);
	bool is_initialized(void);

	/* Updating */
	char update(double new_acc_raw[3]);
	char march(void); //marches filters only is was updated

	/* Data retrieval */
	uint64_t get_time(void);
	void get_raw(double* buff);
	void get_raw_NED(double* buff);
	void get(double* buff);

	/* Reset and cleanup */
	char reset(void);
	void cleanup(void);
};


/** @name Logging class for accel
* Defines how logging should be done for this class
*/
class accel_log_entry_t
{
private:
	uint64_t time;

	double raw[3];
	double raw_NED[3];
	double filtered_NED[3];

	char print_vec(FILE* file, double* vec_in, int size);
	char print_header_vec(FILE* file, const char* prefix, const char* var_name, int size);
public:
	char update(accel_gen_t& new_state, accel_settings_t& new_settings);
	char print_header(FILE* file, const char* prefix, accel_settings_t& new_settings);
	char print_entry(FILE* file, accel_settings_t& new_settings);
};

int parse_accel_gen_settings(json_object* in_json, const char* name, accel_settings_t& sensor);

#endif // !ACCEL_GEN_HPP
