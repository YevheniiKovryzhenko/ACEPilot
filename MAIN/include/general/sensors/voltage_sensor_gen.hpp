/*
 * voltage_sensor_gen.hpp
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
 * Last Edit:  09/02/2022 (MM/DD/YYYY)
 *
 * Summary :
 * This contains the nessesary framework for operating voltage sensors
 */

#ifndef VOLTAGE_SENSOR_GEN_HPP
#define VOLTAGE_SENSOR_GEN_HPP
#include <json.h>

#include "signal_filter_gen.hpp"


 /* Voltage sensor */
typedef struct voltage_sensor_settings_t
{
	signal_filter_gen_settings_t filter; //filter settings
	bool en_warnings;// = false;
	double nominal;// = 5.0;
	double min_critical;// = 3.0;
}voltage_sensor_settings_t;

/* General class for all battery instances */
class voltage_sensor_gen_t
{
private:
	bool initialized = false;
	double raw = 0.0;
	double filtered = 0.0;

	voltage_sensor_settings_t settings; //sensor settings

	// battery filter
	signal_filter1D_gen_t filter{};
public:
	char init(void);
	char init(voltage_sensor_settings_t new_settings);
	char init(voltage_sensor_settings_t new_settings, double new_in);
	bool is_initialized(void);

	char march(double new_v);

	double get_raw(void);
	double get(void);

	char reset(void);
	char reset(voltage_sensor_settings_t new_settings);
	void cleanup(void);
};
extern voltage_sensor_gen_t batt;

/** @name Logging class for battery
* Defines how logging should be done for this class
*/
class battery_log_entry_t
{
private:
	double raw;
	double filtered;

	char print_vec(FILE* file, double* vec_in, int size);
	char print_header_vec(FILE* file, const char* prefix, const char* var_name, int size);
public:
	char update(voltage_sensor_gen_t* new_state);
	char print_header(FILE* file, const char* prefix);
	char print_entry(FILE* file);
};


int parse_voltage_sensor_gen_settings(json_object* in_json, const char* name, voltage_sensor_settings_t& sensor);

#endif // VOLTAGE_SENSOR_GEN_HPP
