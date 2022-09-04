/*
 * gyro_gen.cpp
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
 * This contains the nessesary framework for operating Gyroscope.
 */

#include "gyro_gen.hpp"
#include "settings_gen.hpp"
#include <rc/time.h>

#ifndef GET_VARIABLE_NAME
#define GET_VARIABLE_NAME(Variable) (#Variable)
#endif // !GET_VARIABLE_NAME


 /* General class for all GYRO instances */
char gyro_gen_t::init(gyro_gen_settings_t& new_gyro_settings)
{
	if (unlikely(initialized))
	{
		fprintf(stderr, "ERROR in init: gyro is already initialized.\n");
		return -1;
	}

	if (unlikely(lp.set(new_gyro_settings.filter) < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize gyro filters\n");
		return -1;
	}
	settings = new_gyro_settings;

	updated = false;
	initialized = true;
	return 0;
}
char gyro_gen_t::init(void)
{
	/* assume some default values */
	settings.frame_type = ENU;
	for (int i = 0; i < 3; i++) {
		settings.filter[i].dt = DT;
		settings.filter[i].type = Lowpass;
		settings.filter[i].enable_saturation = false;
	}
	settings.filter[0].tc = 6.0 * DT;
	settings.filter[1].tc = 6.0 * DT;
	settings.filter[2].tc = 7.0 * DT;

	return init(settings);
}

bool gyro_gen_t::is_initialized(void)
{
	return initialized;
}

char gyro_gen_t::update(double new_gyro_raw[3])
{
	if (updated)
	{
		printf("WARNING in update: already updated gyro data.\n");
		return 0; //skip if already updated
	}
	/* save new mocap data internally */
	raw[0] = new_gyro_raw[0];
	raw[1] = new_gyro_raw[1];
	raw[2] = new_gyro_raw[2];

	/* transform from Gyro frame to NED */
	rotate2NED(settings.frame_type, raw_NED, raw);

	updated = true;
	time = rc_nanos_since_boot(); //update timer
	return 0;
}

char gyro_gen_t::march(void)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in march: gyro is not initialized.\n");
		return -1;
	}
	if (updated)
	{
		lp.march(filtered_NED, raw_NED);
		updated = false;
	}
	return 0;
}

uint64_t gyro_gen_t::get_time(void)
{
	return time;
}
void gyro_gen_t::get_raw(double* buff)
{
	for (int i = 0.0; i < 3; i++)
	{
		buff[i] = raw[i];
	}
	return;
}

void gyro_gen_t::get_raw_NED(double* buff)
{
	for (int i = 0.0; i < 3; i++)
	{
		buff[i] = raw_NED[i];
	}
	return;
}

void gyro_gen_t::get(double* buff)
{
	for (int i = 0.0; i < 3; i++)
	{
		buff[i] = filtered_NED[i];
	}
	return;
}

char gyro_gen_t::reset(void)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in reset: gyro is not initialized.\n");
		return -1;
	}
	updated = false;
	return lp.reset();
}

void gyro_gen_t::cleanup(void)
{
	if (!initialized) return;
	lp.cleanup();
	initialized = false;
	updated = false;
	return;
}

/** @name Logging class for gyro
* Defines how logging should be done for this class
*/
char gyro_log_entry_t::update(gyro_gen_t& new_state, gyro_gen_settings_t& new_settings)
{
	if (!new_settings.enable_logging) return 0;//return if disabled

	time = new_state.get_time();
	if (new_settings.log_raw)
	{
		new_state.get_raw(raw);
		new_state.get_raw_NED(raw_NED);
	}
	new_state.get(filtered_NED);

	return 0;
}
char gyro_log_entry_t::print_vec(FILE* file, double* vec_in, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%.4F", vec_in[i]);
	}
	return 0;
}

char gyro_log_entry_t::print_header_vec(FILE* file, const char* prefix, const char* var_name, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%s%s_%i", prefix, var_name, i);
	}
	return 0;
}

char gyro_log_entry_t::print_header(FILE* file, const char* prefix, gyro_gen_settings_t& new_settings)
{
	if (!new_settings.enable_logging) return 0;//return if disabled

	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(time));
	if (new_settings.log_raw) {
		print_header_vec(file, prefix, GET_VARIABLE_NAME(raw), 3);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(raw_NED), 3);
	}
	print_header_vec(file, prefix, GET_VARIABLE_NAME(filtered_NED), 3);
	return 0;
}
char gyro_log_entry_t::print_entry(FILE* file, gyro_gen_settings_t& new_settings)
{
	if (!new_settings.enable_logging) return 0;//return if disabled

	fprintf(file, ",%" PRIu64, time);
	if (new_settings.log_raw)
	{
		print_vec(file, raw, 3);
		print_vec(file, raw_NED, 3);
	}
	print_vec(file, filtered_NED, 3);
	return 0;
}

int parse_gyro_gen_settings(json_object* in_json, const char* name, gyro_gen_settings_t& sensor)
{
	struct json_object* tmp_main_json = NULL;
	struct json_object* tmp = NULL;    // temp object

	//find filter entry
	if (json_object_object_get_ex(in_json, name, &tmp_main_json) == 0)
	{
		fprintf(stderr, "ERROR: can't find %s entry\n", name);
		return -1;
	}
	if (json_object_is_type(tmp_main_json, json_type_object) == 0)
	{
		fprintf(stderr, "ERROR: %s must be an object\n", name);
		return -1;
	}

	// Parse coordinate frame type:
	if (parse_coordinate_frame_gen_type(tmp_main_json, "frame_type", sensor.frame_type))
	{
		fprintf(stderr, "ERROR: failed to parse frame_type flag for %s\n", name);
		return -1;
	}

	// Parse filter
	char buf[8 + 3];
	for (int i = 0; i < 3; i++)
	{
		snprintf(buf, 11, "filter[%i]", i); // puts string into buffer
		if (parse_signal_filter_gen_settings(tmp_main_json, buf, sensor.filter[i]) < 0)
		{
			fprintf(stderr, "ERROR: failed to parse filter for %s\n", name);
			return -1;
		}
	}

	// Parse other entries
	if (parse_bool(tmp_main_json, "enable_logging", sensor.enable_logging))
	{
		fprintf(stderr, "ERROR: failed to parse enable_logging flag for %s\n", name);
		return -1;
	}

	if (!sensor.enable_logging) return 0; //done if logging is disabled
	if (parse_bool(tmp_main_json, "log_raw", sensor.log_raw))
	{
		fprintf(stderr, "ERROR: failed to parse log_raw flag for %s\n", name);
		return -1;
	}

	return 0;
}