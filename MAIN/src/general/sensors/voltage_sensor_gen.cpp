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
 * Last Edit:  09/03/2022 (MM/DD/YYYY)
 *
 * Summary :
 * This contains the nessesary framework for operating voltage sensors
 */

#include "voltage_sensor_gen.hpp"
#include "settings_gen.hpp"

#include <rc/adc.h>
#include <cstring>

#ifndef GET_VARIABLE_NAME
#define GET_VARIABLE_NAME(Variable) (#Variable)
#endif // !GET_VARIABLE_NAME

voltage_sensor_gen_t batt{};	//battery voltage sensor

/* Methods for general BATTERY class */
char voltage_sensor_gen_t::init(voltage_sensor_settings_t& new_settings)
{
	if (unlikely(initialized))
	{
		fprintf(stderr, "ERROR in init: battery already initialized.\n");
		return -1;
	}
	if (new_settings.nominal < new_settings.min_critical)
	{
		fprintf(stderr, "ERROR in init: Nominal voltage (%4.1fV) must be greater than critical minium of %4.1fV.\n", new_settings.nominal, new_settings.min_critical);
		return -1;
	}
	// init the battery filter
	if (unlikely(filter.set(new_settings.filter) < 0))
	{
		fprintf(stderr, "ERROR in init: failted to initialize battery filter.\n");
		return -1;
	}

	initialized = true;
	return 0;
}
char voltage_sensor_gen_t::init(voltage_sensor_settings_t& new_settings, double new_in)
{
	if (unlikely(initialized))
	{
		fprintf(stderr, "ERROR in init: battery already initialized.\n");
		return -1;
	}
	if (new_settings.nominal < new_settings.min_critical)
	{
		fprintf(stderr, "ERROR in init: Nominal voltage (%4.1fV) must be greater than critical minium of %4.1fV.\n", new_settings.nominal, new_settings.min_critical);
		return -1;
	}
	// init the battery filter
	if (unlikely(filter.set(new_settings.filter) < 0))
	{
		fprintf(stderr, "ERROR in init: failted to initialize battery filter.\n");
		return -1;
	}

	if (new_in < new_settings.min_critical) {
		if (new_settings.enable_warnings) {
			fprintf(stderr, "WARNING in init: voltage read is %2.1fV (too low).\n", new_in);
			fprintf(stderr, "Assuming nominal voltage for now.\n");
		}
		new_in = new_settings.nominal;
	}

	settings = new_settings;

	filter.prefill_inputs(new_in);
	filter.prefill_outputs(new_in);

	initialized = true;
	return 0;
}
char voltage_sensor_gen_t::init(void)
{
	/* assume some default values */
	settings.enable_gain_scaling = false;
	settings.filter.type = Moving_Avg;
	settings.filter.dt = DT;
	settings.filter.n_samples = 20;
	settings.enable_warnings = true;
	settings.min_critical = 3.0;
	settings.nominal = 3.3;

	return init(settings, rc_adc_dc_jack());
}


char voltage_sensor_gen_t::march(double new_v)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in march: battery is not initialized.\n");
		return -1;
	}
	raw = new_v;

	if (new_v < settings.min_critical)
	{
		if (settings.enable_warnings)
		{
			printf("WARNING in march: measured voltage is below critical, assuming measured is equal to critical voltage.\n");
		}
		filtered = filter.march(settings.min_critical);
		return 0;
	}

	filtered = filter.march(raw);
	return 0;
}

double voltage_sensor_gen_t::get_raw(void)
{
	return raw;
}
double voltage_sensor_gen_t::get(void)
{
	return filtered;
}

char voltage_sensor_gen_t::reset(void)
{
	return filter.reset();;
}
char voltage_sensor_gen_t::reset(voltage_sensor_settings_t new_settings)
{
	initialized = false;
	reset();
	return init(new_settings);
}

void voltage_sensor_gen_t::cleanup(void)
{
	if (!initialized) return;
	filter.cleanup();
	initialized = false;
	return;
}


/** @name Logging class for battery
* Defines how logging should be done for this class
*/
char voltage_sensor_log_entry_t::update(voltage_sensor_gen_t& new_state, voltage_sensor_settings_t& new_settings)
{
	if (!new_settings.enable_logging) return 0; //return if disabled
	if (new_settings.log_raw) raw = new_state.get_raw();
	filtered = new_state.get();
	return 0;
}
char voltage_sensor_log_entry_t::print_vec(FILE* file, double* vec_in, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%.4F", vec_in[i]);
	}
	return 0;
}

char voltage_sensor_log_entry_t::print_header_vec(FILE* file, const char* prefix, const char* var_name, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%s%s_%i", prefix, var_name, i);
	}
	return 0;
}

char voltage_sensor_log_entry_t::print_header(FILE* file, const char* prefix, voltage_sensor_settings_t& new_settings)
{
	if (!new_settings.enable_logging) return 0; //return if disabled
	if (new_settings.log_raw) fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(raw));
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(filtered));
	return 0;
}
char voltage_sensor_log_entry_t::print_entry(FILE* file, voltage_sensor_settings_t& new_settings)
{
	if (!new_settings.enable_logging) return 0; //return if disabled
	if (new_settings.log_raw) fprintf(file, ",%.4F", raw);
	fprintf(file, ",%.4F", filtered);
	return 0;
}


int parse_voltage_sensor_gen_settings(json_object* in_json, const char* name, voltage_sensor_settings_t& sensor)
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

	// Parse filter
	if (parse_signal_filter_gen_settings(tmp_main_json, "filter", sensor.filter) < 0)
	{
		fprintf(stderr, "ERROR: failed to parse filter for %s\n", name);
		return -1;
	}

	// Parse other entries
	if (parse_bool(tmp_main_json, "enable_gain_scaling", sensor.enable_gain_scaling))
	{
		fprintf(stderr, "ERROR: failed to parse enable_gain_scaling flag for %s\n", name);
		return -1;
	}
	if (parse_bool(tmp_main_json, "enable_warnings", sensor.enable_warnings))
	{
		fprintf(stderr, "ERROR: failed to parse enable_warnings flag for %s\n", name);
		return -1;
	}

	if (parse_double_min(tmp_main_json, "nominal", sensor.nominal, 3.1))
	{
		fprintf(stderr, "ERROR: failed to parse nominal voltage for %s\n", name);
		return -1;
	}
	if (parse_double_min(tmp_main_json, "min_critical", sensor.min_critical, 3.0))
	{
		fprintf(stderr, "ERROR: failed to parse critical minimum voltage for %s\n", name);
		return -1;
	}

	if (parse_bool(tmp_main_json, "enable_logging", sensor.enable_logging))
	{
		fprintf(stderr, "ERROR: failed to parse enable_logging flag for %s\n", name);
		return -1;
	}
	if (!sensor.enable_logging) return 0; //return if disabled
	if (parse_bool(tmp_main_json, "log_raw", sensor.log_raw))
	{
		fprintf(stderr, "ERROR: failed to parse log_raw flag for %s\n", name);
		return -1;
	}


	return 0;
}