/*
 * signal_filter_gen.cpp
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
 * Last Edit:  09/04/2022 (MM/DD/YYYY)
 *
 * Summary :
 * General-purpose class for applying simple filtering on a signal.
 *
 */
#include "settings_gen.hpp"
#include "signal_filter_gen.hpp"

#include <cstring>


// preposessor macros
#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif // !unlikely

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif // !likely

 /**
 * Setpoint filter class.
 */
// depricated {
bool signal_filter_gen_t::is_init(void)
{
	return initialized;
}
int signal_filter_gen_t::set_type(signal_filter_gen_type_t type)
{
	filter = RC_FILTER_INITIALIZER;
	switch (type)
	{
	case Lowpass:
		if (unlikely(rc_filter_first_order_lowpass(&filter, dt, tc) == -1))
		{
			printf("ERROR in set_type: failed to create low-pass filter\n");
			disable();
			reset();
			initialized = false;
			return -1;
		}

		break;
	case Highpass:
		if (unlikely(rc_filter_first_order_highpass(&filter, dt, tc) == -1))
		{
			printf("ERROR in set_type: failed to create high-pass filter\n");
			disable();
			reset();
			initialized = false;
			return -1;
		}
		break;

	case Integrator:
		if (unlikely(rc_filter_integrator(&filter, dt) == -1))
		{
			printf("ERROR in set_type: failed to create integrator\n");
			disable();
			reset();
			initialized = false;
			return -1;
		}

		break;

	default:
		printf("ERROR in set_type: undefined filter option.\n");
		disable();
		reset();
		initialized = false;
		return -1;
	}
	if (en_saturation)
	{
		if (unlikely(rc_filter_enable_saturation(&filter, min, max) == -1))
		{
			printf("ERROR in set_type: failed to enable saturation\n");
			return -1;
		}
	}


	reset();

	initialized = true;

	return 0;
}
bool signal_filter_gen_t::is_en(void)
{
	return en;
}
int signal_filter_gen_t::enable(void)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in enable: not initialized\n");
		return -1;
	}

	en = true;

	return 0;
}
int signal_filter_gen_t::disable(void)
{
	en = false;
	last_en = false;

	return 0;
}
int signal_filter_gen_t::reset(void)
{
	output = 0.0;
	input = 0.0;

	if (unlikely(rc_filter_reset(&filter) < 0))
	{
		printf("ERROR in reset: failed to reset filter\n");
		disable();
		return -1;
	}


	return 0;
}
double signal_filter_gen_t::get(void)
{
	return output;
}
int signal_filter_gen_t::set_dt(double in)
{
	dt = in;
	return 0;
}
int signal_filter_gen_t::set_tc(double in)
{
	tc = in;
	return 0;
}
int signal_filter_gen_t::set_min_max(double new_min, double new_max)
{
	min = new_min;
	max = new_max;
	return 0;
}
int signal_filter_gen_t::set_saturation(bool in)
{
	en_saturation = in;

	return 0;
}
int signal_filter_gen_t::update(double in)
{
	if (!en) return 0;

	if (unlikely(!initialized))
	{
		printf("ERROR in update: not initialized\n");
		return -1;
	}
	input = in;
	output = rc_filter_march(&filter, input);

	if (en_saturation) rc_saturate_double(&output, min, max);


	return 0;
}




int signal_filter_triplet_gen_t::init_all(void)
{
	if (unlikely(lowpass.set_type(Lowpass) < 0))
	{
		printf("ERROR in init_filters: failed to initialize low-pass filter\n");
		filters_initialized = false;
		return -1;
	}
	if (unlikely(highpass.set_type(Highpass) < 0))
	{
		printf("ERROR in init_filters: failed to initialize high-pass filter\n");
		filters_initialized = false;
		return -1;
	}
	if (unlikely(integrator.set_type(Integrator) < 0))
	{
		printf("ERROR in init_filters: failed to initialize inegrator filter\n");
		filters_initialized = false;
		return -1;
	}

	filters_initialized = true;
	return 0;
}
int signal_filter_triplet_gen_t::update_all(double in_lp, double in_hp, double in_i)
{
	lowpass.update(in_lp);
	highpass.update(in_hp);
	integrator.update(in_i);
	return 0;
}
int signal_filter_triplet_gen_t::stop_all(void)
{
	lowpass.disable();
	highpass.disable();
	integrator.disable();
	return 0;
}
int signal_filter_triplet_gen_t::reset_all(void)
{
	stop_all();
	lowpass.reset();
	highpass.reset();
	integrator.reset();
	return 0;
}
// } depricated


/* Parser for filter type */
int parse_singal_filter_gen_type(json_object* in_json, const char* name, signal_filter_gen_type_t& type)
{
	struct json_object* tmp_main_json = NULL;    // temp object

	//find and parse entry
	if (json_object_object_get_ex(in_json, name, &tmp_main_json) == 0)
	{
		fprintf(stderr, "ERROR: can't find %s entry\n", name);
		return -1;
	}
	if (json_object_is_type(tmp_main_json, json_type_string) == 0)
	{
		fprintf(stderr, "ERROR: %s must be a string\n", name);
		return -1;
	}

	char* tmp_str = NULL;
	if (json_object_is_type(tmp_main_json, json_type_string) == 0) {
		fprintf(stderr, "ERROR: %s should be a string\n", name);
		return -1;
	}
	tmp_str = (char*)json_object_get_string(tmp_main_json);
	if (strcmp(tmp_str, "Lowpass") == 0) {
		type = Lowpass;
	}
	else if (strcmp(tmp_str, "Highpass") == 0) {
		type = Highpass;
	}
	else if (strcmp(tmp_str, "Integrator") == 0) {
		type = Integrator;
	}
	else if (strcmp(tmp_str, "Moving_Avg") == 0) {
		type = Moving_Avg;
	}
	else {
		fprintf(stderr, "ERROR: invalid type\n");
		return -1;
	}

	return 0;
}

int parse_signal_filter_gen_settings(json_object* in_json, const char* name, signal_filter_gen_settings_t& filter)
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

	// Parse filter type
	if (parse_singal_filter_gen_type(tmp_main_json, "type", filter.type) < 0)
	{
		fprintf(stderr, "ERROR: failed to parse filter type for %s\n", name);
		return -1;
	}

	// Parse other entries
	if (parse_double_positive(tmp_main_json, "dt", filter.dt))
	{
		fprintf(stderr, "ERROR: failed to parse time step dt for %s\n", name);
		return -1;
	}
	if (filter.dt < 0)
	{
		fprintf(stderr, "ERROR: time step dt should be positive for %s\n", name);
		return -1;
	}
	switch (filter.type)
	{
	case Lowpass:
		if (parse_double_positive(tmp_main_json, "tc", filter.tc))
		{
			fprintf(stderr, "ERROR: failed to parse time constant tc for %s\n", name);
			return -1;
		}
		if (filter.tc < 0)
		{
			fprintf(stderr, "ERROR: time constant tc should be positive for %s\n", name);
			return -1;
		}
		break;
	case Highpass:
		if (parse_double_positive(tmp_main_json, "tc", filter.tc))
		{
			fprintf(stderr, "ERROR: failed to parse time constant tc for %s\n", name);
			return -1;
		}
		if (filter.tc < 0)
		{
			fprintf(stderr, "ERROR: time constant tc should be positive for %s\n", name);
			return -1;
		}
		break;
	case Integrator:
		//nothing extra for now...
		break;
	case Moving_Avg:
		if (parse_int_positive(tmp_main_json, "n_samples", filter.n_samples))
		{
			fprintf(stderr, "ERROR: failed to parse n_samples for %s\n", name);
			return -1;
		}
		if (filter.n_samples < 0)
		{
			fprintf(stderr, "ERROR: time constant n_samples should be positive for %s\n", name);
			return -1;
		}
		break;
	default:
		fprintf(stderr, "ERROR: something went wrong parsing filter type for %s", name);
		return -1;
	}

	if (json_object_object_get_ex(tmp_main_json, name, &tmp)) {
		if (parse_bool(tmp_main_json, "en_saturation", filter.enable_saturation))
		{
			fprintf(stderr, "ERROR: failed to parse en_saturation for %s\n", name);
			return -1;
		}
		if (parse_double_min_max(tmp_main_json, filter.min, filter.max))
		{
			fprintf(stderr, "ERROR: failed to parse min max for %s\n", name);
			return -1;
		}
	}
	else
	{
		filter.enable_saturation = false; //just in case
	}


	return 0;
}


/* Methods for general low-pass filter class */
char signal_filter1D_gen_t::set(signal_filter_gen_settings_t new_settings)
{
	filter = RC_FILTER_INITIALIZER;
	switch (new_settings.type)
	{
	case Lowpass:
		if (unlikely(rc_filter_first_order_lowpass(&filter, new_settings.dt, new_settings.tc) == -1))
		{
			printf("ERROR in set: failed to create low-pass filter with dt=%f and tc=%f\n", new_settings.dt, new_settings.tc);
			reset();
			return -1;
		}

		break;
	case Highpass:
		if (unlikely(rc_filter_first_order_highpass(&filter, new_settings.dt, new_settings.tc) == -1))
		{
			printf("ERROR in set: failed to create high-pass filter with dt=%f and tc=%f\n", new_settings.dt, new_settings.tc);
			reset();
			return -1;
		}
		break;

	case Integrator:
		if (unlikely(rc_filter_integrator(&filter, new_settings.dt) == -1))
		{
			printf("ERROR in set: failed to create integrator with dt=%f\n", new_settings.dt);
			reset();
			return -1;
		}

		break;
	case Moving_Avg:
		if (unlikely(rc_filter_moving_average(&filter, new_settings.n_samples, new_settings.dt) == -1))
		{
			printf("ERROR in set: failed to create moving average filter with n_samples=%f and tc=%f\n", new_settings.n_samples, new_settings.dt);
			reset();
			return -1;
		}

		break;
	default:
		printf("ERROR in set: undefined filter option.\n");
		reset();
		return -1;
	}
	if (new_settings.enable_saturation)
	{
		if (unlikely(rc_filter_enable_saturation(&filter, new_settings.min, new_settings.max) == -1))
		{
			printf("ERROR in set: failed to enable saturation\n");
			return -1;
		}
	}
	settings = new_settings;
	return 0;
}
char signal_filter1D_gen_t::prefill_inputs(double new_in)
{
	return rc_filter_prefill_inputs(&filter, new_in);
}
char signal_filter1D_gen_t::prefill_outputs(double new_out)
{
	return rc_filter_prefill_outputs(&filter, new_out);
}
double signal_filter1D_gen_t::march(double new_raw)
{
	return rc_filter_march(&filter, new_raw);
}
char signal_filter1D_gen_t::enable_saturation(double new_min, double new_max)
{
	if (unlikely(rc_filter_enable_saturation(&filter, new_min, new_max) == -1))
	{
		printf("ERROR in enable_saturation: failed to enable saturation\n");
		return -1;
	}
	settings.min = new_min;
	settings.max = new_max;
	settings.enable_saturation = true;
	return 0;
}
char signal_filter1D_gen_t::reset(void)
{
	if (unlikely(rc_filter_reset(&filter) < 0))
	{
		fprintf(stderr, "ERROR in reset: failed to reset filter\n");
		return -1;
	}
	settings.enable_saturation = false;
	return 0;
}
void signal_filter1D_gen_t::cleanup(void)
{
	rc_filter_free(&filter);
	return;
}


/* Methods for general low-pass filter triplet class */
char signal_filter3D_gen_t::set(signal_filter_gen_settings_t new_settings[3])
{
	for (int i = 0; i < 3; i++)
	{
		if (unlikely(filter[i].set(new_settings[i]) < 0))
		{
			fprintf(stderr, "ERROR in set: failed to set %i-th filter\n", i);
			return -1;
		}
	}
	return 0;
}
char signal_filter3D_gen_t::prefill_inputs(double new_in[3])
{
	char tmp = 0;
	for (int i = 0; i < 3; i++) tmp += filter[i].prefill_inputs(new_in[i]);
	return tmp;
}
char signal_filter3D_gen_t::prefill_outputs(double new_out[3])
{
	char tmp = 0;
	for (int i = 0; i < 3; i++) tmp += filter[i].prefill_outputs(new_out[i]);
	return tmp;
}
void signal_filter3D_gen_t::march(double(&out)[3], double new_raw[3])
{
	for (int i = 0; i < 3; i++)
	{
		out[i] = filter[i].march(new_raw[i]);
	}
	return;
}

char signal_filter3D_gen_t::reset(void)
{
	for (int i = 0; i < 3; i++)
	{
		if (unlikely(filter[i].reset() < 0))
		{
			fprintf(stderr, "ERROR in reset: failed to reset %i-th filter\n", i);
			return -1;
		}
	}
	return 0;
}

void signal_filter3D_gen_t::cleanup(void)
{
	for (int i = 0; i < 3; i++)
	{
		filter[i].cleanup();
	}
	return;
}