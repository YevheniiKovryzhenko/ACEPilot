/*
 * filter_gen.cpp
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
 * Last Edit:  09/20/2022 (MM/DD/YYYY)
 */
#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include <rc/math.h>
#include <rc/time.h>
#include <rc/math/filter.h>
#include "feedback_filter_gen.hpp"
#include "settings_gen.hpp"

// preposessor macros
#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif // !unlikely

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif // !likely

/**
* @brief      Sets the default PD and I gains for the control system.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::set_default_gain_set(controller_settings_t& new_ctrl)
{
	if (unlikely(rc_filter_duplicate(&def_gain_pd, new_ctrl.pd) == -1))
	{
		printf("Error in set_default_gain_set: failed to dublicate PD controller\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&def_gain_i, new_ctrl.i) == -1))
	{
		printf("Error in set_default_gain_set: failed to dublicate I controller\n");
		return -1;
	}

	gain_K = new_ctrl.K;
	def_gain_FF = new_ctrl.FF;

	return 0;
}

/**
* @brief      Sets the active PD and I gains for the control system.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::set_gain_set(controller_settings_t& new_ctrl)
{
	if (unlikely(rc_filter_duplicate(&gain_pd, new_ctrl.pd) == -1))
	{
		printf("Error in set_gain_set: failed to dublicate PD controller\n");
		return -1;
	}
	if (unlikely(rc_filter_duplicate(&gain_i, new_ctrl.i) == -1))
	{
		printf("Error in set_gain_set: failed to dublicate I controller\n");
		return -1;
	}

	gain_K = new_ctrl.K;
	gain_FF = new_ctrl.FF;

	return 0;
}

/**
* @brief      Initializes the control system.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::init(controller_settings_t& new_ctrl)
{
	if (unlikely(set_gain_set(new_ctrl) < 0))
	{
		printf("Error in init: failed to set controller gains\n");
		return -1;
	}
	if (unlikely(set_default_gain_set(new_ctrl) < 0))
	{
		printf("Error in init: failed to set default controller gains\n");
		return -1;
	}
	initialized = true;
	return 0;
}

/**
* @brief      Resets the control system to default gains, zeros out inputs/outputs, etc.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::reset(void)
{
	gain_pd = def_gain_pd;
	gain_i = def_gain_i;
	gain_FF = def_gain_FF;
	gain_pd.gain = def_gain_pd.gain;
	gain_i.gain = def_gain_i.gain;

	if (unlikely(rc_filter_reset(&gain_pd) < 0))
	{
		printf("ERROR in reset: failed to reset PD control system\n");
		return -1;
	}
	if (unlikely(rc_filter_reset(&gain_i) < 0))
	{
		printf("ERROR in reset: failed to reset I control system\n");
		return -1;
	}

	return 0;
}


/**
* @brief      Marches the control system forward with new error and referece inputs.
*
* Must be initialized. Error (ref_in - st_in) input path goes though PI control system. 
* Reference path is added directly to the output of the PID control system. 
* Damping (derivative) is applied only on negative state (-st_in).
* PD control system must have p gain set to zero and gain_pd.gain=gain_i.gain for this to function properly.
* Out is the compined output of the system.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::march_std(double &out, double ref_in, double st_in)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in march: not initialized\n");
		return -1;
	}
	out = gain_pd.gain * gain_K * (ref_in - st_in)\
		+ rc_filter_march(&gain_pd, -gain_K * st_in)\
		+ rc_filter_march(&gain_i, gain_K * (ref_in - st_in))\
		+ gain_FF * ref_in;


	return 0;
}
int feedback_filter_gen_t::march_std(double& out, double err_in, double st_dot_in, double FF_in)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in march: not initialized\n");
		return -1;
	}
	err_in = gain_K * err_in;
	st_dot_in = -gain_K * st_dot_in;
	FF_in = gain_FF * FF_in;

	out = gain_pd.gain * err_in\
		+ rc_filter_march(&gain_pd, st_dot_in)\
		+ rc_filter_march(&gain_i, err_in)\
		+ FF_in;
	return 0;
}


/**
* @brief      Marches the control system forward with new error and referece inputs.
*
* Must be initialized. Error input path goes though PID control system. Reference path is added
* directly to the output of the PID control system.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::march(double &out, double err_in, double ref_in)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in march: not initialized\n");
		return -1;
	}
	out = rc_filter_march(&gain_pd, gain_K * err_in)\
		+ rc_filter_march(&gain_i, gain_K * err_in)\
		+ gain_FF * ref_in;


	return 0;
}

/**
* @brief      Marches the control system forward with new error inputs.
*
* Must be initialized. Error input path goes though PID control system.
* Assumes no feedforward path. Out is the compined output of the system.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::march(double* out, double err_in)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in march: not initialized\n");
		return -1;
	}
	*out = rc_filter_march(&gain_pd, gain_K * err_in)
		+ rc_filter_march(&gain_i, gain_K * err_in);


	return 0;
}

/**
* @brief      Scales the control system gains using the input provided.
*
* Must be initialized. Input must be between 0 and 1.
* Scales the gain based on default gain * the input.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::scale_gains(double scale)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in scale_gains: not initialized\n");
		return -1;
	}
	if (unlikely(scale < 0.0 || scale > 1.0))
	{
		printf("ERROR in scale_gains: input must be between 0.0 and 1.0\n");
		return -1;
	}

	gain_pd.gain = def_gain_pd.gain * scale;
	gain_i.gain = def_gain_i.gain * scale;
	gain_FF = def_gain_FF * scale;
	return 0;
}

/**
* @brief      Prefills PD control system input.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::prefill_pd_input(double in)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in prefill_pd_input: not initialized\n");
		return -1;
	}

	if (unlikely(rc_filter_prefill_inputs(&gain_pd, in)))
	{
		printf("ERROR: in prefill_pd_input: failed to prefill PD input\n");
		return -1;
	}

	return 0;
}

/**
* @brief      Prefills I control system input.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::prefill_i_input(double in)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in prefill_i_input: not initialized\n");
		return -1;
	}

	if (unlikely(rc_filter_prefill_inputs(&gain_i, in)))
	{
		printf("ERROR: in prefill_i_input: failed to prefill I input\n");
		return -1;
	}

	return 0;
}

/**
* @brief      Prefills PD control system output.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::prefill_pd_out(double out)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in prefill_pd_out: not initialized\n");
		return -1;
	}

	if (unlikely(rc_filter_prefill_inputs(&gain_pd, out)))
	{
		printf("ERROR: in prefill_pd_out: failed to prefill PD output\n");
		return -1;
	}

	return 0;
}

/**
* @brief      Prefills I control system output.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::prefill_i_out(double out)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in prefill_i_out: not initialized\n");
		return -1;
	}

	if (unlikely(rc_filter_prefill_inputs(&gain_i, out)))
	{
		printf("ERROR: in prefill_i_out: failed to prefill I output\n");
		return -1;
	}

	return 0;
}

/**
* @brief      Changes the active gains based on new input.
*
* @return     0 on success, -1 on failure
*/
int feedback_filter_gen_t::set_tune_gains(PID_vars_set_t& new_input)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in set_tune_gains: not initialized\n");
		return -1;
	}

	gain_i.num.d[1] = new_input.GainN1_i;
	gain_pd.num.d[0] = new_input.GainN0_pd;
	gain_pd.num.d[1] = new_input.GainN1_pd;
	gain_pd.den.d[1] = new_input.GainD1_pd;
	gain_FF = new_input.GainFF;
	gain_K = new_input.GainK;
	return 0;
}



/**
 * @ brief     parses a json_object and sets up a new controller
 *
 * @param      jobj         The jobj to parse
 * @param      filter       pointer to write the new filter to
 *
 * @return     0 on success, -1 on failure
 */
static int __parse_rc_filter_PID(json_object* jobj_ctl, rc_filter_t* filter)
{
	struct json_object* array = NULL;  // to hold num & den arrays
	struct json_object* tmp = NULL;    // temp object
	char* tmp_str = NULL;
	double tmp_flt, tmp_kp, tmp_ki, tmp_kd, tmp_imax;
	int i, num_len, den_len;
	rc_vector_t num_vec = RC_VECTOR_INITIALIZER;
	rc_vector_t den_vec = RC_VECTOR_INITIALIZER;

	// destroy old memory in case the order changes
	rc_filter_free(filter);

	// check if PID gains or transfer function coefficients
	if (json_object_object_get_ex(jobj_ctl, "TF_or_PID", &tmp) == 0)
	{
		fprintf(stderr, "ERROR: can't find TF_or_PID in settings file\n");
		return -1;
	}
	if (json_object_is_type(tmp, json_type_string) == 0)
	{
		fprintf(stderr, "ERROR: TF_or_PID should be a string\n");
		return -1;
	}
	tmp_str = (char*)json_object_get_string(tmp);

	if (strcmp(tmp_str, "TF") == 0)
	{
		// pull out gain
		if (parse_double(jobj_ctl, "gain", tmp_flt) < 0)
		{
			fprintf(stderr, "ERROR in __parse_rc_filter_PID: failed to parse gain\n");
			return -1;
		}

		// pull out numerator
		if (json_object_object_get_ex(jobj_ctl, "numerator", &array) == 0)
		{
			fprintf(stderr, "ERROR: can't find controller numerator in settings file\n");
			return -1;
		}
		if (json_object_is_type(array, json_type_array) == 0)
		{
			fprintf(stderr, "ERROR: controller numerator should be an array\n");
			return -1;
		}
		num_len = json_object_array_length(array);
		if (num_len < 1)
		{
			fprintf(stderr, "ERROR, numerator must have at least 1 entry\n");
			return -1;
		}
		rc_vector_alloc(&num_vec, num_len);
		for (i = 0; i < num_len; i++)
		{
			tmp = json_object_array_get_idx(array, i);
			if (json_object_is_type(tmp, json_type_double) == 0)
			{
				fprintf(stderr, "ERROR: numerator array entries should be a doubles\n");
				return -1;
			}
			tmp_flt = json_object_get_double(tmp);
			num_vec.d[i] = tmp_flt;
		}

		// pull out denominator
		if (json_object_object_get_ex(jobj_ctl, "denominator", &array) == 0)
		{
			fprintf(stderr, "ERROR: can't find controller denominator in settings file\n");
			return -1;
		}
		if (json_object_is_type(array, json_type_array) == 0)
		{
			fprintf(stderr, "ERROR: controller denominator should be an array\n");
			return -1;
		}
		den_len = json_object_array_length(array);
		if (den_len < 1)
		{
			fprintf(stderr, "ERROR, denominator must have at least 1 entry\n");
			return -1;
		}
		rc_vector_alloc(&den_vec, den_len);
		for (i = 0; i < den_len; i++)
		{
			tmp = json_object_array_get_idx(array, i);
			if (json_object_is_type(tmp, json_type_double) == 0)
			{
				fprintf(stderr, "ERROR: denominator array entries should be a doubles\n");
				return -1;
			}
			tmp_flt = json_object_get_double(tmp);
			den_vec.d[i] = tmp_flt;
		}

		// check for improper TF
		if (num_len > den_len)
		{
			fprintf(stderr, "ERROR: improper transfer function\n");
			rc_vector_free(&num_vec);
			rc_vector_free(&den_vec);
			return -1;
		}

		// check CT continuous time or DT discrete time
		if (json_object_object_get_ex(jobj_ctl, "CT_or_DT", &tmp) == 0)
		{
			fprintf(stderr, "ERROR: can't find CT_or_DT in settings file\n");
			return -1;
		}
		if (json_object_is_type(tmp, json_type_string) == 0)
		{
			fprintf(stderr, "ERROR: CT_or_DT should be a string\n");
			return -1;
		}
		tmp_str = (char*)json_object_get_string(tmp);

		// if CT, use tustin's approx to get to DT
		if (strcmp(tmp_str, "CT") == 0)
		{
			// get the crossover frequency
			if (parse_double(jobj_ctl, "crossover_freq_rad_per_sec", tmp_flt) < 0)
			{
				fprintf(stderr, "ERROR in __parse_rc_filter_PID: failed to parse crossover frequency\n");
				return -1;
			}
			if (rc_filter_c2d_tustin(filter, DT, num_vec, den_vec, tmp_flt))
			{
				fprintf(stderr, "ERROR: failed to c2dtustin while parsing json\n");
				return -1;
			}
		}

		// if DT, much easier, just construct filter
		else if (strcmp(tmp_str, "DT") == 0)
		{
			if (rc_filter_alloc(filter, num_vec, den_vec, DT))
			{
				fprintf(stderr, "ERROR in __parse_rc_filter_PID: failed to alloc filter\n");
				return -1;
			}
		}

		// wrong value for CT_or_DT
		else
		{
			fprintf(stderr, "ERROR: CT_or_DT must be 'CT' or 'DT'\n");
			printf("instead got :%s\n", tmp_str);
			return -1;
		}
	}

	else if (strcmp(tmp_str, "P") == 0)
	{
		// pull out gains
		if (parse_double(jobj_ctl, "kp", tmp_kp) < 0)
		{
			fprintf(stderr, "ERROR in __parse_rc_filter_PID: failed to parse kp\n");
			return -1;
		}
		tmp_ki = 0.0;
		tmp_kd = 0.0;

		// Not used in rc_filter_pid for pure P, but (1/tmp_flt) must be >2*DT
		tmp_flt = 62.83;

		if (rc_filter_pid(filter, tmp_kp, tmp_ki, tmp_kd, 1.0 / tmp_flt, DT))
		{
			fprintf(stderr, "ERROR in __parse_rc_filter_PID: failed to alloc pid filter");
			return -1;
		}
	}

	else if (strcmp(tmp_str, "PD") == 0)
	{
		// pull out gains
		if (parse_double(jobj_ctl, "kp", tmp_kp) < 0)
		{
			fprintf(stderr, "ERROR in __parse_rc_filter_PID: failed to parse kp\n");
			return -1;
		}
		tmp_ki = 0.0;
		if (parse_double(jobj_ctl, "kd", tmp_kd) < 0)
		{
			fprintf(stderr, "ERROR in __parse_rc_filter_PID: failed to parse kd\n");
			return -1;
		}
		// get the crossover frequency
		if (parse_double(jobj_ctl, "crossover_freq_rad_per_sec", tmp_flt) < 0)
		{
			fprintf(stderr, "ERROR in __parse_rc_filter_PID: failed to parse crossover frequency\n");
			return -1;
		}
		if (rc_filter_pid(filter, tmp_kp, tmp_ki, tmp_kd, 1.0 / tmp_flt, DT))
		{
			fprintf(stderr, "ERROR in __parse_rc_filter_PID: failed to alloc pid filter\n");
			return -1;
		}
	}

	else if (strcmp(tmp_str, "I") == 0)
	{
		if (parse_double(jobj_ctl, "ki", tmp_ki) < 0)
		{
			fprintf(stderr, "ERROR in __parse_rc_filter_PID: failed to parse ki\n");
			return -1;
		}
		if (parse_double(jobj_ctl, "imax", tmp_imax) < 0)
		{
			fprintf(stderr, "ERROR in __parse_rc_filter_PID: failed to parse imax\n");
			return -1;
		}

		if (rc_filter_integrator(filter, DT))
		{
			fprintf(stderr, "ERROR in __parse_rc_filter_PID: failed to alloc i filter\n");
			return -1;
		}

		rc_filter_enable_saturation(filter, -tmp_imax, tmp_imax);
		filter->gain = tmp_ki;
	}

#ifdef DEBUG
	rc_filter_print(*filter);
#endif

	rc_vector_free(&num_vec);
	rc_vector_free(&den_vec);

	return 0;
}


static char __parse_controller_PID(json_object* in_json, const char* name, rc_filter_t* filter)
{
	struct json_object* tmp_main_json = NULL;    // temp object

	//find and parse entry
	if (json_object_object_get_ex(in_json, name, &tmp_main_json) == 0)
	{
		fprintf(stderr, "ERROR in __parse_controller_PID: can't find %s entry\n", name);
		return -1;
	}
	if (json_object_is_type(tmp_main_json, json_type_object) == 0)
	{
		fprintf(stderr, "ERROR in __parse_controller_PID: %s must be an object\n", name);
		return -1;
	}
	if (__parse_rc_filter_PID(tmp_main_json, filter))
	{

		fprintf(stderr, "ERROR in __parse_controller_PID: could not parse filter\n");
		return -1;
	}
	return 0;
}

char parse_feedback_filter_gen(json_object* in_json, const char* name, controller_settings_t& controller_settings)
{
	struct json_object* tmp_main_json = NULL;    // temp object

	//find and parse entry
	if (json_object_object_get_ex(in_json, name, &tmp_main_json) == 0)
	{
		fprintf(stderr, "ERROR in parse_feedback_filter_gen: can't find %s entry\n", name);
		return -1;
	}
	if (json_object_is_type(tmp_main_json, json_type_object) == 0)
	{
		fprintf(stderr, "ERROR in parse_feedback_filter_gen: %s must be an object\n", name);
		return -1;
	}
	if (__parse_controller_PID(tmp_main_json, "pd", &controller_settings.pd))
	{
		fprintf(stderr, "ERROR in parse_feedback_filter_gen: could not parse PD controller\n");
		return -1;
	}
	if (__parse_controller_PID(tmp_main_json, "i", &controller_settings.i))
	{
		fprintf(stderr, "ERROR in parse_feedback_filter_gen: could not parse I controller\n");
		return -1;
	}
	if (parse_double_positive(tmp_main_json, "FF", controller_settings.FF))
	{
		fprintf(stderr, "ERROR in parse_feedback_filter_gen: could not parse FF gain\n");
		return -1;
	}
	if (parse_double_positive(tmp_main_json, "K", controller_settings.K))
	{
		fprintf(stderr, "ERROR in parse_feedback_filter_gen: could not parse K gain\n");
		return -1;
	}

#ifdef DEBUG
	printf("Finished parsing %s PID controller with the following settings:\n", name);
	printf("PD transfer function:\n");
	rc_filter_print(controller_settings.pd);
	printf("I transfer function:\n");
	rc_filter_print(controller_settings.i);
	printf("FF = %f\n", controller_settings.FF);
	printf("K = %f\n\n", controller_settings.K);
#endif // DEBUG

	return 0;
}