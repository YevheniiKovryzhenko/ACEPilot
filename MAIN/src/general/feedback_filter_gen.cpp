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
 * Last Edit:  08/16/2022 (MM/DD/YYYY)
 */
#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include <rc/math.h>
#include <rc/time.h>
#include <rc/math/filter.h>
#include "feedback_filter_gen.hpp"

#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

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