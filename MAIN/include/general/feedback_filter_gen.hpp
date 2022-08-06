/*
 * feedback_filter_gen.hpp
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
 * Last Edit:  08/06/2022 (MM/DD/YYYY)
 */


#ifndef FILTER_GEN_HPP
#define FILTER_GEN_HPP
#include <rc/math.h>
#include <rc/time.h>
#include <rc/math/filter.h>
#include "settings.h"

typedef struct PID_vars_set_t
{
	uint8_t GainCH; //< tunning channel 
	float GainN1_i; //< N0 is always 0; D0 is always 1; D1 is always -1
	float GainN0_pd;
	float GainN1_pd;
	float GainD1_pd; //< D0 is always  1
	float GainFF;
	float GainK;
} PID_vars_set_t;

/* This is a class for a general purpouse SISO filter for the control system */
class feedback_filter_gen_t
{
private:
	bool initialized = false;
	rc_filter_t gain_pd = RC_FILTER_INITIALIZER; // active PD gain set
	rc_filter_t gain_i = RC_FILTER_INITIALIZER; // active I gain set
	rc_filter_t def_gain_pd = RC_FILTER_INITIALIZER; // default PD gain set
	rc_filter_t def_gain_i = RC_FILTER_INITIALIZER; // default I gain set
	double gain_FF = 0.0;
	double gain_K = 1.0;
	double def_gain_FF = 0.0;
public:

	/**
	* @brief      Initializes the control system.
	*
	* @return     0 on success, -1 on failure
	*/
	int init(controller_settings_t& new_ctrl);

	/**
	* @brief      Sets the active PD and I gains for the control system.
	*
	* @return     0 on success, -1 on failure
	*/
	int set_gain_set(controller_settings_t& new_ctrl);

	/**
	* @brief      Sets the default PD and I gains for the control system.
	*
	* @return     0 on success, -1 on failure
	*/
	int set_default_gain_set(controller_settings_t& new_ctrl);

	/**
	* @brief      Resets the control system to default gains, zeros out inputs/outputs, etc.
	*
	* @return     0 on success, -1 on failure
	*/
	int reset(void);

	/**
	* @brief      Marches the control system forward with new error and referece inputs.
	*
	* Must be initialized. Error (ref_in - st_in) input path goes though PI control system.
	* Reference path is added directly to the output of the PID control system.
	* Damping (derivative) is applied only on negative state (-st_in).
	* Out is the compined output of the system.
	*
	* @return     0 on success, -1 on failure
	*/
	int march_std(double* out, double ref_in, double st_in);

	/**
	* @brief      Marches the control system forward with new error and referece inputs.
	*
	* Must be initialized. Error input path goes though PID control system. Reference path is added
	* directly to the output of the PID control system. Out is the compined output of the system.
	*
	* @return     0 on success, -1 on failure
	*/
	int march(double* out, double err_in, double ref_in);

	/**
	* @brief      Marches the control system forward with new error inputs.
	*
	* Must be initialized. Error input path goes though PID control system.
	* Assumes no feedforward path. Out is the compined output of the system.
	*
	* @return     0 on success, -1 on failure
	*/
	int march(double* out, double err_in);

	/**
	* @brief      Scales the control system gains using the input provided.
	*
	* Must be initialized. Input must be between 0 and 1.
	* Scales the gain based on default gain * the input.
	*
	* @return     0 on success, -1 on failure
	*/
	int scale_gains(double scale);

	/**
	* @brief      Prefills PD control system input.
	*
	* @return     0 on success, -1 on failure
	*/
	int prefill_pd_input(double in);

	/**
	* @brief      Prefills I control system input.
	*
	* @return     0 on success, -1 on failure
	*/
	int prefill_i_input(double in);

	/**
	* @brief      Prefills PD control system output.
	*
	* @return     0 on success, -1 on failure
	*/
	int prefill_pd_out(double out);

	/**
	* @brief      Prefills I control system output.
	*
	* @return     0 on success, -1 on failure
	*/
	int prefill_i_out(double out);

	/**
	* @brief      Changes the active gains based on new input.
	*
	* @return     0 on success, -1 on failure
	*/
	int set_tune_gains(PID_vars_set_t& new_input);
};


#endif // !FILTER_GEN_HPP
