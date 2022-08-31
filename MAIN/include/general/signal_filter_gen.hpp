/*
 * signal_filter_gen.hpp
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
 * Last Edit:  08/31/2022 (MM/DD/YYYY)
 *
 * Summary :
 * General-purpose class for applying simple filtering on a signal.
 *
 */

#ifndef SIGNAL_FILTER_GEN_HPP
#define SIGNAL_FILTER_GEN_HPP
#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include <rc/math.h>
#include <rc/math/filter.h>
#include "rc_pilot_defs.hpp"
#include "settings.hpp"

/**
* Setpoint filter class.
*/
class signal_filter_gen_t
{
private:
	rc_filter_t filter = RC_FILTER_INITIALIZER;
	bool initialized = false;
	bool en = false;
	double input = 0.0;
	double output = 0.0;
	bool last_en = false;

	double dt = DT;
	double tc = 10.0 * DT;

	bool  en_saturation = true;
	double min = -1.0;
	double max = 1.0;

public:
	bool is_init(void);
	int set_type(signal_filter_gen_type_t type);

	bool is_en(void);
	int enable(void);
	int disable(void);

	double get(void);

	int set_dt(double in);
	int set_tc(double in);
	int set_min_max(double new_min, double new_max);
	int set_saturation(bool in);

	int update(double in);

	int reset(void);
};

class signal_filter_triplet_gen_t
{
private:
	bool filters_initialized = false;
public:
	signal_filter_gen_t lowpass{}; //1-st order low-pass filter
	signal_filter_gen_t highpass{}; //1-st order high-pass filter
	signal_filter_gen_t integrator{}; //integrator filter

	int init_all(void);
	int update_all(double in_lp, double in_hp, double in_i);
	int stop_all(void);
	int reset_all(void);
};

/* General class for low-pass filter operation */
class signal_filter1D_gen_t
{
private:
	signal_filter_gen_settings_t settings;
	rc_filter_t filter = RC_FILTER_INITIALIZER;
public:
	char set(signal_filter_gen_settings_t new_settings);
	char prefill_inputs(double new_in);
	char prefill_outputs(double new_out);
	double march(double new_raw);
	char enable_saturation(double new_min, double new_max);
	char reset(void);
	void cleanup(void);
};

/* General class for low-pass 3D filter operation */
class signal_filter3D_gen_t
{
private:
	signal_filter1D_gen_t filter[3];
public:
	char set(signal_filter_gen_settings_t new_settings[3]);
	char prefill_inputs(double new_in[3]);
	char prefill_outputs(double new_out[3]);
	void march(double(&out)[3], double new_raw[3]);
	char reset(void);
	void cleanup(void);
};


#endif // !SIGNAL_FILTER_GEN_HPP
