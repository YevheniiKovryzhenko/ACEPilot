/*
 * setpoint_gen.cpp
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
 * Last Edit:  05/28/2022 (MM/DD/YYYY)
 *
 * Summary :
 * Subclass for simplifying setpoint_manager. Defines multiple overloads for wider compatibility.
 *
 */

#include <math.h>
#include <stdio.h>
#include <inttypes.h>

#include <rc/math.h>
#include <rc/time.h>
#include <rc/math/filter.h>
#include "setpoint_gen.hpp"

#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

 /**
 * Setpoint filter class.
 */
bool setpoint_filter_t::is_init(void)
{
	return initialized;
}
int setpoint_filter_t::set_type(setpoint_filter_type_t type)
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
bool setpoint_filter_t::is_en(void)
{
	return en;
}
int setpoint_filter_t::enable(void)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in enable: not initialized\n");
		return -1;
	}

	en = true;

	return 0;
}
int setpoint_filter_t::disable(void)
{
	en = false;
	last_en = false;

	return 0;
}
int setpoint_filter_t::reset(void)
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
double setpoint_filter_t::get(void)
{
	return output;
}
int setpoint_filter_t::set_dt(double in)
{
	dt = in;
	return 0;
}
int setpoint_filter_t::set_tc(double in)
{
	tc = in;
	return 0;
}
int setpoint_filter_t::set_min_max(double new_min, double new_max)
{
	min = new_min;
	max = new_max;
	return 0;
}
int setpoint_filter_t::set_saturation(bool in)
{
	en_saturation = in;

	return 0;
}
int setpoint_filter_t::update(double in)
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

 /**
 * Generalized setpoint subclass.
 */
bool setpoint_gen_t::is_en(void)
{
	return en;
}
int setpoint_gen_t::enable(void)
{
	en = true;
	return 0;
}
int setpoint_gen_t::disable(void)
{
	en = false;
	return 0;
}
double setpoint_gen_t::get(void)
{
	return *pt_value;
}
double* setpoint_gen_t::get_pt(void)
{
	return pt_value;
}
int setpoint_gen_t::set(double val)
{
	value = val;
	pt_value = &value;
	return 0;
}
int setpoint_gen_t::set(double* val)
{
	value = *val;
	pt_value = val;
	return 0;
}
int setpoint_gen_t::set(setpoint_gen_t& val)
{
	return set(val.get());
}
int setpoint_gen_t::set_def(double val)
{
	def_value = val;
	pt_def_value = &def_value;
	return 0;
}
int setpoint_gen_t::set_def(double* val)
{
	pt_def_value = val;
	return 0;
}
int setpoint_gen_t::reset(void)
{
	return set(*pt_def_value);
}
int setpoint_gen_t::saturate(double min, double max)
{
	return rc_saturate_double(pt_value, min, max);
}
int setpoint_gen_t::increment(double in)
{
	*pt_value += in;
	return 0;
}
int setpoint_gen_t::init_filters(void)
{
	if (unlikely(lp.set_type(Lowpass) < 0))
	{
		printf("ERROR in init_filters: failed to initialize low-pass filter\n");
		filters_initialized = false;
		return -1;
	}
	if (unlikely(hp.set_type(Highpass) < 0))
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
int setpoint_gen_t::stop_filters(void)
{
	lp.disable();
	lp.reset();
	hp.disable();
	hp.reset();
	integrator.disable();
	integrator.reset();
	return 0;
}

/**
 * Generalized setpoint subclass for single degree.
 */
int setpoint_gen_1D_t::set(double in)
{
	return value.set(in);
}
int setpoint_gen_1D_t::set(double* in)
{
	return value.set(in);
}
int setpoint_gen_1D_t::set_FF(double in)
{
	return FF.set(in);
}
int setpoint_gen_1D_t::set_FF(double* in)
{
	return FF.set(in);
}
int setpoint_gen_1D_t::set(setpoint_gen_1D_t& in)
{
	return value.set(in.value);
}
int setpoint_gen_1D_t::set_FF(setpoint_gen_1D_t& in)
{
	return FF.set(in.FF);
}
int setpoint_gen_1D_t::set_all(setpoint_gen_1D_t& in)
{
	bool tmp = set_FF(in) < 0;
	if (set(in) < 0 || tmp)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_1D_t::reset(void)
{
	return value.reset();
}
int setpoint_gen_1D_t::reset_FF(void)
{
	return FF.reset();
}
int setpoint_gen_1D_t::reset_all(void)
{
	bool tmp = FF.reset() < 0;
	if (value.reset() < 0 || tmp)
	{
		return -1;
	}
	return FF.reset();
}
int setpoint_gen_1D_t::enable(void)
{
	return value.enable();
}
int setpoint_gen_1D_t::enable_FF(void)
{
	return FF.enable();
}
int setpoint_gen_1D_t::enable_all(void)
{
	bool tmp = enable() < 0;
	if (enable_FF() < 0 || tmp) return -1;
	return 0;
}
int setpoint_gen_1D_t::disable(void)
{
	return value.disable();
}
int setpoint_gen_1D_t::disable_FF(void)
{
	return FF.disable();
}
int setpoint_gen_1D_t::disable_all(void)
{
	bool tmp = disable() < 0;
	if (disable_FF() < 0 || tmp) return -1;
	return 0;
}
int setpoint_gen_1D_t::init_filters(void)
{
	bool tmp_1 = value.init_filters() < 0;
	if (FF.init_filters() < 0 || tmp_1)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_1D_t::stop_filters(void)
{
	bool tmp_1 = value.stop_filters() < 0;
	if (FF.stop_filters() < 0 || tmp_1)
	{
		return -1;
	}
	return 0;
}

/**
* Generalized setpoint subclass for controlling 2 degrees.
*/
bool setpoint_gen_2D_t::is_en(void)
{
	en = x.value.is_en() && y.value.is_en();
	return en;
}
bool setpoint_gen_2D_t::is_en_FF(void)
{
	en_FF = x.FF.is_en() && y.FF.is_en();
	return en;
}
int setpoint_gen_2D_t::set(double x_in, double y_in)
{
	bool tmp = y.set(y_in) < 0;
	if (x.set(x_in) < 0 || tmp)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_2D_t::set(double* x_in, double* y_in)
{
	bool tmp = y.set(y_in) < 0;
	if (x.set(x_in) < 0 || tmp)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_2D_t::set_FF(double x_in, double y_in)
{
	bool tmp = y.set_FF(y_in) < 0;
	if (x.set_FF(x_in) < 0 || tmp)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_2D_t::set_FF(double* x_in, double* y_in)
{
	bool tmp = y.set_FF(y_in) < 0;
	if (x.set_FF(x_in) < 0 || tmp)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_2D_t::set(setpoint_gen_2D_t& val)
{
	bool tmp = y.set(val.y) < 0;
	if (x.set(val.x) < 0 || tmp)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_2D_t::set_FF(setpoint_gen_2D_t& val)
{
	bool tmp = y.set_FF(val.y) < 0;
	if (x.set_FF(val.x) < 0 || tmp)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_2D_t::set_all(setpoint_gen_2D_t& val)
{
	bool tmp = y.set_all(val.y) < 0;
	if (x.set_all(val.x) < 0 || tmp)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_2D_t::reset(void)
{
	bool tmp = y.reset() < 0;
	if (x.reset() < 0 || tmp)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_2D_t::reset_FF(void)
{
	bool tmp = y.reset_FF() < 0;
	if (x.reset_FF() < 0 || tmp)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_2D_t::reset_all(void)
{
	bool tmp = y.reset_all() < 0;
	if (x.reset_all() < 0 || tmp)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_2D_t::enable(void)
{
	bool tmp = x.enable() < 0;
	if (y.enable() < 0 || tmp) return -1;
	return 0;
}
int setpoint_gen_2D_t::enable_FF(void)
{
	bool tmp = x.enable_FF() < 0;
	if (y.enable_FF() < 0 || tmp) return -1;
	return 0;
}
int setpoint_gen_2D_t::enable_all(void)
{
	bool tmp = enable() < 0;
	if (enable_FF() < 0 || tmp) return -1;
	return 0;
}
int setpoint_gen_2D_t::disable(void)
{
	bool tmp = x.disable() < 0;
	if (y.disable() < 0 || tmp) return -1;
	return 0;
}
int setpoint_gen_2D_t::disable_FF(void)
{
	bool tmp = x.disable_FF() < 0;
	if (y.disable_FF() < 0 || tmp) return -1;
	return 0;
}
int setpoint_gen_2D_t::disable_all(void)
{
	bool tmp = disable() < 0;
	if (disable_FF() < 0 || tmp) return -1;
	return 0;
}
int setpoint_gen_2D_t::init_filters(void)
{
	bool tmp = x.init_filters() < 0;
	if (y.init_filters() < 0 || tmp) return -1;
	return 0;
}
int setpoint_gen_2D_t::stop_filters(void)
{
	bool tmp = x.stop_filters() < 0;
	if (y.stop_filters() < 0 || tmp) return -1;
	return 0;
}


/**
* Generalized setpoint subclass for controlling 3 degrees.
*/
bool setpoint_gen_3D_t::is_en(void)
{
	bool tmp_1 = z.value.is_en() < 0;
	bool tmp_2 = y.value.is_en() < 0;
	en = x.value.is_en() && tmp_1 && tmp_2;
	return en;
}
bool setpoint_gen_3D_t::is_en_FF(void)
{
	bool tmp_1 = z.FF.is_en();
	bool tmp_2 = y.FF.is_en();
	en = x.FF.is_en() && tmp_1 && tmp_2;
	return en;
}
int setpoint_gen_3D_t::set(double x_in, double y_in, double z_in)
{
	bool tmp_1 = z.value.set(y_in) < 0;
	bool tmp_2 = y.value.set(z_in) < 0;
	if (x.value.set(x_in) < 0 || tmp_1 || tmp_2)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_3D_t::set_FF(double x_in, double y_in, double z_in)
{
	bool tmp_1 = z.FF.set(y_in) < 0;
	bool tmp_2 = y.FF.set(z_in) < 0;
	if (x.FF.set(x_in) < 0 || tmp_1 || tmp_2)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_3D_t::set(double* x_in, double* y_in, double* z_in)
{
	bool tmp_1 = z.value.set(y_in) < 0;
	bool tmp_2 = y.value.set(z_in) < 0;
	if (x.value.set(x_in) < 0 || tmp_1 || tmp_2)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_3D_t::set_FF(double* x_in, double* y_in, double* z_in)
{
	bool tmp_1 = z.FF.set(y_in) < 0;
	bool tmp_2 = y.FF.set(z_in) < 0;
	if (x.FF.set(x_in) < 0 || tmp_1 || tmp_2)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_3D_t::set(setpoint_gen_3D_t& val)
{
	bool tmp_1 = z.set(val.y) < 0;
	bool tmp_2 = y.set(val.z) < 0;
	if (x.set(val.x) < 0 || tmp_1 || tmp_2)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_3D_t::set_FF(setpoint_gen_3D_t& val)
{
	bool tmp_1 = z.set_FF(val.y) < 0;
	bool tmp_2 = y.set_FF(val.z) < 0;
	if (x.set_FF(val.x) < 0 || tmp_1 || tmp_2)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_3D_t::set_all(setpoint_gen_3D_t& val)
{
	if (x.set_all(val.x) < 0 || y.set_all(val.y) < 0 || z.set_all(val.z) < 0)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_3D_t::reset(void)
{
	bool tmp_1 = z.reset() < 0;
	bool tmp_2 = y.reset() < 0;
	if (x.reset() < 0 || tmp_1 || tmp_2)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_3D_t::reset_FF(void)
{
	bool tmp_1 = z.reset_FF() < 0;
	bool tmp_2 = y.reset_FF() < 0;
	if (x.reset_FF() < 0 || tmp_1 || tmp_2)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_3D_t::reset_all(void)
{
	bool tmp_1 = reset_FF() < 0;
	if (reset() < 0 || tmp_1)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_3D_t::enable(void)
{
	bool tmp_1 = x.enable() < 0;
	bool tmp_2 = y.enable() < 0;
	if (z.enable() < 0 || tmp_1 || tmp_2) return -1;
	return 0;
}
int setpoint_gen_3D_t::enable_FF(void)
{
	bool tmp_1 = x.enable_FF() < 0;
	bool tmp_2 = y.enable_FF() < 0;
	if (z.enable_FF() < 0 || tmp_1 || tmp_2) return -1;
	return 0;
}
int setpoint_gen_3D_t::enable_all(void)
{
	bool tmp_1 = enable() < 0;
	bool tmp_2 = enable_FF() < 0;
	if (tmp_1 || tmp_2) return -1;
	return 0;
}
int setpoint_gen_3D_t::disable_all(void)
{
	bool tmp_1 = disable() < 0;
	bool tmp_2 = disable_FF() < 0;
	if (tmp_1 || tmp_2) return -1;
	return 0;
}
int setpoint_gen_3D_t::disable(void)
{
	bool tmp_1 = x.disable() < 0;
	bool tmp_2 = y.disable() < 0;
	if (z.disable() || tmp_1 || tmp_2) return -1;
	return 0;
}
int setpoint_gen_3D_t::disable_FF(void)
{
	bool tmp_1 = x.disable_FF() < 0;
	bool tmp_2 = y.disable_FF() < 0;
	if (z.disable_FF() || tmp_1 || tmp_2) return -1;
	return 0;
}
int setpoint_gen_3D_t::init_filters(void)
{
	bool tmp_1 = x.init_filters() < 0;
	bool tmp_2 = y.init_filters() < 0;
	if (z.init_filters() || tmp_1 || tmp_2) return -1;
	return 0;
}
int setpoint_gen_3D_t::stop_filters(void)
{
	bool tmp_1 = x.stop_filters() < 0;
	bool tmp_2 = y.stop_filters() < 0;
	if (z.stop_filters() || tmp_1 || tmp_2) return -1;
	return 0;
}