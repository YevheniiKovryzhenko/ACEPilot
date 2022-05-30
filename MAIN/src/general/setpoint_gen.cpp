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
 * Last Edit:  05/30/2022 (MM/DD/YYYY)
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

 /* generalized attachement node */
int attachement_gen_t::reset(void)
{
	source = 0.0;
	target = 0.0;
	source_pt = &source;
	target_pt = &target;
	return 0;
}
int attachement_gen_t::bridge(double* new_target, double* new_source)
{
	use_pt = true;
	source_pt = new_source;
	target_pt = new_target;
	return 0;
}
int attachement_gen_t::bridge(double new_target, double new_source)
{
	use_pt = false;
	source = new_source;
	target = new_target;
	return 0;
}
int attachement_gen_t::update(void)
{
	
	if (use_pt) *target_pt = *source_pt;
	else target = source;
	
	return 0;
}



 /**
 * Generalized setpoint class.
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
	if (en)
	{
		if (unlikely(connector.update() < 0))
		{
			printf("ERROR in get: failed to update connector\n");
			en = false;
		}
	}
	return *pt_value;
}
double* setpoint_gen_t::get_pt(void)
{
	return pt_value;
}
void setpoint_gen_t::reset_gain(void)
{
	gain = 0.0;
	gain_pt = &gain;
}
int setpoint_gen_t::set(double val)
{
	value = val;
	pt_value = &value;
	
	reset_gain();
	return 0;
}
int setpoint_gen_t::set(double* val)
{
	value = *val;
	pt_value = val;

	reset_gain();
	return 0;
}
int setpoint_gen_t::set(double* val, double new_gain)
{
	value = *val;
	pt_value = val;

	reset_gain();
	gain = new_gain;
	return 0;
}
int setpoint_gen_t::set(double* val, double* new_gain)
{
	value = *val;
	pt_value = val;

	gain_pt = new_gain;
	gain = *gain_pt;
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
	reset_gain();
	return set(*pt_def_value);
}
int setpoint_gen_t::reset_all(void)
{
	bool tmp_1 = reset() < 0;
	bool tmp_2 = connector.reset() < 0;
	bool tmp_3 = filters.reset_all() < 0;
	if (tmp_1 || tmp_2 || tmp_3) return -1;
	return 0;
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
int setpoint_gen_1D_t::set(double* in, double new_gain)
{
	return value.set(in, new_gain);
}
int setpoint_gen_1D_t::set(double* in, double* new_gain)
{
	return value.set(in, new_gain);
}
int setpoint_gen_1D_t::set(setpoint_gen_1D_t& in)
{
	return value.set(in.value);
}
int setpoint_gen_1D_t::set_FF(double in)
{
	return FF.set(in);
}
int setpoint_gen_1D_t::set_FF(double* in)
{
	return FF.set(in);
}
int setpoint_gen_1D_t::set_FF(double* in, double new_gain)
{
	return FF.set(in, new_gain);
}
int setpoint_gen_1D_t::set_FF(double* in, double* new_gain)
{
	return FF.set(in, new_gain);
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
	if (value.reset() < 0)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_1D_t::reset_FF(void)
{
	if (FF.reset() < 0)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_1D_t::reset_all(void)
{
	bool tmp_1 = FF.reset_all() < 0;

	if (value.reset_all() < 0 || tmp_1)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_1D_t::enable(void)
{
	return value.enable();
}
int setpoint_gen_1D_t::enable_FF(void)
{
	if (FF.connector.bridge(FF.get_pt(), value.get_pt()) < 0) return -1;
	else FF.enable();
	return 0;
}
int setpoint_gen_1D_t::enable_all(void)
{
	bool tmp = enable() < 0;
	if (enable_FF() < 0 || tmp) return -1;
	return 0;
}
int setpoint_gen_1D_t::disable(void)
{
	if (unlikely(value.connector.reset() < 0))
	{
		printf("ERROR in disable: failed to reset connector\n");
	}
	return value.disable();
}
int setpoint_gen_1D_t::disable_FF(void)
{
	if (unlikely(FF.connector.reset() < 0))
	{
		printf("ERROR in disable_FF: failed to reset connector\n");
	}
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
	bool tmp_1 = value.filters.init_all() < 0;
	if (FF.filters.init_all() < 0 || tmp_1)
	{
		return -1;
	}
	return 0;
}
int setpoint_gen_1D_t::stop_filters(void)
{
	bool tmp_1 = value.filters.stop_all() < 0;
	if (FF.filters.stop_all() < 0 || tmp_1)
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
	bool tmp = x.value.is_en() && y.value.is_en();
	return tmp;
}
bool setpoint_gen_2D_t::is_en_FF(void)
{
	bool tmp = x.FF.is_en() && y.FF.is_en();
	return tmp;
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
	bool tmp_1 = x.value.is_en() && x.value.is_en() && z.value.is_en();
	return tmp_1;
}
bool setpoint_gen_3D_t::is_en_FF(void)
{
	bool tmp_1 = x.FF.is_en() && x.FF.is_en() && z.FF.is_en();
	return tmp_1;
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