/*
 * barometer_gen.cpp
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
 * This contains the nessesary framework for operating Barometer.
 */

#include "barometer_gen.hpp"
#include <json.h>

// preposessor macros
#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif // !unlikely

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif // !likely

#ifndef GET_VARIABLE_NAME
#define GET_VARIABLE_NAME(Variable) (#Variable)
#endif // !GET_VARIABLE_NAME

barometer_gen_t bmp{};	//barometer

/* Methods for general BAROMETER class */
char barometer_gen_t::init(void)
{
	if (unlikely(initialized))
	{
		fprintf(stderr, "ERROR in init: barometer already initialized.\n");
		return -1;
	}

	initialized = true;
	first_run = true;
	return 0;
}

char barometer_gen_t::march(double new_pr, double new_alt, double new_temp)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in march: barometer is not initialized.\n");
		return -1;
	}
	if (first_run) initial_alt = new_alt;
	pressure_raw = new_pr;
	temperature_raw = new_temp;
	altitude_raw = new_alt;
	alt_ground = new_alt - initial_alt;

	first_run = false;
	return 0;
}

double barometer_gen_t::get_alt(void)
{
	return altitude_raw;
}
double barometer_gen_t::get_alt_ground(void)
{
	return alt_ground;
}
double barometer_gen_t::get_pr(void)
{
	return pressure_raw;
}
double barometer_gen_t::get_temp(void)
{
	return temperature_raw;
}

char barometer_gen_t::reset(void)
{
	first_run = true;
	return 0;
}

void barometer_gen_t::cleanup(void)
{
	if (!initialized) return;
	initialized = false;
	first_run = true;
	return;
}


 /** @name Logging class for barometer
 * Defines how logging should be done for this class
 */
char barometer_log_entry_t::update(barometer_gen_t& new_state)
{
	pressure_raw = new_state.get_pr();
	altitude_raw = new_state.get_alt();
	temperature_raw = new_state.get_temp();
	alt_ground = new_state.get_alt_ground();

	return 0;
}
char barometer_log_entry_t::print_vec(FILE* file, double* vec_in, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%.4F", vec_in[i]);
	}
	return 0;
}

char barometer_log_entry_t::print_header_vec(FILE* file, const char* prefix, const char* var_name, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%s%s_%i", prefix, var_name, i);
	}
	return 0;
}

char barometer_log_entry_t::print_header(FILE* file, const char* prefix)
{
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(pressure_raw));
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(altitude_raw));
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(temperature_raw));
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(alt_ground));
	return 0;
}
char barometer_log_entry_t::print_entry(FILE* file)
{
	fprintf(file, ",%.4F", pressure_raw);
	fprintf(file, ",%.4F", altitude_raw);
	fprintf(file, ",%.4F", temperature_raw);
	fprintf(file, ",%.4F", alt_ground);
	return 0;
}