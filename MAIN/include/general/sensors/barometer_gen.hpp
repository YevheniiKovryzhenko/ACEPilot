/*
 * barometer_gen.hpp
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

#ifndef BAROMETER_GEN_HPP
#define BAROMETER_GEN_HPP

#include <stdio.h>

 /* General class for all barometer instances */
class barometer_gen_t
{
private:
	bool initialized = false;
	double pressure_raw = 0.0; // (Pa)
	double altitude_raw = 0.0; // (m)
	double temperature_raw = 0.0; // (c)

	bool first_run = true;
	double initial_alt = 0.0;
	double alt_ground = 0.0; // (m) altitude from the initialization point
public:
	char init(void);
	bool is_initialized(void);

	char march(double new_pr, double new_alt, double new_temp);

	double get_alt(void);
	double get_alt_ground(void);
	double get_pr(void);
	double get_temp(void);

	char reset(void);
	void cleanup(void);
};
extern barometer_gen_t bmp;


/** @name Logging class for barometer
* Defines how logging should be done for this class
*/
class barometer_log_entry_t
{
private:
	double pressure_raw; // (Pa)
	double altitude_raw; // (m)
	double temperature_raw; // (c)

	double alt_ground; // (m) altitude from the initialization point

	char print_vec(FILE* file, double* vec_in, int size);
	char print_header_vec(FILE* file, const char* prefix, const char* var_name, int size);
public:
	char update(barometer_gen_t& new_state);
	char print_header(FILE* file, const char* prefix);
	char print_entry(FILE* file);
};

#endif // !BAROMETER_GEN_HPP
