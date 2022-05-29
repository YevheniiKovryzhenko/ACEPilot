/*
 * setpoint_gen.hpp
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


/**
* <setpoint_gen.h>
*
* @brief      
*/
#ifndef SETPOINT_GEN_HPP
#define SETPOINT_GEN_HPP
#include <rc/math/filter.h>
#include "rc_pilot_defs.h"

/**
* Setpoint filter types.
*/
enum setpoint_filter_type_t {
	Lowpass = 0,
	Highpass = 1,
	Integrator = 2
};

/**
* Setpoint filter class.
*/
class setpoint_filter_t
{
private:
	rc_filter_t filter = RC_FILTER_INITIALIZER;
	bool initialized = false;
	bool en = false;
	double input = 0.0;
	double output = 0.0;
	bool last_en = false;

	double dt = DT;
	double tc = 10.0*DT;
	
	bool  en_saturation = true;
	double min = -1.0;
	double max = 1.0;

public:
	bool is_init(void);
	int set_type(setpoint_filter_type_t type);

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


/**
* Generalized setpoint subclass.
*/
class setpoint_gen_t
{
private:
	bool en = false; // enable flag
	double value = 0.0; // value
	double* pt_value = &value; // pointer to value

	double def_value = 0.0; //set default value
	double* pt_def_value = &def_value; //pointer to default value for reset function

	bool filters_initialized = false;

public:
	bool is_en(void); // returns en flag
	int disable(void); // sets en flag to true
	int enable(void); // sets en flag to false

	double get(void); // returns value
	double* get_pt(void); // returns pt_value
	int set(double val); // sets value; resets pointer to point at value
	int set(double* val); // sets value source;
	int set(setpoint_gen_t& val); //sets value equal to val
	int saturate(double min, double max); //saturates value between min and max
	int increment(double gain); //performs value += gain

	int set_def(double val); //sets def_value; resets pointer to point at def_value
	int set_def(double* val); //sets def_value sourse

	int reset(void); //resets value to def_value pointer


	/* signal filters */
	setpoint_filter_t lp{};
	setpoint_filter_t hp{};
	setpoint_filter_t integrator{};

	int init_filters(void);
	int stop_filters(void);

};

/**
* Generalized setpoint subclass for single degree with feedforward.
*/
class setpoint_gen_1D_t
{
public:
	setpoint_gen_t value{}; // setpoint control
	setpoint_gen_t FF{}; // feedforward control

	int set(double in);
	int set_FF(double in);
	int set(double* in);
	int set_FF(double* in);
	int set(setpoint_gen_1D_t& in);
	int set_FF(setpoint_gen_1D_t& in);
	int set_all(setpoint_gen_1D_t& in);

	int reset(void);
	int reset_FF(void);
	int reset_all(void);

	int enable(void);
	int enable_FF(void);
	int enable_all(void);
	int disable(void);
	int disable_FF(void);
	int disable_all(void);

	int init_filters(void);
	int stop_filters(void);

};

/**
* Generalized setpoint subclass for controlling 2 degrees.
*/
class setpoint_gen_2D_t
{
private:
	bool en = false; // enable flag	
	bool en_FF = false; // enable FF flag	
public:
	setpoint_gen_1D_t x{}; //setpoint for x/roll
	setpoint_gen_1D_t y{}; //setpoint for y/pitch


	bool is_en(void); // return en flag for both
	bool is_en_FF(void); // return en_FF flag for both
	int disable(void); // sets en flag to true for both
	int enable(void); // sets en flag to false for both
	int disable_FF(void); // sets en_FF flag to true for both
	int enable_FF(void); // sets en_FF flag to false for both
	int enable_all(void);
	int disable_all(void);

	int set(double x_in, double y_in);
	int set(double* x_in, double* y_in);
	int set_FF(double x_in, double y_in);
	int set_FF(double* x_in, double* y_in);
	int set(setpoint_gen_2D_t& val);
	int set_FF(setpoint_gen_2D_t& val);
	int set_all(setpoint_gen_2D_t& val);

	int reset(void);
	int reset_FF(void);
	int reset_all(void);	

	int init_filters(void);
	int stop_filters(void);
};

/**
* Generalized setpoint subclass for controlling 3 degrees.
*/
class setpoint_gen_3D_t
{
private:
	bool en = false; // enable flag	
public:
	setpoint_gen_1D_t x{}; //setpoint for x/roll
	setpoint_gen_1D_t y{}; //setpoint for y/pitch
	setpoint_gen_1D_t z{}; //setpoint for z/yaw/heading

	bool is_en(void); // return en flag for all
	int disable(void); // sets en flag to true for all
	int enable(void); // sets en flag to false for all
	bool is_en_FF(void); // return en_FF flag for all
	int disable_FF(void); // sets en flag to true for all
	int enable_FF(void); // sets en flag to false for all
	int enable_all(void);
	int disable_all(void);

	int set(double x_in, double y_in, double z_in);
	int set(double* x_in, double* y_in, double* z_in);
	int set_FF(double x_in, double y_in, double z_in);
	int set_FF(double* x_in, double* y_in, double* z_in);
	int set(setpoint_gen_3D_t& input);
	int set_FF(setpoint_gen_3D_t& input);
	int set_all(setpoint_gen_3D_t& input);

	int reset(void);
	int reset_FF(void);
	int reset_all(void);

	int init_filters(void);
	int stop_filters(void);
};


#endif // SETPOINT_GEN_HPP