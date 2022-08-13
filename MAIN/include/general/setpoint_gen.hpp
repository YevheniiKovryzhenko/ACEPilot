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
 * Last Edit:  08/12/2022 (MM/DD/YYYY)
 *
 * Summary :
 * General-purpose class for handling clustered signals. 
 * Used for simplifying setpoint_manager. Defines multiple overloads for wider compatibility. 
 *
 */

#ifndef SETPOINT_GEN_HPP
#define SETPOINT_GEN_HPP

#include "signal_filter_gen.hpp"

/* generalized attachement node */
class attachement_gen_t
{
private:
	bool use_pt = false;

	double source = 0.0;
	double* source_pt = &source;

	double target = 0.0;
	double* target_pt = &target;
public:
	int reset(void); // resets target and source to internal values
	int bridge(double* new_target, double* new_source); //sets source to target using pointer
	int bridge(double new_target, double new_source); //sets internal source to target
	int update(void);
};


/**
* Generalized setpoint class.
*/
class setpoint_gen_t
{
private:
	bool en = false; // enable flag
	double value = 0.0; // value
	double* pt_value = &value; // pointer to value

	double gain = 0.0; // signal gain
	double* gain_pt = &gain;// pointer to signal gain

	double def_value = 0.0; //set default value
	double* pt_def_value = &def_value; //pointer to default value for reset function

	bool filters_initialized = false;

	void reset_gain(void);

public:
	bool is_en(void) const; // returns en flag
	int disable(void); // sets en flag to true
	int enable(void); // sets en flag to false

	/* functions for retreving value parameter */
	double get(void); //updates internal functions if enabled; returns value;
	double* get_pt(void) const; // returns pt_value
	
	/* Functions for seting value parameter */
	int set(double val); // sets value; resets pointer to point at value
	int set(double* val); // sets value source;
	int set(double* val, double new_gain); // sets value source; sets constant gain;
	int set(double* val, double* new_gain); // sets value source; sets source for gain;
	int set(setpoint_gen_t& val); //sets value equal to val

	/* functions for setting default value */
	int set_def(double val); //sets def_value; resets pointer to point at def_value
	int set_def(double* val); //sets def_value sourse

	int saturate(double min, double max); //saturates value between min and max
	int increment(double gain); //performs value += gain	

	int reset(void); //resets value to def_value pointer
	int reset_all(void); //resets everything

	/* functions for manipulating data pipelines */
	attachement_gen_t connector{}; // is updated if enabled

	/* signal filters */
	signal_filter_triplet_gen_t filters{}; // is updated if enabled

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
	int set(double* in);
	int set(double* in, double new_gain);
	int set(double* in, double* new_gain);
	int set(setpoint_gen_1D_t& in);
	
	int set_FF(double in);
	int set_FF(double* in);	
	int set_FF(double* in, double new_gain);
	int set_FF(double* in, double* new_gain);
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
public:
	setpoint_gen_1D_t x{}; //setpoint for x/roll
	setpoint_gen_1D_t y{}; //setpoint for y/pitch


	bool is_en(void) const; // return en flag for both
	bool is_en_FF(void) const; // return en_FF flag for both
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
public:
	setpoint_gen_1D_t x{}; //setpoint for x/roll
	setpoint_gen_1D_t y{}; //setpoint for y/pitch
	setpoint_gen_1D_t z{}; //setpoint for z/yaw/heading

	bool is_en(void) const; // return en flag for all
	int disable(void); // sets en flag to true for all
	int enable(void); // sets en flag to false for all
	bool is_en_FF(void) const; // return en_FF flag for all
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