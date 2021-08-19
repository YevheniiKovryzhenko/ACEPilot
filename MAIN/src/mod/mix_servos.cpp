/*
 * mix-servos.cpp
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
 * Last Edit:  08/18/2020 (MM/DD/YYYY)
 *
 * Summary :
 * Functions to mix orthogonal inputs to motor controls
 *
 * MultiRotors and other areal vehciles are controlled by mixing roll, pitch, yaw, and
 * throttle control oututs, a linear combination of which forms the
 * control output to each motor and/or servo. The coefficients to this
 * combination is stored in a mixing matrix based on rotor/servo layout.
 * Also included here are functions to parse configuration strings
 * and do the actual mixing.
 *
 */

#include "mix_servos.hpp"
 /*
 * Invoke defaut constructor for all the built in and exernal types in the class
 */
servo_mix_t servo_mix{};	// extern variable in mix_servos.hpp

/**
 * The simplest: 6-servo layout with servos
 * mapped directly to throttle sticks.
 * 
 * columns: X Y Z Roll Pitch Yaw
 * rows: motors 1-6
 */
static double mix_6x_direct_test[][MAX_SERVO_INPUTS] = { \
{0.0,   0.0,	-1.0,	1.0,	0.0,	0.0},\
{0.0,   0.0,	-1.0,	-1.0,	0.0,	0.0},\
{0.0,   0.0,	-1.0,	0.0,	1.0,	0.0},\
{0.0,	0.0,	-1.0,	0.0,	-1.0,	0.0}, \
{0.0,	0.0,	-1.0,	0.0,	0.0,	1.0}, \
{0.0,   0.0,	-1.0,	0.0,	0.0,	-1.0} };

/*
* Droneship Zeppelin 16 servo mixing matrix.
* top view:
*  1   2       cw ccw      X   Z down
*    X                     ^
*  3   4       ccw cw      + > Y
* columns:	X(2nd Yaw)	Y		Z		Roll	Pitch	Yaw		
*/
static double mix_16x_zeppelin[][MAX_SERVO_INPUTS] = { \
{0.0,	0.0,	-1.0,	1.0,	1.0,	-1.0},\
{0.0,	0.0,	-1.0,	1.0,	1.0,	-1.0},\
{0.0,	0.0,	-1.0,	-1.0,	1.0,	1.0},\
{0.0,	0.0,	-1.0,	-1.0,	1.0,	1.0},\
{0.0,	0.0,	-1.0,	1.0,	-1.0,	-1.0},\
{0.0,	0.0,	-1.0,	1.0,	-1.0,	-1.0},\
{0.0,	0.0,	-1.0,	-1.0,	-1.0,	1.0},\
{0.0,	0.0,	-1.0,	-1.0,	-1.0,	1.0},\
{1.0,	0.0,	0.0,	0.0,	0.0,	0.0},\
{1.0,	0.0,	0.0,	0.0,	0.0,	0.0},\
{1.0,	0.0,	0.0,	0.0,	0.0,	0.0},\
{1.0,	0.0,	0.0,	0.0,	0.0,	0.0},\
{-1.0,	0.0,	0.0,	0.0,	0.0,	0.0},\
{-1.0,	0.0,	0.0,	0.0,	0.0,	0.0},\
{-1.0,	0.0,	0.0,	0.0,	0.0,	0.0},\
{-1.0,	0.0,	0.0,	0.0,	0.0,	0.0}};

int servo_mix_t::init(servo_layout_t layout)
{
	if (initialized)
	{
		fprintf(stderr,"\nERROR in init, already intialized");
		return -1;
	}
	switch(layout){
	case LAYOUT_6xDIRECT_TEST:
		servos = 6;
		dof = 4;
		mix_matrix = mix_6x_direct_test;
		break;
	case LAYOUT_16xZEPPELIN:
		servos = 16;
		dof = 6;
		mix_matrix = mix_16x_zeppelin;
		break;
	default:
		fprintf(stderr,"ERROR in init unknown servo layout\n");
		return -1;
	}

	initialized = 1;
	return 0;
}


int servo_mix_t::all_controls(double u[MAX_SERVO_INPUTS], double* mot)
{
	int i, j;
	if (!initialized) 
	{
		fprintf(stderr, "ERROR in mix_all_controls, mixing matrix not set yet\n");
		return -1;
	}
	// sum control inputs
	for (i = 0; i < servos; i++) {
		mot[i] = 0.0;
		for (j = 0; j < MAX_SERVO_INPUTS; j++) {
			mot[i] += mix_matrix[i][j] * u[j];
		}
	}
	// ensure saturation, should not need to do this if mix_check_saturation
	// was used properly, but here for safety anyway.
	for (i = 0; i < servos; i++) {
		if (mot[i] > 1.0) mot[i] = 1.0;
		else if (mot[i] < 0.0) mot[i] = 0.0;
	}
	return 0;
}


int servo_mix_t::check_saturation(int ch, double* mot, double* min, double* max)
{
	int i, min_ch;
	double tmp;
	double new_max = DBL_MAX;
	double new_min = -DBL_MAX;

	if(!initialized){
		fprintf(stderr,"ERROR: in check_channel_saturation, mix matrix not set yet\n");
		return -1;
	}

	switch (dof)
	{
	case 4:
		min_ch = 2;
		break;
	case 6:
		min_ch = 0;
		break;
	default:
		fprintf(stderr, "ERROR: in check_saturation, dof should be 4 or 6, currently %d\n", dof);
		return -1;
	}

	if (ch < min_ch || ch >= 6)
	{
		fprintf(stderr, "ERROR: in check_channel_saturation, ch out of bounds\n");
		return -1;
	}

	// make sure motors are not already saturated
	for (i = 0; i < servos; i++)
	{
		if (mot[i] > 1.0 || mot[i] < 0.0) 
		{
			fprintf(stderr, "ERROR: motor channel already out of bounds\n");
			return -1;
		}
	}

	// find max positive input
	for (i = 0; i < servos; i++)
	{
		// if mix channel is 0, impossible to saturate
		if (mix_matrix[i][ch] == 0.0) continue;
		// for positive entry in mix matrix
		if (mix_matrix[i][ch] > 0.0)	tmp = (1.0 - mot[i]) / mix_matrix[i][ch];
		// for negative entry in mix matrix
		else tmp = -mot[i] / mix_matrix[i][ch];
		// set new upper limit if lower than current
		if (tmp < new_max) new_max = tmp;
	}

	// find min (most negative) input
	for (i = 0; i < servos; i++) {
		// if mix channel is 0, impossible to saturate
		if (mix_matrix[i][ch] == 0.0) continue;
		// for positive entry in mix matrix
		if (mix_matrix[i][ch] > 0.0)	tmp = -mot[i] / mix_matrix[i][ch];
		// for negative entry in mix matrix
		else tmp = (1.0 - mot[i]) / mix_matrix[i][ch];
		// set new upper limit if lower than current
		if (tmp > new_min) new_min = tmp;
	}

	*min = new_min;
	*max = new_max;
	return 0;
}


int servo_mix_t::add_input(double u, int ch, double* mot)
{
	int i;
	int min_ch;

	if (!initialized || dof == 0)
	{
		fprintf(stderr, "ERROR: in add_input, mix matrix not set yet\n");
		return -1;
	}
	switch (dof)
	{
	case 4:
		min_ch = 2;
		break;
	case 6:
		min_ch = 0;
		break;
	default:
		fprintf(stderr, "ERROR: in add_input, dof should be 4 or 6, currently %d\n", dof);
		return -1;
	}

	if (ch < min_ch || ch >= 6)
	{
		fprintf(stderr, "ERROR: in mix_add_input, ch out of bounds\n");
		return -1;
	}

	// add inputs
	for (i = 0; i < servos; i++)
	{
		mot[i] += u * mix_matrix[i][ch];
		// ensure saturation, should not need to do this if mix_check_saturation
		// was used properly, but here for safety anyway.
		if (mot[i] > 1.0) mot[i] = 1.0;
		else if (mot[i] < 0.0) mot[i] = 0.0;
	}
	return 0;
}
