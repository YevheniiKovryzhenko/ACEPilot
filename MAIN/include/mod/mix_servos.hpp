/*
 * mix_servos.hpp
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
 * Last Edit:  07/29/2020 (MM/DD/YYYY)
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

#ifndef MIX_SERVOS_HPP
#define MIX_SERVOS_HPP

#include <stdio.h>
#include <stdlib.h>
#include <float.h> // for DBL_MAX
#include <stdbool.h>

#define MAX_SERVO_INPUTS 6		///< up to 6 control inputs (roll,pitch,yaw,z,x,y)
#define MAX_SERVOS 16			///< up to 16 servos

/**
 * @brief enum for possible mixing matrices defined here
 *
 * possible rotor configurations, see mixing_matrix_defs.h
 */
#include "servo_mix_defs.h" //include this untill settings.h is converted into C++ 
/*
typedef enum servo_layout_t {
	LAYOUT_4xDIRECT_TEST
} servo_layout_t;
*/

class servo_mix_t 
{
private:
	bool initialized;
	servo_layout_t servo_layout;
	double (*mix_matrix)[MAX_SERVO_INPUTS];
	int servos;
	int dof;
public:
	/**
 * @brief      Initiallizes the mixing matrix for a given input layout.
 *
 *             For a given number of rotors, layout character ('X','+') and
 *             degrees of freedom in control input (4 or 6), this selects the
 *             correct predefined mixing matrix from mixing_matrix_defs.h. The
 *             matrix is kept locally in mixing _matrix.c to prevent accidental
 *             misuse or modification. Use the other function from
 *             mixing_matrix.c below to interface with it. Used in
 *             mixing_matrix.c
 *
 * @param[in]  layout  The layout enum
 *
 * @return     0 on success, -1 on failure
 */
	int init(servo_layout_t layout);

	/**
	 * @brief      Fills the vector mot with the linear combination of XYZ, roll
	 *             pitch yaw. Not actually used, only for testing.
	 *
	 *             Fills the vector mot with the linear combination of roll pitch
	 *             yaw and throttle based on mixing matrix. If dof = 6, X and Y are
	 *             also added. Outputs are blindly saturated between 0 and 1. This
	 *             is for rudimentary mixing and testing only. It is recommended to
	 *             check for saturation for each input with check_channel_saturation
	 *             then add inputs sequentially with add_mixed_input() instead. Used
	 *             in mixing_matrix.c
	 *
	 * @param      u     6 control inputs
	 * @param      mot   pointer to motor outputs
	 *
	 * @return     0 on success, -1 on failure
	 */
	int all_controls(double u[MAX_SERVO_INPUTS], double* mot);

	/**
	 * @brief      Finds the min and max inputs u that can be applied to a current
	 *             set of motor outputs before saturating any one motor.
	 *
	 *             This is a precurser check to be done before marching a feedback
	 *             controller forward so we know what to saturate the transfer
	 *             function output at. Outputs min and max are given as pointers
	 *             that are written back to. The mot motor array argument is the
	 *             array of current motor outputs that the new channel ch will be
	 *             adding onto.
	 *
	 * @param[in]  ch    channel
	 * @param[in]      mot   The array of motor signals to be added onto
	 * @param[out]      min   The minimum possible input without saturation
	 * @param[out]      max   The maximum possible input without saturation
	 *
	 * @return     0 on success, -1 on failure
	 */
	int check_saturation(int ch, double* mot, double* min, double* max);

	/**
	 * @brief      Mixes the control input u for a single channel ch to the existing
	 *             motor array mot.
	 *
	 *             Mixes the control input u for a single channel ch corresponding
	 *             to throttle roll pitch etc to the existing motor array mot. No
	 *             saturation is done, the input u should be checked for saturation
	 *             validity with check_channel_saturation() first. Used in
	 *             mixing_matrix.c
	 *
	 * @param[in]  u     control input
	 * @param[in]  ch    channel
	 * @param      mot   array of motor channels
	 *
	 * @return     0 on success, -1 on failure
	 */
	int add_input(double u, int ch, double* mot);
};

extern servo_mix_t servo_mix;


#endif // MIX_SERVOS_HPP