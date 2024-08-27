/*
 * tools.h
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
 * Last Edit:  08/29/2022 (MM/DD/YYYY)
 */

#ifndef TOOLS_H
#define TOOLS_H

#include <rc/time.h> // for nanos
#include <inttypes.h> // for PRIu64
#include <math.h>
#include <stdio.h> //for fscanf

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

	/**
	* @brief		finds time elapsed in seconds
	*
	* @return		time in seconds
	*/
	double finddt_s(uint64_t ti);

	/**
	* @brief		kinamatics equations for computing
	*				euler angle rates from body rates and 
	*				attitude
	*
	* @return		assumes and modifies att_rates[3]
	*/
	void omega_att2att_rates(double* att_rates, double att[3], double omega[3]);

	/**
	* @brief		kinamatics equations for computing
	*				body rates from attitude rates and
	*				attitude
	*
	* @return		assumes and modifies omega[3]
	*/
	void att_rates_att2omega(double* omega, double att[3], double att_rates[3]);

	/**
	* @brief		uses a cubic pol. to connect two points in space using initial and final
	*				constraints
	*
	* @return     possition (m) based on time elapsed dt and total time tt_s (in seconds)
	*/
	double cubicPol(double xi, double xf, double xdoti, double xdotf, float tt_s, double dt);

	/**
	* @brief		scans data from a text file
	*
	* @return		0 on success, -1 on failure
	*/
	int scan_file_d(void);


	/**
	* @brief		this function multiplies two square 3x3 matrices
	*
	* @return		0 on success, -1 on failure
	*/
	//void multiply(double mat1[3][3], double mat2[3][3], double res[3][3])


	/**
	* @brief		applies a coordinate tranformation from body to inetrial frame
	*				using 1-2-3 psi-theta-phi rotation
	*
	* @return		0 on success, -1 on failure
	*/
	//int rotate_i2b(double** vec);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* TOOLS_H */

