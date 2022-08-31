/*
 * KF.cpp
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
 * This class defines a Kalman Filter operations.
 * Currently estimates altitude from IMU and barometer.
 */

#ifndef KF_HPP
#define KF_HPP
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <rc/math/kalman.h>

#include <rc/time.h>

#include "tools.h"

class KF_t
{
private:	
	bool initialized = false;

	// altitude filter components
	rc_kalman_t alt_kf = RC_KALMAN_INITIALIZER;
	rc_vector_t u = RC_VECTOR_INITIALIZER;
	rc_vector_t y = RC_VECTOR_INITIALIZER;
	double alt_est = 0.0;
	double alt_vel_est = 0.0;
	double alt_acc_est = 0.0;

public:
	char init(void);
	bool is_initialized(void);

	char march(double Z_acc, double Z);
	void cleanup(void);

	/* Data retreival */
	double get_alt(void);
	double get_alt_vel(void);
	double get_alt_acc(void);
};

extern KF_t KF_altitude;

/** @name Logging class for KF
* Defines how logging should be done for this class
*/
class KF_log_entry_t
{
private:
	double alt_est;
	double alt_vel_est;
	double alt_acc_est;

public:
	char update(KF_t* new_state);
	char print_header(FILE* file, const char* prefix);
	char print_entry(FILE* file);
};
#endif
