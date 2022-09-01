/*
 * EKF.hpp
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
 * This class defines an Extended Kalman Filter V-1 operations.
 * Currently estimates attitude from IMU.
 * 
 * this code is based on the following thread:
 *	https://github.com/beagleboard/librobotcontrol/issues/74#issuecomment-331214723
 * 
 */

#ifndef EKF_HPP
#define EKF_HPP
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include <rc/time.h>

#include "tools.h"

class EKF_t
{
private:
	uint64_t time = rc_nanos_since_boot(); //internally updates time
	
	bool initialized = false;
	bool enable_mag = false;
	double Pcov[16];
	
	double quat_raw[4];		// raw estimated quaternians
	double quat_NED[4];		// estimated quaternian in NED
	double att_tb_NED[3]; //roll pitch and yaw from estimated quaternians
	double continuous_heading_NED;
	int num_heading_spins = 0;

	double Cov_info[3] = { 0.00001,0.1,1.0 };

public:

	char march(double omega[3], double accel[3], double mag[3]);
	char reset(void);

	/* Data retreival */
	uint64_t get_time(void);
	void get_quat_raw(double* buff);
	void get_quat(double* buff);
	void get_tb(double* buff);
	double get_continuous_heading(void);
};

extern EKF_t EKF1;

/** @name Logging class for EKF
* Defines how logging should be done for this class
*/
class EKF_log_entry_t
{
private:
	uint64_t time;

	double quat_raw[4];		// raw estimated quaternians
	double quat_NED[4];		// estimated quaternian in NED
	double att_tb_NED[3]; //roll pitch and yaw from estimated quaternians
	double continuous_heading_NED;

	char print_vec(FILE* file, double* vec_in, int size);
	char print_header_vec(FILE* file, const char* prefix, const char* var_name, int size);
public:
	char update(EKF_t* new_state);
	char print_header(FILE* file, const char* prefix);
	char print_entry(FILE* file);
};
#endif
