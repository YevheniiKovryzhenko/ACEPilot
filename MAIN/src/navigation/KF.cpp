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

#include "KF.hpp"
#include "settings.hpp"
#include "rc/math/quaternion.h"
#include "rc/math.h"
#include "tools.h"

#ifndef GET_VARIABLE_NAME
#define GET_VARIABLE_NAME(Variable) (#Variable)
#endif // !GET_VARIABLE_NAME

KF_t KF_altitude{};

/**
* @brief      initialize the altitude kalman filter
*
* @return     0 on success, -1 on failure
*/
char KF_t::init(void)
{
	if (initialized)
	{
		fprintf(stderr, "ERROR in init: KF is already initilized\n");
		return -1;
	}	

	//initialize altitude kalman filter and bmp sensor
	rc_matrix_t F = RC_MATRIX_INITIALIZER;
	rc_matrix_t G = RC_MATRIX_INITIALIZER;
	rc_matrix_t H = RC_MATRIX_INITIALIZER;
	rc_matrix_t Q = RC_MATRIX_INITIALIZER;
	rc_matrix_t R = RC_MATRIX_INITIALIZER;
	rc_matrix_t Pi = RC_MATRIX_INITIALIZER;

	const int Nx = 3;
	const int Ny = 1;
	const int Nu = 1;

	// allocate appropirate memory for system
	rc_matrix_zeros(&F, Nx, Nx);
	rc_matrix_zeros(&G, Nx, Nu);
	rc_matrix_zeros(&H, Ny, Nx);
	rc_matrix_zeros(&Q, Nx, Nx);
	rc_matrix_zeros(&R, Ny, Ny);
	rc_matrix_zeros(&Pi, Nx, Nx);

	// define system -DT; // accel bias
	F.d[0][0] = 1.0;
	F.d[0][1] = DT;
	F.d[0][2] = 0.0;
	F.d[1][0] = 0.0;
	F.d[1][1] = 1.0;
	F.d[1][2] = -DT; // subtract accel bias
	F.d[2][0] = 0.0;
	F.d[2][1] = 0.0;
	F.d[2][2] = 1.0; // accel bias state

	G.d[0][0] = 0.5 * DT * DT;
	G.d[0][1] = DT;
	G.d[0][2] = 0.0;

	H.d[0][0] = 1.0;
	H.d[0][1] = 0.0;
	H.d[0][2] = 0.0;

	// covariance matrices
	Q.d[0][0] = 0.000000001;
	Q.d[1][1] = 0.000000001;
	Q.d[2][2] = 0.0001; // don't want bias to change too quickly
	R.d[0][0] = 1000000.0;

	// initial P, cloned from converged P while running
	Pi.d[0][0] = 1258.69;
	Pi.d[0][1] = 158.6114;
	Pi.d[0][2] = -9.9937;
	Pi.d[1][0] = 158.6114;
	Pi.d[1][1] = 29.9870;
	Pi.d[1][2] = -2.5191;
	Pi.d[2][0] = -9.9937;
	Pi.d[2][1] = -2.5191;
	Pi.d[2][2] = 0.3174;

	// initialize the kalman filter
	if (rc_kalman_alloc_lin(&alt_kf, F, G, H, Q, R, Pi) < 0) return -1;
	rc_matrix_free(&F);
	rc_matrix_free(&G);
	rc_matrix_free(&H);
	rc_matrix_free(&Q);
	rc_matrix_free(&R);
	rc_matrix_free(&Pi);

	initialized = true;
	return 0;
}
bool KF_t::is_initialized(void)
{
	return initialized;
}

char KF_t::march(double Z_acc, double Z)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in march: KF is not initialized\n");
		return -1;
	}

	// do first-run filter setup
	if (alt_kf.step == 0) {
		rc_vector_zeros(&u, 1);
		rc_vector_zeros(&y, 1);
		alt_kf.x_est.d[0] = Z;
	}

	// calculate acceleration and smooth it just a tad
	// put result in u for kalman and flip sign since with altitude, positive
	// is up whereas acceleration in Z points down.
	u.d[0] = Z_acc;// acc_imu[2] + GRAVITY;
	y.d[0] = Z;

	rc_kalman_update_lin(&alt_kf, u, y);

	// altitude estimate
	alt_est = -alt_kf.x_est.d[0]; //Z
	alt_vel_est = -alt_kf.x_est.d[1]; //Z velocity
	alt_acc_est = -alt_kf.x_est.d[2]; //Z acceleration
	return 0;
}



double KF_t::get_alt(void)
{
    return alt_est;
}
double KF_t::get_alt_vel(void)
{
	return alt_vel_est;
}
double KF_t::get_alt_acc(void)
{
	return alt_acc_est;
}

void KF_t::cleanup(void)
{
	rc_kalman_free(&alt_kf);
	rc_vector_free(&u);
	rc_vector_free(&y);
	initialized = false;
	return;
}



/** @name Logging class for KF
* Defines how logging should be done for this class
*/
char KF_log_entry_t::update(KF_t* new_state)
{
	alt_est = new_state->get_alt();
	alt_vel_est = new_state->get_alt_vel();
	alt_acc_est = new_state->get_alt_acc();
    return 0;
}
char KF_log_entry_t::print_header(FILE* file, const char* prefix)
{
    fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(alt_est));
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(alt_vel_est));
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(alt_acc_est));
    return 0;
}
char KF_log_entry_t::print_entry(FILE* file)
{
	fprintf(file, ",%.4F", alt_est);
	fprintf(file, ",%.4F", alt_vel_est);
	fprintf(file, ",%.4F", alt_acc_est);
    return 0;
}