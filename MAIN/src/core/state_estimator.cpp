/*
 * state_estimator.cpp
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
 *
 * Summary :
 * This contains all the primary functionality and framework for state estimation rountines
 *
 * This runs at the same rate as the feedback controller.
 * state_estimator_march() is called immediately before  feedback_march() in the
 * IMU interrupt service routine.
 */

#include <stdint.h> // for uint64_t
#include <stdio.h>
#include <math.h>

#include <rc/mpu.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/quaternion.h>
#include <rc/math/matrix.h>
#include <rc/math/other.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/bmp.h>

#include "rc_pilot_defs.h"
#include "settings.h"
#include "tools.h"
 //#include "input_manager.h"	
#include "comms_tmp_data_packet.h"
#include "gps.h"
#include "benchmark.h" //for velocity estimation

#include "state_estimator.hpp"
//#include <rc/math/filter.h>
//#include <rc/math/kalman.h>

#define TWO_PI (M_PI*2.0)

rc_mpu_data_t mpu_data; // extern variable in state_estimator.hpp
state_estimate_t state_estimate{}; // extern variable in state_estimator.hpp
ext_mag_t ext_mag;

/*
static void __mocap_march(void)
{
	state_estimate.get_X_vel() = rc_filter_march(&mocap_dx_lpf, state_estimate.X_dot_raw);
	state_estimate.get_Y_vel() = rc_filter_march(&mocap_dy_lpf, state_estimate.Y_dot_raw);
	state_estimate.get_Z_vel() = rc_filter_march(&mocap_dz_lpf, state_estimate.Z_dot_raw);

	if (settings.use_mocap_roll_rate)
	{
		state_estimate.get_roll_dot() = rc_filter_march(&mocap_r_lpf, state_estimate.roll_dot_raw);
	}
	if (settings.use_mocap_pitch_rate)
	{
		state_estimate.get_pitch_dot() = rc_filter_march(&mocap_p_lpf, state_estimate.pitch_dot_raw);
	}
	if (settings.use_mocap_pitch_rate)
	{
		state_estimate.get_yaw_dot() = rc_filter_march(&mocap_y_lpf, state_estimate.yaw_dot_raw);
	}
	return;
}


static void __z_init(void)
{
    rc_filter_first_order_lowpass(&z_lpf, DT, 300*DT);
    return;
}

static void __z_march(void)
{
    // state_estimate.Z_ddot = rc_filter_march(&z_lpf, state_estimate.accel_ground_frame[2]);
    state_estimate.Z_ddot = 0;
    return;
}

static void __z_cleanup(void)
{
    rc_filter_free(&z_lpf);
    return;
}

static void __imu_march(void)
{
	static double last_yaw = 0.0;
	static int num_yaw_spins = 0;
	double diff;

	static double last_yaw_mocap = 0.0;
    static int num_yaw_spins_mocap = 0;
    double diff_mocap;

	static double last_mocap_x = 0.0;
	static double last_mocap_y = 0.0;
	static double last_mocap_z = 0.0;
	double last_mocap_dt = 0.0;

	// gyro and accel require converting to NED coordinates
	state_estimate.gyro[0] =  mpu_data.gyro[1] * DEG_TO_RAD; //East
	state_estimate.gyro[1] =  mpu_data.gyro[0] * DEG_TO_RAD; //North
	state_estimate.gyro[2] = -mpu_data.gyro[2] * DEG_TO_RAD; //Up

	// quaternion also needs coordinate transform
	state_estimate.quat_imu[0] =  mpu_data.dmp_quat[0]; // W
	state_estimate.quat_imu[1] =  mpu_data.dmp_quat[2]; // X (i)
	state_estimate.quat_imu[2] =  mpu_data.dmp_quat[1]; // Y (j)
	state_estimate.quat_imu[3] = -mpu_data.dmp_quat[3]; // Z (k)
	if (settings.enable_mocap) {
		state_estimate.quat_mocap[0] = GS_RX.qw; // W
		state_estimate.quat_mocap[1] = GS_RX.qx; // X (i)
		state_estimate.quat_mocap[2] = GS_RX.qy; // Y (j)
		state_estimate.quat_mocap[3] = GS_RX.qz; // Z (k)
		
		// normalize quaternion because we don't trust the mocap system
		rc_normalize_quaternion_array(state_estimate.quat_mocap);
		// calculate tait bryan angles too
		double tmp[3];
		tmp[0] = state_estimate.tb_mocap[0];
		tmp[1] = state_estimate.tb_mocap[1];
		tmp[2] = state_estimate.tb_mocap[2];

		rc_quaternion_to_tb_array(state_estimate.quat_mocap, state_estimate.tb_mocap);
		state_estimate.tb_mocap[1] = -state_estimate.tb_mocap[1];
		state_estimate.tb_mocap[2] = -state_estimate.tb_mocap[2];

		// we need to estimate velocity from mocap position
		last_mocap_dt = finddt_s(benchmark_timers.tNAV); //get time elapsed since last itter.

		state_estimate.roll_dot_raw = (state_estimate.tb_mocap[0] - tmp[0]) / last_mocap_dt;
		state_estimate.pitch_dot_raw = (state_estimate.tb_mocap[1] - tmp[1]) / last_mocap_dt;
		state_estimate.yaw_dot_raw = (state_estimate.tb_mocap[2] - tmp[2]) / last_mocap_dt;
		
		last_mocap_x = state_estimate.pos_mocap[0];
		last_mocap_y = state_estimate.pos_mocap[1];
		last_mocap_z = state_estimate.pos_mocap[2];
		// position in the inertial frame
		state_estimate.pos_mocap[0]=(double)GS_RX.x;
		state_estimate.pos_mocap[1]=-(double)GS_RX.y;
		state_estimate.pos_mocap[2]=-(double)GS_RX.z;
		
		
		if (last_mocap_dt <= 0) //undefined division by zero, or if time is negative assume velocity did not change
		{
			printf("WARNING in __imu_march: undefined time step of %d\n", last_mocap_dt);
		}
		else
		{
			state_estimate.X_dot_raw = (state_estimate.pos_mocap[0] - last_mocap_x)\
				/ last_mocap_dt;
			state_estimate.Y_dot_raw = (state_estimate.pos_mocap[1] - last_mocap_y)\
				/ last_mocap_dt;
			state_estimate.Z_dot_raw = (state_estimate.pos_mocap[2] - last_mocap_z)\
				/ last_mocap_dt;
		}

		// yaw is more annoying since we have to detect spins
        // also make sign negative since NED coordinates has Z point down
        diff_mocap = -state_estimate.tb_mocap[2] + (num_yaw_spins_mocap * TWO_PI) - last_yaw_mocap;
        // detect the crossover point at +-PI and update num yaw spins
        if (diff_mocap < -M_PI) num_yaw_spins_mocap++;
        else if (diff_mocap > M_PI) num_yaw_spins_mocap--;

        // finally the new value can be written
        state_estimate.mocap_continuous_yaw = -state_estimate.tb_mocap[2] + (num_yaw_spins_mocap * TWO_PI);
        last_yaw_mocap = state_estimate.mocap_continuous_yaw;
	}

	// normalize it just in case
	rc_normalize_quaternion_array(state_estimate.quat_imu);
	// generate tait bryan angles
	rc_quaternion_to_tb_array(state_estimate.quat_imu, state_estimate.tb_imu);

	// yaw is more annoying since we have to detect spins
	// also make sign negative since NED coordinates has Z point down
	diff = state_estimate.tb_imu[2] + (num_yaw_spins * TWO_PI) - last_yaw;
	//detect the crossover point at +-PI and update num yaw spins
	if(diff < -M_PI) num_yaw_spins++;
	else if(diff > M_PI) num_yaw_spins--;

	// finally the new value can be written
	state_estimate.imu_continuous_yaw = state_estimate.tb_imu[2] + (num_yaw_spins * TWO_PI);
	last_yaw = state_estimate.imu_continuous_yaw;
	
	state_estimate.accel[0] =  mpu_data.accel[1];
	state_estimate.accel[1] =  mpu_data.accel[0];
	state_estimate.accel[2] = -mpu_data.accel[2];
	
	accel_in.d[0] = state_estimate.accel[0];
    accel_in.d[1] = state_estimate.accel[1];
    accel_in.d[2] = state_estimate.accel[2];

    // rotate accel vector
    rc_quaternion_rotate_vector_array(accel_in.d, state_estimate.quat_imu);

    state_estimate.accel_ground_frame[0] = accel_in.d[0];
    state_estimate.accel_ground_frame[1] = accel_in.d[1];
    state_estimate.accel_ground_frame[2] = accel_in.d[2] + GRAVITY;
	return;
}

static void __mag_march(void)
{
	static double last_yaw = 0.0;
	static int num_yaw_spins = 0;

	// don't do anything if mag isn't enabled
	if(!settings.enable_magnetometer) return;

	// mag require converting to NED coordinates
	state_estimate.mag[0] =  mpu_data.mag[1]; // East
	state_estimate.mag[1] =  mpu_data.mag[0]; // North
	state_estimate.mag[2] = -mpu_data.mag[2]; // Up

	// quaternion also needs coordinate transform
	state_estimate.quat_mag[0] =  mpu_data.fused_quat[0]; // W
	state_estimate.quat_mag[1] =  mpu_data.fused_quat[2]; // X (i)
	state_estimate.quat_mag[2] =  mpu_data.fused_quat[1]; // Y (j)
	state_estimate.quat_mag[3] = -mpu_data.fused_quat[3]; // Z (k)
	
	
	// normalize it just in case
	rc_normalize_quaternion_array(state_estimate.quat_mag);
	// generate tait bryan angles
	rc_quaternion_to_tb_array(state_estimate.quat_mag, state_estimate.tb_mag);

	// heading
	//state_estimate.mag_heading_raw = mpu_data.compass_heading_raw; //this is never used for some reason
	//state_estimate.mag_heading = state_estimate.tb_mag[2]; //this is never used for some reason

	// yaw is more annoying since we have to detect spins
	// also make sign negative since NED coordinates has Z point down
	double diff = state_estimate.tb_mag[2] + (num_yaw_spins * TWO_PI) - last_yaw;
	//detect the crossover point at +-PI and update num yaw spins
	if(diff < -M_PI) num_yaw_spins++;
	else if(diff > M_PI) num_yaw_spins--;

	// finally the new value can be written
	state_estimate.mag_heading_continuous = state_estimate.tb_mag[2] + (num_yaw_spins * TWO_PI);
	last_yaw = state_estimate.mag_heading_continuous;
	return;
}
*/

/**
 * @brief      initialize the altitude kalman filter
 *
 * @return     0 on success, -1 on failure
 */
char state_estimate_t::init_altitude_kf(void)
{
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

	return 0;
}

void state_estimate_t::march_altitude_kf(double* Z_est, double Z_acc, double Z)
{
	//int i;
	//double accel_vec[3];
	
	//static double global_update;

	// grab raw data
	//state_estimate.bmp_pressure_raw = bmp.pressure_pa;
	//state_estimate.alt_bmp_raw = bmp_data.alt_m;
	//state_estimate.bmp_temp = bmp_data.temp_c;

	// Set Global update variable
	//global_update = gps_data.lla.alt;


	// make copy of acceleration reading before rotating
	//for (i = 0; i < 3; i++) accel_vec[i] = state_estimate.accel[i];

	// rotate accel vector
	//rc_quaternion_rotate_vector_array(acc_imu, att_quat);

	// do first-run filter setup
	if (alt_kf.step == 0) {
		rc_vector_zeros(&u, 1);
		rc_vector_zeros(&y, 1);
		alt_kf.x_est.d[0] = Z;
		//rc_filter_prefill_inputs(&acc_lp, accel_vec[2] + GRAVITY);
		//rc_filter_prefill_outputs(&acc_lp, accel_vec[2] + GRAVITY);
	}

	// calculate acceleration and smooth it just a tad
	// put result in u for kalman and flip sign since with altitude, positive
	// is up whereas acceleration in Z points down.
	u.d[0] = Z_acc;// acc_imu[2] + GRAVITY;
	y.d[0] = Z;
	// don't bother filtering Barometer, kalman will deal with that
	/*
	if (settings.enable_gps)
	{
		// Use gps for kalman update
		y.d[0] = -global_update;
	}
	else
	{
		y.d[0] = -bmp_data.alt_m;
	}
	*/
	rc_kalman_update_lin(&alt_kf, u, y);

	// altitude estimate
	Z_est[0] = alt_kf.x_est.d[0]; //Z
	Z_est[1] = alt_kf.x_est.d[1]; //Z velocity
	Z_est[2] = alt_kf.x_est.d[2]; //Z acceleration
	return;
}


/**
* @brief       Updates and Marches all the sourses
*
* Runns update routines for all the sensors and external sourses
* of data for state estimation. This does not actually update
* state estimator itself, but makes sure new data is available.
*
* @param[in]   void
*
* @return      0 on success, -1 on failure
*/
char state_estimate_t::update_all_sourses(void)
{
	if (unlikely(!initialized)) {
		fprintf(stderr, "ERROR in update_all_sourses, estimator not initialized\n");
		return -1;
	}
	/* Populate sensor data objecs and march their subrouties */
	//BATT:
	if (batt.march(rc_adc_dc_jack()) < 0)
	{
		fprintf(stderr, "ERROR in update_all_sourses: failed to march battery\n");
		return -1;
	}

	//IMU:
	double tmp[3];
	for (int i = 0; i < 3; i++) tmp[i] = mpu_data.gyro[i] * DEG_TO_RAD; //convert to radians
	if (unlikely(imu.gyro.update(tmp) < 0)) //populate gyro data struct
	{
		fprintf(stderr, "ERROR in update_all_sourses: failed to update gyro data\n");
		return -1;
	}
	if (unlikely(imu.accel.update(mpu_data.accel) < 0)) //populate accel data struct
	{
		fprintf(stderr, "ERROR in update_all_sourses: failed to update accel data\n");
		return -1;
	}
	if (unlikely(imu.update_att_from_quat_dmp(mpu_data.dmp_quat) < 0)) //populate dmp data struct
	{
		fprintf(stderr, "ERROR in update_all_sourses: failed to fuse gyro and accel data\n");
		return -1;
	}
	if (settings.enable_magnetometer) // don't update mag if mag isn't enabled
	{
		if (unlikely(imu.compass.update(mpu_data.accel) < 0)) //populate mag data struct
		{
			fprintf(stderr, "ERROR in update_all_sourses: failed to update compass data\n");
			return -1;
		}

		if (unlikely(imu.update_att_from_quat_fused(mpu_data.fused_quat) < 0)) //populate fused data struct
		{
			fprintf(stderr, "ERROR in update_all_sourses: failed to fuse gyro, accel and compass data\n");
			return -1;
		}
	}
	if (unlikely(imu.march() < 0)) //march all gyro accel and mag filters
	{
		fprintf(stderr, "ERROR in update_all_sourses: failed to march IMU\n");
		return -1;
	}

	//MOCAP
	if (settings.enable_mocap)
	{
		double tmp[4];
		tmp[0] = GS_RX.qw;
		tmp[1] = GS_RX.qx;
		tmp[2] = GS_RX.qy;
		tmp[3] = GS_RX.qz;
		if (unlikely(mocap.update_att_from_quat(tmp) < 0))
		{
			fprintf(stderr, "ERROR in march: failed to update mocap attitude\n");
			return -1;
		}
		tmp[0] = GS_RX.x;
		tmp[1] = GS_RX.y;
		tmp[2] = GS_RX.z;

		if (unlikely(mocap.update_pos_vel_from_pos(tmp) < 0))
		{
			fprintf(stderr, "ERROR in march: failed to update mocap position\n");
			return -1;
		}

		if (unlikely(mocap.march() < 0))
		{
			fprintf(stderr, "ERROR in march: failed to march mocap\n");
			return -1;
		}

		mocap_check_timeout();
	}
	return 0;
}


/**
* @brief       Fetches data from IMU and internal sensors
*
* Updates estimator from IMU and on-board sensors,
* also selects proper sourses.
* Since this is the first update, it assumed to be the least accurate,
* so the subsequent ones might overwrite these.
* We want to update as many things as possible, in case we don't
* have any other sourses.
*
* @param[in]   void
*
* @return      void
*/
void state_estimate_t::fetch_internal_sourses(void)
{
	//Read encoders
	/*
	if (settings.enable_encoders) {
		for (int i = 1; i < 5; i++)
		{
			state_estimate.rev[i - 1] = rc_encoder_read(i);
		}
	}
	*/
	v_batt = batt.get();


	/* Attitude Angles and Rates + Body Rates */
	if (settings.enable_magnetometer) //use fused data from Gyro, Accel and Mag
	{
		imu.get_quat_fused(quat);
		imu.get_tb_fused(att);
		continuous_yaw = imu.get_continuous_yaw_fused();
	}
	else //use fused data from Gyro and Accel
	{
		imu.get_quat_dmp(quat);
		imu.get_tb_dmp(att);
		continuous_yaw = imu.get_continuous_yaw_dmp();
	}
	imu.gyro.get(omega);
	// need to also calculate omega -> att_rates
	omega_att2att_rates(att_rates, att, omega); //uses kinematics

	/* Acceleration */
	imu.accel.get(accel_relative);
	for (int i = 0; i < 3; i++) accel_global[i] = accel_relative[i];
	rc_quaternion_rotate_vector_array(accel_global, quat);
	accel_global[2] = accel_global[2] + GRAVITY;

	/* Vertical parameters */
	alt = bmp.get_alt_ground();
	//vel = need to update velocity
	alt_acc = -accel_global[2];

	/* Velocity */
	//need solution for this

	/* Position */
	//need solution for this as well
	
}


/**
* @brief       Fetches data from MOCAP, GPS and other external sources
*
* Updates estimator from external sourses such as GPS and mocap,
* also selects proper sourses.
*
* @param[in]   void
*
* @return      void
*/
void state_estimate_t::fetch_external_sourses(void)
{

	/* GPS */
	if (settings.enable_gps)
	{
		//Update data from GPS, assume mocap is not available 
	}

	/* MOCAP */
	if (settings.enable_mocap) //assumed to be the most accurate sourse, so directly update position and attitude
	{
		mocap.get_pos(pos_global);
		mocap.get_vel(vel_global);
		for (int i = 0; i < 3; i++)
		{
			pos_relative[i] = pos_global[i];
			vel_relative[i] = vel_global[i];
		}
		rc_quaternion_rotate_vector_array(pos_relative, quat);
		rc_quaternion_rotate_vector_array(vel_relative, quat);
		
		//also assume mocap gives perfect altitude from the ground:
		alt = -pos_global[2];
		alt_vel = -vel_global[2];

		/* Attitude Angles and Rates + Body Rates */
		if (settings.use_mocap_roll || settings.use_mocap_pitch || settings.use_mocap_yaw)
		{
			/* quaternians */
			mocap.get_quat(quat); //not sure it would be wise to select certain components, so ignore settings...			

			/* get roll, pitch and yaw as well as attitude rates and body rates */
			double tmp_angles[3], tmp_rates[3], tmp_omega[3];
			mocap.get_tb(tmp_angles);
			mocap.get_tb_rate(tmp_rates);
			// need to also calculate att_rates -> omega
			att_rates_att2omega(tmp_omega, tmp_angles, tmp_rates); //uses kinematics

			/* only use those if enabled by settings */
			if (settings.use_mocap_roll) {
				att[0] = tmp_angles[0];
				att_rates[0] = tmp_rates[0];
				omega[0] = tmp_omega[0];
			}
			if (settings.use_mocap_pitch) {
				att[1] = tmp_angles[1];
				att_rates[1] = tmp_rates[1];
				omega[1] = tmp_omega[1];
			}
			if (settings.use_mocap_yaw) {
				att[2] = tmp_angles[2];
				continuous_yaw = mocap.get_continuous_heading();
				att_rates[2] = tmp_rates[2];
				omega[2] = tmp_omega[2];
			}

			/* Update gloval acceleration using more accurate attitude */
			for (int i = 0; i < 3; i++) accel_global[i] = accel_relative[i];
			rc_quaternion_rotate_vector_array(accel_global, quat);
			accel_global[2] = accel_global[2] + GRAVITY;
		}		
	}	
}



void state_estimate_t::update_internal_filters(void)
{
	/* Update Altitude KF */
	
	double tmp[3];
	march_altitude_kf(tmp, accel_global[2], pos_global[2]); //using the best available data
	alt = -tmp[0];
	alt_vel = -tmp[1];
	alt_acc = -tmp[2];

	/* Update Attitude EKF */
	imu.compass.get(tmp);
	EKF.march(omega, accel_relative, tmp);
	EKF2.march(omega, accel_relative, tmp);
}

void state_estimate_t::cleanup_altitude_kf(void)
{
	rc_kalman_free(&alt_kf);
	rc_vector_free(&u);
	rc_vector_free(&y);
	return;
}



void state_estimate_t::mocap_check_timeout(void)
{
	if (mocap_running) {
		uint64_t current_time = rc_nanos_since_boot();
		// check if mocap data is > 3 steps old
		if ((current_time - mocap_timestamp_ns) > (3 * 1E7)) {
			mocap_running = false;
			if (settings.warnings_en) {
				fprintf(stderr, "WARNING, MOCAP LOST VISUAL\n");
			}
		}
	}
	return;
}


int state_estimate_t::init(void)
{
	if (initialized) {
		fprintf(stderr, "ERROR in init: estimator is already initialized\n");
		return -1;
	}

	if (unlikely(batt.init() < 0)) //need settings
	{
		fprintf(stderr, "ERROR in init: failed to initialize battery\n");
		return -1;
	}
	if (unlikely(bmp.init() < 0)) //need settings
	{
		fprintf(stderr, "ERROR in init: failed to initialize barometer\n");
		return -1;
	}
	else
	{
		if (unlikely(march_jobs_after_feedback() < 0)) //need settings
		{
			fprintf(stderr, "ERROR in init: failed to do first barometer read\n");
			return -1;
		}
		
	}
	if (unlikely(imu.init() < 0)) //need settings
	{
		fprintf(stderr, "ERROR in init: failed to initialize IMU\n");
		return -1;
	}
	if (unlikely(mocap.init() < 0)) //need settings
	{
		fprintf(stderr, "ERROR in init: failed to initialize mocap\n");
		return -1;
	}


	if (unlikely(init_altitude_kf() < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize altitude Kalman Filter\n");
		return -1;
	}
    
	EKF.reset();
	EKF2.reset();

	initialized = true;
	return 0;
}

bool state_estimate_t::is_initialized(void)
{
	return initialized;
}




int state_estimate_t::march(void)
{
	if (unlikely(!initialized)) {
		fprintf(stderr, "ERROR in march, estimator not initialized\n");
		return -1;
	}
	/* Fetch data froma all the sensors */
	if (unlikely(update_all_sourses() < 0))
	{
		fprintf(stderr, "ERROR in march, failed to update data sourses\n");
		return -1;
	}
	
	fetch_internal_sourses();
	fetch_external_sourses();
	update_internal_filters();

	//Print warning if too slow
	if (settings.delay_warnings_en)
	{   //check the update frequency
		double tmp = 1.0 / finddt_s(time);
		if (tmp < 90.0) printf("WARNING in march: Low update frequency of state estimator: %6.2f (Hz)\n", tmp);
	}
	time = rc_nanos_since_boot();
	return 0;
}


int state_estimate_t::march_jobs_after_feedback(void)
{
	// check if we need to sample BMP this loop
	if (bmp_sample_counter >= BMP_RATE_DIV) {
		rc_bmp_data_t tmp_bmp_data;
		// perform the i2c reads to the sensor, on bad read just try later
		if (rc_bmp_read(&tmp_bmp_data) < 0) return -1;
		bmp_sample_counter = 0;

		if (bmp.march(tmp_bmp_data.pressure_pa, tmp_bmp_data.alt_m, tmp_bmp_data.temp_c) < 0)
		{
			fprintf(stderr, "ERROR in march: failed to march barometer\n");
			return -1;
		}
	}
	bmp_sample_counter++;
	return 0;
}


int state_estimate_t::cleanup(void)
{
	batt.cleanup();
	bmp.cleanup();
	imu.cleanup();
	mocap.cleanup();

	cleanup_altitude_kf();
	return 0;
}

uint64_t state_estimate_t::get_time(void)
{
	return time;
}

double state_estimate_t::get_roll(void)
{
	return att[0];
}
double state_estimate_t::get_pitch(void)
{
	return att[1];
}
double state_estimate_t::get_yaw(void)
{
	return att[2];
}
double state_estimate_t::get_continuous_heading(void)
{
	return continuous_yaw;
}
void state_estimate_t::get_att_tb(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = att[i];
	return;
}
void state_estimate_t::get_att_quat(double* buff)
{
	for (int i = 0; i < 4; i++) buff[i] = quat[i];
	return;
}

double state_estimate_t::get_roll_dot(void)
{
	return att_rates[0];
}
double state_estimate_t::get_pitch_dot(void)
{
	return att_rates[1];
}
double state_estimate_t::get_yaw_dot(void)
{
	return att_rates[2];
}
void state_estimate_t::get_att_rates(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = att_rates[i];
	return;
}


double state_estimate_t::get_p(void)
{
	return omega[0];
}
double state_estimate_t::get_q(void)
{
	return omega[1];
}
double state_estimate_t::get_r(void)
{
	return omega[2];
}
void state_estimate_t::get_omega(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = omega[i];
	return;
}

double state_estimate_t::get_X(void)
{
	return pos_global[0];
}
double state_estimate_t::get_Y(void)
{
	return pos_global[1];
}
double state_estimate_t::get_Z(void)
{
	return pos_global[2];
}
void state_estimate_t::get_pos_glob(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = pos_global[i];
	return;
}

double state_estimate_t::get_x(void)
{
	return pos_relative[0];
}
double state_estimate_t::get_y(void)
{
	return pos_relative[1];
}
double state_estimate_t::get_z(void)
{
	return pos_relative[2];
}
void state_estimate_t::get_pos_rel(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = pos_relative[i];
	return;
}

double state_estimate_t::get_X_vel (void)
{
	return vel_global[0];
}
double state_estimate_t::get_Y_vel(void)
{
	return vel_global[1];
}
double state_estimate_t::get_Z_vel(void)
{
	return vel_global[2];
}
void state_estimate_t::get_vel_glob(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = vel_global[i];
	return;
}

double state_estimate_t::get_x_vel(void)
{
	return vel_relative[0];
}
double state_estimate_t::get_y_vel(void)
{
	return vel_relative[1];
}
double state_estimate_t::get_z_vel(void)
{
	return vel_relative[2];
}
void state_estimate_t::get_vel_rel(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = vel_relative[i];
	return;
}

double state_estimate_t::get_X_acc(void)
{
	return accel_global[0];
}
double state_estimate_t::get_Y_acc(void)
{
	return accel_global[1];
}
double state_estimate_t::get_Z_acc(void)
{
	return accel_global[2];
}
void state_estimate_t::get_acc_glob(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = accel_global[i];
	return;
}

double state_estimate_t::get_x_acc(void)
{
	return accel_relative[0];
}
double state_estimate_t::get_y_acc(void)
{
	return accel_relative[1];
}
double state_estimate_t::get_z_acc(void)
{
	return accel_relative[2];
}
void state_estimate_t::get_acc_rel(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = accel_relative[i];
	return;
}

double state_estimate_t::get_alt(void)
{
	return alt;
}
double state_estimate_t::get_alt_vel(void)
{
	return alt_vel;
}
double state_estimate_t::get_alt_acc(void)
{
	return alt_acc;
}

double state_estimate_t::get_v_batt(void)
{
	return v_batt;
}

double state_estimate_t::get_temp(void)
{
	return Temp;
}


#ifndef GET_VARIABLE_NAME
#define GET_VARIABLE_NAME(Variable) (#Variable)
#endif // !GET_VARIABLE_NAME


/** @name Logging class for state estimator
* Defines how logging should be done for this class
*/
char state_estimator_log_entry_t::update(state_estimate_t* new_state)
{
	time = new_state->get_time();

	new_state->get_pos_glob(pos_global);
	new_state->get_vel_glob(vel_global);
	new_state->get_acc_glob(accel_global);
	new_state->get_pos_rel(pos_relative);
	new_state->get_vel_rel(vel_relative);
	new_state->get_acc_rel(accel_relative);

	new_state->get_att_quat(quat);
	new_state->get_att_tb(att);
	new_state->get_att_rates(att_rates);
	continuous_yaw = new_state->get_continuous_heading();
	new_state->get_omega(omega);

	v_batt = new_state->get_v_batt();

	Temp = new_state->get_temp();

	alt = new_state->get_alt();
	alt_vel = new_state->get_alt_vel();
	alt_acc = new_state->get_alt_acc();
	return 0;
}
char state_estimator_log_entry_t::print_vec(FILE* file, double* vec_in, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%.4F", vec_in[i]);
	}
	return 0;
}

char state_estimator_log_entry_t::print_header_vec(FILE* file, const char* var_name, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%s_%i", var_name, i);
	}
	return 0;
}

char state_estimator_log_entry_t::print_header(FILE* file)
{
	fprintf(file, ",time_ns");

	print_header_vec(file, GET_VARIABLE_NAME(pos_global), 3);
	print_header_vec(file, GET_VARIABLE_NAME(vel_global), 3);
	print_header_vec(file, GET_VARIABLE_NAME(accel_global), 3);
	print_header_vec(file, GET_VARIABLE_NAME(pos_relative), 3);
	print_header_vec(file, GET_VARIABLE_NAME(vel_relative), 3);
	print_header_vec(file, GET_VARIABLE_NAME(accel_relative), 3);
	print_header_vec(file, GET_VARIABLE_NAME(quat), 4);
	print_header_vec(file, GET_VARIABLE_NAME(att), 3);
	fprintf(file, ",%s", GET_VARIABLE_NAME(continuous_yaw));
	print_header_vec(file, GET_VARIABLE_NAME(att_rates), 3);
	print_header_vec(file, GET_VARIABLE_NAME(omega), 3);
	fprintf(file, ",%s", GET_VARIABLE_NAME(v_batt));
	fprintf(file, ",%s", GET_VARIABLE_NAME(Temp));
	fprintf(file, ",%s", GET_VARIABLE_NAME(alt));
	fprintf(file, ",%s", GET_VARIABLE_NAME(alt_vel));
	fprintf(file, ",%s", GET_VARIABLE_NAME(alt_acc));
	return 0;
}
char state_estimator_log_entry_t::print_entry(FILE* file)
{
	fprintf(file, ",%" PRIu64, time);

	print_vec(file, pos_global, 3);
	print_vec(file, vel_global, 3);
	print_vec(file, accel_global, 3);
	print_vec(file, pos_relative, 3);
	print_vec(file, vel_relative, 3);
	print_vec(file, accel_relative, 3);
	print_vec(file, quat, 4);
	print_vec(file, att, 3);
	fprintf(file, ",%.4F", continuous_yaw);
	print_vec(file, att_rates, 3);
	print_vec(file, omega, 3);
	fprintf(file, ",%.1F", v_batt);
	fprintf(file, ",%.1F", Temp);
	fprintf(file, ",%.4F", alt);
	fprintf(file, ",%.4F", alt_vel);
	fprintf(file, ",%.4F", alt_acc);
	return 0;
}