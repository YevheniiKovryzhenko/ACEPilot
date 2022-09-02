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
 * Last Edit:  09/01/2022 (MM/DD/YYYY)
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

#include "rc_pilot_defs.hpp"
#include "settings.hpp"
#include "tools.h"
 //#include "input_manager.h"	
#include "comms_tmp_data_packet.h"
#include "gps.hpp"
#include "benchmark.hpp" //for velocity estimation

#include "state_estimator.hpp"
#include "sensors_gen.hpp"
#include "voltage_sensor_gen.hpp"
#include "mocap_gen.hpp"
#include "KF.hpp"
#include "EKF.hpp"
#include "EKF2.hpp"

#define TWO_PI (M_PI*2.0)

rc_mpu_data_t mpu_data; // extern variable in state_estimator.hpp
state_estimate_t state_estimate{}; // extern variable in state_estimator.hpp
//ext_mag_t ext_mag;

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
		mocap.update_time(GS_RX.time, GS_RX.trackingValid);
		if (mocap.is_valid())
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
		}		

		//mocap_check_timeout();
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
	if (settings.enable_mocap && mocap.is_valid()) //assumed to be the most accurate sourse, so directly update position and attitude
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
	KF_altitude.march(accel_global[2], pos_global[2]);
	alt = KF_altitude.get_alt();
	alt_vel = KF_altitude.get_alt_vel();
	alt_acc = KF_altitude.get_alt_acc();

	/* Update Attitude EKF */
	double tmp[3];
	imu.compass.get(tmp);
	EKF1.march(omega, accel_relative, tmp);
	EKF2.march(omega, accel_relative, tmp);
}

char state_estimate_t::init_internal_filters(void)
{
	if (unlikely(KF_altitude.init() < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize altitude KF\n");
		return -1;
	}

	if (unlikely(EKF1.reset() < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize EKF-1\n");
		return -1;
	}

	if (unlikely(EKF2.reset() < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize EKF-1\n");
		return -1;
	}
	return 0;
}

void state_estimate_t::cleanup_internal_filters(void)
{
	EKF1.reset();
	EKF2.reset();

	KF_altitude.cleanup();

	return;
}

/*
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
*/

int state_estimate_t::init(void)
{
	if (initialized) {
		fprintf(stderr, "ERROR in init: estimator is already initialized\n");
		return -1;
	}

	if (unlikely(batt.init(settings.battery) < 0))
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


	if (unlikely(init_internal_filters() < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize internal filters\n");
		return -1;
	}

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

	cleanup_internal_filters();
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