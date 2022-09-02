/*
 * sensors_gen.cpp
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
 * Last Edit:  09/02/2022 (MM/DD/YYYY)
 *
 * Summary :
 * This contains the nessesary framework for operating sensors. Currently supports:
 *		- Voltage sensor
 *		- Barometer
 *		- Gyroscope
 *		- Accelerometer
 *		- Magnetometer (Compass)
 *		- IMU-9DOF: combination of gyro + accel + mag
 */
#include "sensors_gen.hpp"

#include <cstring>
#include "signal_filter_gen.hpp"


#include "coordinate_frames_gen.hpp"
//#include "settings_gen.hpp"
//#include "signal_filter_gen.hpp"

#ifndef GET_VARIABLE_NAME
#define GET_VARIABLE_NAME(Variable) (#Variable)
#endif // !GET_VARIABLE_NAME

barometer_gen_t bmp{};	//barometer
IMU_9DOF_gen_t imu{};	//IMU-9DOF with Gyro + Accel + Mag

/* Methods for general BAROMETER class */
char barometer_gen_t::init(void)
{
	if (unlikely(initialized))
	{
		fprintf(stderr, "ERROR in init: barometer already initialized.\n");
		return -1;
	}

	initialized = true;
	first_run = true;
	return 0;
}

char barometer_gen_t::march(double new_pr, double new_alt, double new_temp)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in march: barometer is not initialized.\n");
		return -1;
	}
	if (first_run) initial_alt = new_alt;
	pressure_raw = new_pr;
	temperature_raw = new_temp;
	altitude_raw = new_alt;
	alt_ground = new_alt - initial_alt;

	first_run = false;
	return 0;
}

double barometer_gen_t::get_alt(void)
{
	return altitude_raw;
}
double barometer_gen_t::get_alt_ground(void)
{
	return alt_ground;
}
double barometer_gen_t::get_pr(void)
{
	return pressure_raw;
}
double barometer_gen_t::get_temp(void)
{
	return temperature_raw;
}

char barometer_gen_t::reset(void)
{
	first_run = true;
	return 0;
}

void barometer_gen_t::cleanup(void)
{
	if (!initialized) return;
	initialized = false;
	first_run = true;
	return;
}



/* General class for all GYRO instances */
char gyro_gen_t::init(gyro_settings_t new_gyro_settings)
{
	if (unlikely(initialized))
	{
		fprintf(stderr, "ERROR in init: gyro is already initialized.\n");
		return -1;
	}

	if (unlikely(lp.set(new_gyro_settings.filter) < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize gyro filters\n");
		return -1;
	}
	settings = new_gyro_settings;

	updated = false;
	initialized = true;
	return 0;
}
char gyro_gen_t::init(void)
{
	/* assume some default values */
	settings.frame_type = ENU;
	for (int i = 0; i < 3; i++) {
		settings.filter[i].dt = DT;
		settings.filter[i].type = Lowpass;
		settings.filter[i].en_saturation = false;
	}
	settings.filter[0].tc = 6.0 * DT;
	settings.filter[1].tc = 6.0 * DT;
	settings.filter[2].tc = 7.0 * DT;

	return init(settings);
}

bool gyro_gen_t::is_initialized(void)
{
	return initialized;
}

char gyro_gen_t::update(double new_gyro_raw[3])
{
	if (updated)
	{
		printf("WARNING in update: already updated gyro data.\n");
		return 0; //skip if already updated
	}
	/* save new mocap data internally */
	raw[0] = new_gyro_raw[0];
	raw[1] = new_gyro_raw[1];
	raw[2] = new_gyro_raw[2];

	/* transform from Gyro frame to NED */
	rotate2NED(settings.frame_type, raw_NED, raw);

	updated = true;
	time = rc_nanos_since_boot(); //update timer
	return 0;
}

char gyro_gen_t::march(void)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in march: gyro is not initialized.\n");
		return -1;
	}
	if (updated)
	{
		lp.march(filtered_NED, raw_NED);
		updated = false;
	}
	return 0;
}

uint64_t gyro_gen_t::get_time(void)
{
	return time;
}
void gyro_gen_t::get_raw(double* buff)
{
	for (int i = 0.0; i < 3; i++)
	{
		buff[i] = raw[i];
	}
	return;
}

void gyro_gen_t::get_raw_NED(double* buff)
{
	for (int i = 0.0; i < 3; i++)
	{
		buff[i] = raw_NED[i];
	}
	return;
}

void gyro_gen_t::get(double* buff)
{
	for (int i = 0.0; i < 3; i++)
	{
		buff[i] = filtered_NED[i];
	}
	return;
}

char gyro_gen_t::reset(void)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in reset: gyro is not initialized.\n");
		return -1;
	}
	updated = false;
	return lp.reset();
}

void gyro_gen_t::cleanup(void)
{
	if (!initialized) return;
	lp.cleanup();
	initialized = false;
	updated = false;
	return;
}

/* General class for all ACCELEROMETER instances */
char accel_gen_t::init(void)
{
	/* assume some default values */
	settings.frame_type = ENU;
	for (int i = 0; i < 3; i++) {
		settings.filter[i].dt = DT;
		settings.filter[i].tc = 20.0 * DT;
		settings.filter[i].type = Lowpass;
		settings.filter[i].en_saturation = false;
	}

	return init(settings);
}
char accel_gen_t::init(accel_settings_t new_acc_settings)
{
	if (unlikely(initialized))
	{
		fprintf(stderr, "ERROR in init: accelerometer is already initialized.\n");
		return -1;
	}

	if (unlikely(lp.set(new_acc_settings.filter) < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize accelerometer filters\n");
		return -1;
	}
	settings = new_acc_settings;

	updated = false;
	initialized = true;
	return 0;
}
bool accel_gen_t::is_initialized(void)
{
	return initialized;
}

char accel_gen_t::update(double new_acc_raw[3])
{
	if (updated)
	{
		printf("WARNING in update: already updated accelerometer data.\n");
		return 0; //skip if already updated
	}
	/* save new mocap data internally */
	raw[0] = new_acc_raw[0];
	raw[1] = new_acc_raw[1];
	raw[2] = new_acc_raw[2];

	/* transform from Accelerometer frame to NED */
	rotate2NED(settings.frame_type, raw_NED, raw);

	updated = true;
	time = rc_nanos_since_boot(); //update timer
	return 0;
}

char accel_gen_t::march(void)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in march: accelerometer is not initialized.\n");
		return -1;
	}
	if (updated)
	{
		lp.march(filtered_NED, raw_NED);
		updated = false;
	}
	return 0;
}

uint64_t accel_gen_t::get_time(void)
{
	return time;
}
void accel_gen_t::get_raw(double* buff)
{
	for (int i = 0.0; i < 3; i++)
	{
		buff[i] = raw[i];
	}
	return;
}

void accel_gen_t::get_raw_NED(double* buff)
{
	for (int i = 0.0; i < 3; i++)
	{
		buff[i] = raw_NED[i];
	}
	return;
}

void accel_gen_t::get(double* buff)
{
	for (int i = 0.0; i < 3; i++)
	{
		buff[i] = filtered_NED[i];
	}
	return;
}

char accel_gen_t::reset(void)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in reset: accelerometer is not initialized.\n");
		return -1;
	}
	updated = false;
	return lp.reset();
}

void accel_gen_t::cleanup(void)
{
	if (!initialized) return;
	lp.cleanup();
	initialized = false;
	updated = false;
	return;
}


/* General class for all COMPASS instances */
char compass_gen_t::init(void)
{
	settings.frame_type = ENU;
	return init(settings); //assume default values
}
char compass_gen_t::init(compass_settings_t new_compass_settings)
{
	if (unlikely(initialized))
	{
		fprintf(stderr, "ERROR in init: compass is already initialized.\n");
		return -1;
	}

	settings = new_compass_settings;

	updated = false;
	initialized = true;
	return 0;
}


bool compass_gen_t::is_initialized(void)
{
	return initialized;
}

char compass_gen_t::update(double new_compass_raw[3])
{
	if (updated)
	{
		printf("WARNING in update: already updated compass data.\n");
		return 0; //skip if already updated
	}
	/* save new mocap data internally */
	raw[0] = new_compass_raw[0];
	raw[1] = new_compass_raw[1];
	raw[2] = new_compass_raw[2];

	/* transform from Compass frame to NED */
	rotate2NED(settings.frame_type, raw_NED, raw);

	updated = true;
	time = rc_nanos_since_boot(); //update timer
	return 0;
}

char compass_gen_t::march(void)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in march: compass not initialized.\n");
		return -1;
	}
	if (updated)
	{
		updated = false;
	}
	return 0;
}

uint64_t compass_gen_t::get_time(void)
{
	return time;
}
void compass_gen_t::get_raw(double* buff)
{
	for (int i = 0.0; i < 3; i++)
	{
		buff[i] = raw[i];
	}
	return;
}

void compass_gen_t::get(double* buff)
{
	for (int i = 0.0; i < 3; i++)
	{
		buff[i] = raw_NED[i];
	}
	return;
}

char compass_gen_t::reset(void)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in reset: compass not initialized.\n");
		return -1;
	}
	updated = false;
	return 0;
}

void compass_gen_t::cleanup(void)
{
	if (!initialized) return;
	initialized = false;
	updated = false;
	return;
}


/* General class for all IMU-9DOF instances */
char IMU_9DOF_gen_t::init(IMU_9DOF_settings_t new_settings)
{
	if (unlikely(initialized))
	{
		fprintf(stderr, "ERROR in init: IMU-9DOF is already initialized.\n");
		return -1;
	}

	if (unlikely(gyro.init(new_settings.gyro) < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize gyroscope\n");
		return -1;
	}
	if (unlikely(accel.init(new_settings.accel) < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize accelerometer\n");
		return -1;
	}
	if (new_settings.en_compass)
	{
		if (unlikely(compass.init(new_settings.compass) < 0))
		{
			fprintf(stderr, "ERROR in init: failed to initialize compass\n");
			return -1;
		}
	}
	settings = new_settings;

	initialized = true;
	return 0;
}
char IMU_9DOF_gen_t::init(void)
{
	if (unlikely(initialized))
	{
		fprintf(stderr, "ERROR in init: IMU-9DOF is already initialized.\n");
		return -1;
	}

	/* assume default values */
	settings.frame_type = ENU;
	settings.en_compass = true;

	//gyro
	settings.gyro.frame_type = ENU;
	for (int i = 0; i < 3; i++) {
		settings.gyro.filter[i].dt = DT;
		settings.gyro.filter[i].type = Lowpass;
		settings.gyro.filter[i].en_saturation = false;
	}
	settings.gyro.filter[0].tc = 6.0 * DT;
	settings.gyro.filter[1].tc = 6.0 * DT;
	settings.gyro.filter[2].tc = 7.0 * DT;

	//accel
	settings.accel.frame_type = ENU;
	for (int i = 0; i < 3; i++) {
		settings.accel.filter[i].dt = DT;
		settings.accel.filter[i].tc = 20.0 * DT;
		settings.accel.filter[i].type = Lowpass;
		settings.accel.filter[i].en_saturation = false;
	}

	//compass
	settings.compass.frame_type = ENU;

	
	return init(settings);
}


bool IMU_9DOF_gen_t::is_initialized(void)
{
	return initialized;
}


char IMU_9DOF_gen_t::update_att_from_quat_dmp(double new_att_quat_dmp_raw[4])
{
	/* save new raw data internally */
	for (int i = 0; i < 4; i++)	att_quat_dmp_raw[i] = new_att_quat_dmp_raw[i];

	/* transform from sensor frame to NED */
	att_quat_dmp_NED[0] = att_quat_dmp_raw[0];
	rotate2NED(settings.frame_type, &att_quat_dmp_NED[1], &att_quat_dmp_raw[1]);

	/* normalize quaternians just in case, but do not touch "raw" data */
	rc_normalize_quaternion_array(att_quat_dmp_NED);
	rc_normalize_quaternion_array(new_att_quat_dmp_raw);

	/* calculate roll pitch and yaw from quaternians */
	rc_quaternion_to_tb_array(new_att_quat_dmp_raw, att_tb_dmp_raw); //use normalized raw quaternians
	rc_quaternion_to_tb_array(att_quat_dmp_NED, att_tb_dmp_NED);

	/*	Heading
	continuous heading is more annoying since we have to detect spins
	also make sign negative since NED coordinates has Z point down
	*/
	double diff_yaw = att_tb_dmp_NED[2] + (num_heading_spins_dmp_NED * TWO_PI) - continuous_heading_dmp_NED;
	// detect the crossover point at +-PI and update num yaw spins
	if (diff_yaw < -M_PI) num_heading_spins_dmp_NED++;
	else if (diff_yaw > M_PI) num_heading_spins_dmp_NED--;

	// finally the new value can be written
	continuous_heading_dmp_NED = att_tb_dmp_NED[2] + (num_heading_spins_dmp_NED * TWO_PI);
	return 0;
}

char IMU_9DOF_gen_t::update_att_from_quat_fused(double new_att_quat_fused_raw[4])
{
	/* save new raw data internally */
	for (int i = 0; i < 4; i++)	att_quat_fused_raw[i] = new_att_quat_fused_raw[i];

	/* transform from sensor frame to NED */
	att_quat_fused_NED[0] = att_quat_fused_raw[0];
	rotate2NED(settings.frame_type, &att_quat_fused_NED[1], &att_quat_fused_raw[1]);

	/* normalize quaternians just in case, but do not touch "raw" data */
	rc_normalize_quaternion_array(att_quat_fused_NED);
	rc_normalize_quaternion_array(new_att_quat_fused_raw);

	/* calculate roll pitch and yaw from quaternians */
	rc_quaternion_to_tb_array(new_att_quat_fused_raw, att_tb_fused_raw); //use normalized raw quaternians
	rc_quaternion_to_tb_array(att_quat_fused_NED, att_tb_fused_NED);

	/*	Heading
	continuous heading is more annoying since we have to detect spins
	also make sign negative since NED coordinates has Z point down
	*/
	double diff_yaw = att_tb_fused_NED[2] + (num_heading_spins_fused_NED * TWO_PI) - continuous_heading_fused_NED;
	// detect the crossover point at +-PI and update num yaw spins
	if (diff_yaw < -M_PI) num_heading_spins_fused_NED++;
	else if (diff_yaw > M_PI) num_heading_spins_fused_NED--;

	// finally the new value can be written
	continuous_heading_fused_NED = att_tb_fused_NED[2] + (num_heading_spins_fused_NED * TWO_PI);
	return 0;
}

char IMU_9DOF_gen_t::march(void)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in march: IMU_9DOF not initialized.\n");
		return -1;
	}
	gyro.march();
	accel.march();
	if (settings.en_compass) compass.march();

	time = rc_nanos_since_boot();
	return 0;
}
uint64_t IMU_9DOF_gen_t::get_time(void)
{
	return time;
}
void IMU_9DOF_gen_t::get_quat_dmp_raw(double* buff)
{
	for (int i = 0; i < 4; i++) buff[i] = att_quat_dmp_raw[i];
	return;
}
void IMU_9DOF_gen_t::get_quat_dmp(double* buff)
{
	for (int i = 0; i < 4; i++) buff[i] = att_quat_dmp_NED[i];
	return;
}
void IMU_9DOF_gen_t::get_tb_dmp_raw(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = att_tb_dmp_raw[i];
	return;
}
void IMU_9DOF_gen_t::get_tb_dmp(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = att_tb_dmp_NED[i];
	return;
}
double IMU_9DOF_gen_t::get_continuous_yaw_dmp(void)
{
	return continuous_heading_dmp_NED;
}
void IMU_9DOF_gen_t::get_quat_fused_raw(double* buff)
{
	for (int i = 0; i < 4; i++) buff[i] = att_quat_fused_raw[i];
	return;
}
void IMU_9DOF_gen_t::get_quat_fused(double* buff)
{
	for (int i = 0; i < 4; i++) buff[i] = att_quat_fused_NED[i];
	return;
}
void IMU_9DOF_gen_t::get_tb_fused_raw(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = att_tb_fused_raw[i];
	return;
}
void IMU_9DOF_gen_t::get_tb_fused(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = att_tb_fused_NED[i];
	return;
}
double IMU_9DOF_gen_t::get_continuous_yaw_fused(void)
{
	return continuous_heading_fused_NED;
}

char IMU_9DOF_gen_t::reset(void)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in reset: compass not initialized.\n");
		return -1;
	}
	char tmp = gyro.reset();
	if (accel.reset() < 0) tmp = -1;
	if (settings.en_compass && compass.reset() < 0)tmp = -1;
	
	return tmp;
}

void IMU_9DOF_gen_t::cleanup(void)
{
	if (!initialized) return;
	gyro.cleanup();
	accel.cleanup();
	compass.cleanup();
	initialized = false;
	return;
}


/** @name Logging class for barometer
* Defines how logging should be done for this class
*/
char barometer_log_entry_t::update(barometer_gen_t* new_state)
{
	pressure_raw = new_state->get_pr();
	altitude_raw = new_state->get_alt();
	temperature_raw = new_state->get_temp();
	alt_ground = new_state->get_alt_ground();

	return 0;
}
char barometer_log_entry_t::print_vec(FILE* file, double* vec_in, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%.4F", vec_in[i]);
	}
	return 0;
}

char barometer_log_entry_t::print_header_vec(FILE* file, const char* prefix, const char* var_name, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%s%s_%i", prefix, var_name, i);
	}
	return 0;
}

char barometer_log_entry_t::print_header(FILE* file, const char* prefix)
{
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(pressure_raw));
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(altitude_raw));
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(temperature_raw));
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(alt_ground));
	return 0;
}
char barometer_log_entry_t::print_entry(FILE* file)
{
	fprintf(file, ",%.4F", pressure_raw);
	fprintf(file, ",%.4F", altitude_raw);
	fprintf(file, ",%.4F", temperature_raw);
	fprintf(file, ",%.4F", alt_ground);
	return 0;
}


/** @name Logging class for gyro
* Defines how logging should be done for this class
*/
char gyro_log_entry_t::update(gyro_gen_t* new_state)
{
	time = new_state->get_time();
	new_state->get_raw(raw);
	new_state->get_raw_NED(raw_NED);
	new_state->get(filtered_NED);

	return 0;
}
char gyro_log_entry_t::print_vec(FILE* file, double* vec_in, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%.4F", vec_in[i]);
	}
	return 0;
}

char gyro_log_entry_t::print_header_vec(FILE* file, const char* prefix, const char* var_name, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%s%s_%i", prefix, var_name, i);
	}
	return 0;
}

char gyro_log_entry_t::print_header(FILE* file, const char* prefix)
{
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(time));
	print_header_vec(file, prefix, GET_VARIABLE_NAME(raw), 3);
	print_header_vec(file, prefix, GET_VARIABLE_NAME(raw_NED), 3);
	print_header_vec(file, prefix, GET_VARIABLE_NAME(filtered_NED), 3);
	return 0;
}
char gyro_log_entry_t::print_entry(FILE* file)
{
	fprintf(file, ",%" PRIu64, time);
	print_vec(file, raw, 3);
	print_vec(file, raw_NED, 3);
	print_vec(file, filtered_NED, 3);
	return 0;
}


/** @name Logging class for accel
* Defines how logging should be done for this class
*/
char accel_log_entry_t::update(accel_gen_t* new_state)
{
	time = new_state->get_time();
	new_state->get_raw(raw);
	new_state->get_raw_NED(raw_NED);
	new_state->get(filtered_NED);

	return 0;
}
char accel_log_entry_t::print_vec(FILE* file, double* vec_in, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%.4F", vec_in[i]);
	}
	return 0;
}

char accel_log_entry_t::print_header_vec(FILE* file, const char* prefix, const char* var_name, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%s%s_%i", prefix, var_name, i);
	}
	return 0;
}

char accel_log_entry_t::print_header(FILE* file, const char* prefix)
{
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(time));
	print_header_vec(file, prefix, GET_VARIABLE_NAME(raw), 3);
	print_header_vec(file, prefix, GET_VARIABLE_NAME(raw_NED), 3);
	print_header_vec(file, prefix, GET_VARIABLE_NAME(filtered_NED), 3);
	return 0;
}
char accel_log_entry_t::print_entry(FILE* file)
{
	fprintf(file, ",%" PRIu64, time);
	print_vec(file, raw, 3);
	print_vec(file, raw_NED, 3);
	print_vec(file, filtered_NED, 3);
	return 0;
}


/** @name Logging class for compass
* Defines how logging should be done for this class
*/
char compass_log_entry_t::update(compass_gen_t* new_state)
{
	time = new_state->get_time();
	new_state->get_raw(raw);
	new_state->get(raw_NED);

	return 0;
}
char compass_log_entry_t::print_vec(FILE* file, double* vec_in, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%.4F", vec_in[i]);
	}
	return 0;
}

char compass_log_entry_t::print_header_vec(FILE* file, const char* prefix, const char* var_name, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%s%s_%i", prefix, var_name, i);
	}
	return 0;
}

char compass_log_entry_t::print_header(FILE* file, const char* prefix)
{
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(time));
	print_header_vec(file, prefix, GET_VARIABLE_NAME(raw), 3);
	print_header_vec(file, prefix, GET_VARIABLE_NAME(raw_NED), 3);
	return 0;
}
char compass_log_entry_t::print_entry(FILE* file)
{
	fprintf(file, ",%" PRIu64, time);
	print_vec(file, raw, 3);
	print_vec(file, raw_NED, 3);
	return 0;
}


/** @name Logging class for IMU-9DOF
* Defines how logging should be done for this class
*/
char IMU_9DOF_log_entry_t::update(IMU_9DOF_gen_t* new_state)
{
	time = new_state->get_time();
	
	new_state->get_quat_dmp_raw(att_quat_dmp_raw);
	new_state->get_quat_dmp(att_quat_dmp_NED);
	new_state->get_tb_dmp_raw(att_tb_dmp_raw);
	new_state->get_tb_dmp(att_tb_dmp_NED);
	continuous_heading_dmp_NED = new_state->get_continuous_yaw_dmp();

	new_state->get_quat_fused_raw(att_quat_fused_raw);
	new_state->get_quat_fused(att_quat_fused_NED);
	new_state->get_tb_fused_raw(att_tb_fused_raw);
	new_state->get_tb_fused(att_tb_fused_NED);
	continuous_heading_fused_NED = new_state->get_continuous_yaw_fused();

	gyro.update(&new_state->gyro);
	accel.update(&new_state->accel);
	compass.update(&new_state->compass);

	return 0;
}
char IMU_9DOF_log_entry_t::print_vec(FILE* file, double* vec_in, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%.4F", vec_in[i]);
	}
	return 0;
}

char IMU_9DOF_log_entry_t::print_header_vec(FILE* file, const char* prefix, const char* var_name, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%s%s_%i", prefix, var_name, i);
	}
	return 0;
}

char IMU_9DOF_log_entry_t::print_header(FILE* file, const char* prefix)
{
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(time));

	print_header_vec(file, prefix, GET_VARIABLE_NAME(att_quat_dmp_raw), 4);
	print_header_vec(file, prefix, GET_VARIABLE_NAME(att_quat_dmp_NED), 4);
	print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_dmp_raw), 3);
	print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_dmp_NED), 3);
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(continuous_heading_dmp_NED));

	print_header_vec(file, prefix, GET_VARIABLE_NAME(att_quat_fused_raw), 4);
	print_header_vec(file, prefix, GET_VARIABLE_NAME(att_quat_fused_NED), 4);
	print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_fused_raw), 3);
	print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_fused_NED), 3);
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(continuous_heading_fused_NED));

	char buffer[100]; // <- danger, only storage for 100 characters.
	const char* buff_pt = buffer;
	strncpy(buffer, prefix, 50);
	strncat(buffer, "gyro_", 50);
	gyro.print_header(file, buffer);

	strncpy(buffer, prefix, 50);
	strncat(buffer, "accel_", 50);
	accel.print_header(file, buffer);

	strncpy(buffer, prefix, 50);
	strncat(buffer, "compass_", 50);
	compass.print_header(file, buffer);

	return 0;
}
char IMU_9DOF_log_entry_t::print_entry(FILE* file)
{
	fprintf(file, ",%" PRIu64, time);
	
	print_vec(file, att_quat_dmp_raw, 4);
	print_vec(file, att_quat_dmp_NED, 4);
	print_vec(file, att_tb_dmp_raw, 3);
	print_vec(file, att_tb_dmp_NED, 3);
	fprintf(file, ",%.4F", continuous_heading_dmp_NED);

	print_vec(file, att_quat_fused_raw, 4);
	print_vec(file, att_quat_fused_NED, 4);
	print_vec(file, att_tb_fused_raw, 3);
	print_vec(file, att_tb_fused_NED, 3);
	fprintf(file, ",%.4F", continuous_heading_fused_NED);

	gyro.print_entry(file);
	accel.print_entry(file);
	compass.print_entry(file);
	return 0;
}