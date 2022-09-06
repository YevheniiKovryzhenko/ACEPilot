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
 * Last Edit:  09/05/2022 (MM/DD/YYYY)
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
#include "IMU_9DOF_gen.hpp"

#include <cstring>
#include "signal_filter_gen.hpp"
#include "settings_gen.hpp"

#ifndef GET_VARIABLE_NAME
#define GET_VARIABLE_NAME(Variable) (#Variable)
#endif // !GET_VARIABLE_NAME

IMU_9DOF_gen_t IMU0{};	//IMU-9DOF with Gyro + Accel + Mag

/* General class for all IMU-9DOF instances */
char IMU_9DOF_gen_t::init(IMU_9DOF_gen_settings_t& new_settings)
{
	if (!new_settings.enable) 
	{
		settings.enable = false;
		return 0;
	}
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
	if (new_settings.compass.enable)
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
	settings.compass.enable = true;

	//gyro
	settings.gyro.frame_type = ENU;
	for (int i = 0; i < 3; i++) {
		settings.gyro.filter[i].dt = DT;
		settings.gyro.filter[i].type = Lowpass;
		settings.gyro.filter[i].enable_saturation = false;
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
		settings.accel.filter[i].enable_saturation = false;
	}

	//compass
	settings.compass.frame_type = ENU;

	printf("IMU test settings\n");
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


char IMU_9DOF_gen_t::update_att_from_quat_est(double new_att_quat_est_raw[4])
{
	/* save new raw data internally */
	for (int i = 0; i < 4; i++)	att_quat_est_raw[i] = new_att_quat_est_raw[i];

	/* transform from sensor frame to NED */
	att_quat_est_NED[0] = att_quat_est_raw[0];
	rotate2NED(settings.frame_type, &att_quat_est_NED[1], &att_quat_est_raw[1]);

	/* normalize quaternians just in case, but do not touch "raw" data */
	rc_normalize_quaternion_array(att_quat_est_NED);
	rc_normalize_quaternion_array(new_att_quat_est_raw);

	/* calculate roll pitch and yaw from quaternians */
	rc_quaternion_to_tb_array(new_att_quat_est_raw, att_tb_est_raw); //use normalized raw quaternians
	rc_quaternion_to_tb_array(att_quat_est_NED, att_tb_est_NED);

	/*	Heading
	continuous heading is more annoying since we have to detect spins
	also make sign negative since NED coordinates has Z point down
	*/
	double diff_yaw = att_tb_est_NED[2] + (num_heading_spins_est_NED * TWO_PI) - continuous_heading_est_NED;
	// detect the crossover point at +-PI and update num yaw spins
	if (diff_yaw < -M_PI) num_heading_spins_est_NED++;
	else if (diff_yaw > M_PI) num_heading_spins_est_NED--;

	// finally the new value can be written
	continuous_heading_est_NED = att_tb_est_NED[2] + (num_heading_spins_est_NED * TWO_PI);
	return 0;
}

char IMU_9DOF_gen_t::update_accel_grav_est(double new_accel_grav_est_raw[3])
{
	/* save new raw data internally */
	for (int i = 0; i < 3; i++)	accel_grav_est_raw[i] = new_accel_grav_est_raw[i];

	/* transform from sensor frame to NED */
	rotate2NED(settings.frame_type, accel_grav_est_NED, accel_grav_est_raw);
	return 0;
}

char IMU_9DOF_gen_t::update_accel_lin_est(double new_accel_lin_est_raw[3])
{
	/* save new raw data internally */
	for (int i = 0; i < 3; i++)	accel_lin_est_raw[i] = new_accel_lin_est_raw[i];

	/* transform from sensor frame to NED */
	rotate2NED(settings.frame_type, accel_lin_est_NED, accel_lin_est_raw);
	return 0;
}

char IMU_9DOF_gen_t::update_Temp_est(double new_Temp)
{
	
	Temp = new_Temp;
	return 0;
}
char IMU_9DOF_gen_t::update_calibration(uint8_t new_calibration[4])
{
	/* save new raw data internally */
	for (int i = 0; i < 3; i++)	calibration[i] = new_calibration[i];
	return 0;
}

char IMU_9DOF_gen_t::march(void)
{
	if (!settings.enable) return 0;
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in march: IMU_9DOF not initialized.\n");
		return -1;
	}
	gyro.march();
	accel.march();
	if (settings.compass.enable) compass.march();

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

void IMU_9DOF_gen_t::get_calibration(uint8_t* buff)
{
	for (int i = 0; i < 4; i++) buff[i] = calibration[i];
	return;
}
double IMU_9DOF_gen_t::get_Temp(void)
{
	return Temp;
}
void IMU_9DOF_gen_t::get_tb_est_raw(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = att_tb_est_raw[i];
	return;
}
void IMU_9DOF_gen_t::get_tb_est(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = att_tb_est_NED[i];
	return;
}
void IMU_9DOF_gen_t::get_quat_est_raw(double* buff)
{
	for (int i = 0; i < 4; i++) buff[i] = att_quat_est_raw[i];
	return;
}
void IMU_9DOF_gen_t::get_quat_est(double* buff)
{
	for (int i = 0; i < 4; i++) buff[i] = att_quat_est_NED[i];
	return;
}
double IMU_9DOF_gen_t::get_continuous_yaw_est(void)
{
	return continuous_heading_est_NED;
}

void IMU_9DOF_gen_t::get_accel_lin_est_raw(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = accel_lin_est_raw[i];
	return;
}
void IMU_9DOF_gen_t::get_accel_lin_est(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = accel_lin_est_NED[i];
	return;
}
void IMU_9DOF_gen_t::get_accel_grav_est_raw(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = accel_grav_est_raw[i];
	return;
}
void IMU_9DOF_gen_t::get_accel_grav_est(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = accel_grav_est_NED[i];
	return;
}

char IMU_9DOF_gen_t::reset(void)
{
	if (!settings.enable) return 0;
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in reset: IMU_9DOF not initialized.\n");
		return -1;
	}
	char tmp = gyro.reset();
	if (accel.reset() < 0) tmp = -1;
	if (settings.compass.enable && compass.reset() < 0)tmp = -1;
	
	return tmp;
}

void IMU_9DOF_gen_t::cleanup(void)
{
	if (!settings.enable) return;
	if (!initialized) return;
	gyro.cleanup();
	accel.cleanup();
	compass.cleanup();
	initialized = false;
	return;
}

/** @name Logging class for IMU-9DOF
* Defines how logging should be done for this class
*/
char IMU_9DOF_log_entry_t::update(IMU_9DOF_gen_t& new_state, IMU_9DOF_gen_settings_t& new_settings)
{
	if (!new_settings.enable) return 0;
	if (!new_settings.enable_logging) return 0;//return if disabled

	time = new_state.get_time();
	
	if (new_settings.log_dmp)
	{
		if (new_settings.log_raw) new_state.get_quat_dmp_raw(att_quat_dmp_raw);
		new_state.get_quat_dmp(att_quat_dmp_NED);
		if (new_settings.log_raw) new_state.get_tb_dmp_raw(att_tb_dmp_raw);
		new_state.get_tb_dmp(att_tb_dmp_NED);
		continuous_heading_dmp_NED = new_state.get_continuous_yaw_dmp();
	}

	if (new_settings.log_fused)
	{
		if (new_settings.log_raw) new_state.get_quat_fused_raw(att_quat_fused_raw);
		new_state.get_quat_fused(att_quat_fused_NED);
		if (new_settings.log_raw) new_state.get_tb_fused_raw(att_tb_fused_raw);
		new_state.get_tb_fused(att_tb_fused_NED);
		continuous_heading_fused_NED = new_state.get_continuous_yaw_fused();
	}

	if (new_settings.log_est)
	{
		new_state.get_calibration(calibration);
		Temp = new_state.get_Temp();

		if (new_settings.log_raw) new_state.get_tb_est_raw(att_tb_est_raw);
		new_state.get_tb_est(att_tb_est_NED);

		if (new_settings.log_raw) new_state.get_quat_est_raw(att_quat_est_raw);
		new_state.get_quat_est(att_quat_est_NED);

		if (new_settings.log_raw) new_state.get_accel_lin_est_raw(accel_lin_est_raw);
		new_state.get_accel_lin_est(accel_lin_est_NED);
		if (new_settings.log_raw) new_state.get_accel_grav_est_raw(accel_grav_est_raw);
		new_state.get_accel_grav_est(accel_grav_est_NED);
		continuous_heading_est_NED = new_state.get_continuous_yaw_est();
	}

	gyro.update(new_state.gyro, new_settings.gyro);
	accel.update(new_state.accel, new_settings.accel);
	compass.update(new_state.compass, new_settings.compass);

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
char IMU_9DOF_log_entry_t::print_vec(FILE* file, uint8_t* vec_in, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%i", vec_in[i]);
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

char IMU_9DOF_log_entry_t::print_header(FILE* file, const char* prefix, IMU_9DOF_gen_settings_t& new_settings)
{
	if (!new_settings.enable) return 0;
	if (!new_settings.enable_logging) return 0;//return if disabled

	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(time));

	if (new_settings.log_dmp)
	{
		if (new_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(att_quat_dmp_raw), 4);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(att_quat_dmp_NED), 4);
		if (new_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_dmp_raw), 3);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_dmp_NED), 3);
		fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(continuous_heading_dmp_NED));
	}

	if (new_settings.log_fused)
	{
		if (new_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(att_quat_fused_raw), 4);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(att_quat_fused_NED), 4);
		if (new_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_fused_raw), 3);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_fused_NED), 3);
		fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(continuous_heading_fused_NED));
	}

	if (new_settings.log_est)
	{
		print_header_vec(file, prefix, GET_VARIABLE_NAME(calibration), 4);
		fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(Temp));
		if (new_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_est_raw), 3);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_est_NED), 3);
		if (new_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(att_quat_est_raw), 4);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(att_quat_est_NED), 4);

		if (new_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(accel_lin_est_raw), 3);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(accel_lin_est_NED), 3);

		if (new_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(accel_grav_est_raw), 3);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(accel_grav_est_NED), 3);
		fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(continuous_heading_est_NED));
	}

	char buffer[100]; // <- danger, only storage for 100 characters.
	const char* buff_pt = buffer;
	strncpy(buffer, prefix, 50);
	strncat(buffer, "gyro_", 50);
	gyro.print_header(file, buffer, new_settings.gyro);

	strncpy(buffer, prefix, 50);
	strncat(buffer, "accel_", 50);
	accel.print_header(file, buffer, new_settings.accel);

	strncpy(buffer, prefix, 50);
	strncat(buffer, "compass_", 50);
	compass.print_header(file, buffer, new_settings.compass);

	return 0;
}
char IMU_9DOF_log_entry_t::print_entry(FILE* file, IMU_9DOF_gen_settings_t& new_settings)
{
	if (!new_settings.enable) return 0;
	if (!new_settings.enable_logging) return 0;//return if disabled

	fprintf(file, ",%" PRIu64, time);
	
	if (new_settings.log_dmp)
	{
		if (new_settings.log_raw) print_vec(file, att_quat_dmp_raw, 4);
		print_vec(file, att_quat_dmp_NED, 4);
		if (new_settings.log_raw) print_vec(file, att_tb_dmp_raw, 3);
		print_vec(file, att_tb_dmp_NED, 3);
		fprintf(file, ",%.4F", continuous_heading_dmp_NED);
	}

	if (new_settings.log_fused)
	{
		if (new_settings.log_raw) print_vec(file, att_quat_fused_raw, 4);
		print_vec(file, att_quat_fused_NED, 4);
		if (new_settings.log_raw) print_vec(file, att_tb_fused_raw, 3);
		print_vec(file, att_tb_fused_NED, 3);
		fprintf(file, ",%.4F", continuous_heading_fused_NED);
	}

	if (new_settings.log_est)
	{
		print_vec(file, calibration, 4);
		fprintf(file, ",%.4F", Temp);
		if (new_settings.log_raw) print_vec(file, att_tb_est_raw, 3);
		print_vec(file, att_tb_est_NED, 3);
		if (new_settings.log_raw) print_vec(file, att_quat_est_raw, 4);
		print_vec(file, att_quat_est_NED, 4);

		if (new_settings.log_raw) print_vec(file, accel_lin_est_raw, 3);
		print_vec(file, accel_lin_est_NED, 3);

		if (new_settings.log_raw) print_vec(file, accel_grav_est_raw, 3);
		print_vec(file, accel_grav_est_NED, 3);
		fprintf(file, ",%.4F", continuous_heading_est_NED);
	}

	gyro.print_entry(file, new_settings.gyro);
	accel.print_entry(file, new_settings.accel);
	compass.print_entry(file, new_settings.compass);
	return 0;
}


int parse_IMU_9DOF_gen_settings(json_object* in_json, const char* name, IMU_9DOF_gen_settings_t& sensor)
{
	struct json_object* tmp_main_json = NULL;
	struct json_object* tmp = NULL;    // temp object

	//find filter entry
	if (json_object_object_get_ex(in_json, name, &tmp_main_json) == 0)
	{
		fprintf(stderr, "ERROR: can't find %s entry\n", name);
		return -1;
	}
	if (json_object_is_type(tmp_main_json, json_type_object) == 0)
	{
		fprintf(stderr, "ERROR: %s must be an object\n", name);
		return -1;
	}

	if (parse_bool(tmp_main_json, "enable", sensor.enable))
	{
		fprintf(stderr, "ERROR: failed to parse enable flag for %s\n", name);
		return -1;
	}
	if (!sensor.enable) return 0;

	// Parse coordinate frame type:
	if (parse_coordinate_frame_gen_type(tmp_main_json, "frame_type", sensor.frame_type))
	{
		fprintf(stderr, "ERROR: failed to parse frame_type flag for %s\n", name);
		return -1;
	}

	// Parse gyro
	if (parse_gyro_gen_settings(tmp_main_json, "gyro", sensor.gyro))
	{
		fprintf(stderr, "ERROR: failed to parse gyro for %s\n", name);
		return -1;
	}
	// Parse accel
	if (parse_accel_gen_settings(tmp_main_json, "accel", sensor.accel))
	{
		fprintf(stderr, "ERROR: failed to parse accel for %s\n", name);
		return -1;
	}
	// Parse compass
	if (parse_bool(tmp_main_json, "use_compass", sensor.use_compass))
	{
		fprintf(stderr, "ERROR: failed to parse use_compass flag for %s\n", name);
		return -1;
	}
	if (parse_compass_gen_settings(tmp_main_json, "compass", sensor.compass))
	{
		fprintf(stderr, "ERROR: failed to parse compass for %s\n", name);
		return -1;
	}

	// Parse other entries
	if (parse_bool(tmp_main_json, "enable_logging", sensor.enable_logging))
	{
		fprintf(stderr, "ERROR: failed to parse enable_logging flag for %s\n", name);
		return -1;
	}

	if (!sensor.enable_logging) return 0; //done if logging is disabled
	if (parse_bool(tmp_main_json, "log_raw", sensor.log_raw))
	{
		fprintf(stderr, "ERROR: failed to parse log_raw flag for %s\n", name);
		return -1;
	}
	if (parse_bool(tmp_main_json, "log_dmp", sensor.log_dmp))
	{
		fprintf(stderr, "ERROR: failed to parse log_dmp flag for %s\n", name);
		return -1;
	}
	if (parse_bool(tmp_main_json, "log_fused", sensor.log_fused))
	{
		fprintf(stderr, "ERROR: failed to parse log_fused flag for %s\n", name);
		return -1;
	}
	if (parse_bool(tmp_main_json, "log_est", sensor.log_est))
	{
		fprintf(stderr, "ERROR: failed to parse log_est flag for %s\n", name);
		return -1;
	}

	return 0;
}