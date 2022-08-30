/*
 * sensors_gen.hpp
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
 * This contains the nessesary framework for operating sensors. Currently supports:
 *		- Voltage sensor
 *		- Gyroscope
 *		- Accelerometer
 *		- Magnetometer (Compass)
 *		- IMU-9DOF: combination of gyro + accel + mag
 */

#ifndef SENSORS_GEN_HPP
#define SENSORS_GEN_HPP
#include <stdint.h> // for uint64_t
#include <rc/time.h>

//#include "coordinate_frames_gen.hpp"
#include "settings.h"
#include "signal_filter_gen.hpp"

 /* General class for all battery instances */
class battery_gen_t
{
private:
	bool initialized = false;
	double raw = 0.0;
	double filtered = 0.0;

	voltage_sensor_settings_t settings; //sensor settings

	// battery filter
	signal_filter1D_gen_t filter{};
public:
	char init(void);
	char init(voltage_sensor_settings_t new_settings);
	char init(voltage_sensor_settings_t new_settings, double new_in);
	bool is_initialized(void);

	char march(double new_v);

	double get_raw(void);
	double get(void);

	char reset(void);
	char reset(voltage_sensor_settings_t new_settings);
	void cleanup(void);
};

/* General class for all barometer instances */
class barometer_gen_t
{
private:
	bool initialized = false;
	double pressure_raw = 0.0; // (Pa)
	double altitude_raw = 0.0; // (m)
	double temperature_raw = 0.0; // (c)

	bool first_run = true;
	double initial_alt = 0.0;
	double alt_ground = 0.0; // (m) altitude from the initialization point
public:
	char init(void);
	bool is_initialized(void);

	char march(double new_pr, double new_alt, double new_temp);

	double get_alt(void);
	double get_alt_ground(void);
	double get_pr(void);
	double get_temp(void);

	char reset(void);
	void cleanup(void);
};

 /* General class for all gyro instances */
class gyro_gen_t
{
private:
	bool initialized = false;
	uint64_t time = rc_nanos_since_boot();
	bool updated = false;

	gyro_settings_t settings; //settings for the gyroscope

	double raw[3] = { 0.0 , 0.0, 0.0 };
	double raw_NED[3] = { 0.0 , 0.0, 0.0 };
	double filtered_NED[3] = { 0.0 , 0.0, 0.0 };

	// gyro filters
	signal_filter3D_gen_t lp{};
public:
	/* Initialization */
	char init(void);
	char init(gyro_settings_t new_gyro_settings);
	bool is_initialized(void);

	/* Updating */
	uint64_t get_time(void);
	char update(double new_gyro_raw[3]);
	char march(void); //marches filters only is was updated

	/* Data retrieval */
	void get_raw(double* buff);
	void get_raw_NED(double* buff);
	void get(double* buff);

	/* Reset and cleanup */
	char reset(void);
	void cleanup(void);
};



/* General class for all accelerometer instances */
class accel_gen_t
{
private:
	bool initialized = false;
	uint64_t time = rc_nanos_since_boot();
	bool updated = false;

	accel_settings_t settings;  //settings for the accelerometer

	double raw[3] = { 0.0 , 0.0, 0.0 };
	double raw_NED[3] = { 0.0 , 0.0, 0.0 };
	double filtered_NED[3] = { 0.0 , 0.0, 0.0 };

	// accelerometer filters
	signal_filter3D_gen_t lp{};
public:
	/* Initialization */
	char init(void);
	char init(accel_settings_t new_acc_settings);
	bool is_initialized(void);

	/* Updating */
	char update(double new_acc_raw[3]);
	char march(void); //marches filters only is was updated

	/* Data retrieval */
	uint64_t get_time(void);
	void get_raw(double* buff);
	void get_raw_NED(double* buff);
	void get(double* buff);

	/* Reset and cleanup */
	char reset(void);
	void cleanup(void);
};

/* General class for all compass instances */
class compass_gen_t
{
private:
	bool initialized = false;
	uint64_t time = rc_nanos_since_boot();
	bool updated = false;

	compass_settings_t settings;  //settings for the accelerometer

	double raw[3] = { 0.0 , 0.0, 0.0 };
	double raw_NED[3] = { 0.0 , 0.0, 0.0 };

public:
	/* Initialization */
	char init(void);
	char init(compass_settings_t new_compass_settings);
	bool is_initialized(void);

	/* Updating */
	char update(double new_compass_raw[3]);
	char march(void); //marches filters only is was updated

	/* Data retrieval */
	uint64_t get_time(void);
	void get_raw(double* buff);
	void get(double* buff);

	/* Reset and cleanup */
	char reset(void);
	void cleanup(void);
};

/* General class for all IMU-9DOF instances */
class IMU_9DOF_gen_t
{
private:
	bool initialized = false;
	uint64_t time = rc_nanos_since_boot();

	IMU_9DOF_settings_t settings;

	/*
					DMP based attitude
	* Stands for Digital Motion Processor which is a feature of the MPU9250.
	* in this mode, the DMP will sample the sensors internally and fill a FIFO
	* buffer with the data at a fixed rate. Furthermore, the DMP will also
	* calculate a filtered orientation quaternion which is placed in the same
	* buffer. When new data is ready in the buffer, the IMU sends an interrupt to
	* the BeagleBone triggering the buffer read followed by the execution of a
	* function of your choosing set with the rc_mpu_set_dmp_callback() function.
	*/
	/* Gyroscope + Accelerometer only */
	double att_quat_dmp_raw[4];		///< quaternion from DMP based on ONLY Accel/Gyro
	double att_quat_dmp_NED[4];		///< normalized quaternion from DMP based on ONLY Accel/Gyro in NED
	double att_tb_dmp_raw[3];		///< Tait-Bryan angles (roll pitch yaw) in radians from DMP based on ONLY Accel/Gyro
	double att_tb_dmp_NED[3];		///< Tait-Bryan angles (roll pitch yaw) in radians from DMP based on ONLY Accel/Gyro in NED
	int num_heading_spins_dmp_NED = 0;
	double continuous_heading_dmp_NED = 0.0;

	/* Gyroscope + Accelerometer + Magnetometer */
	double att_quat_fused_raw[4];	///< quaternion from DMP based on Accel + Gyro + Mag
	double att_quat_fused_NED[4];	///< normalized quaternion from DMP based on Accel + Gyro + Mag in NED
	double att_tb_fused_raw[3];		///< Tait-Bryan angles (roll pitch yaw) in radians from DMP based on Accel + Gyro + Mag
	double att_tb_fused_NED[3];		///< Tait-Bryan angles (roll pitch yaw) in radians from DMP based on Accel + Gyro + Mag in NED
	int num_heading_spins_fused_NED = 0;
	double continuous_heading_fused_NED = 0.0;



public:
	/* Sensors */
	gyro_gen_t gyro{};
	accel_gen_t accel{};
	compass_gen_t compass{};

	/* Initialization */
	char init(void);
	char init(IMU_9DOF_settings_t new_settings);
	bool is_initialized(void);

	/* Updating */
	char update_att_from_quat_dmp(double new_att_quat_dmp_raw[4]);
	char update_att_from_quat_fused(double new_att_quat_fused_raw[4]);
	char march(void); //marches all sensors

	/* Data retrieval */
	uint64_t get_time(void);
	void get_quat_dmp_raw(double* buff);
	void get_quat_dmp(double* buff);
	void get_tb_dmp_raw(double* buff);
	void get_tb_dmp(double* buff);
	double get_continuous_yaw_dmp(void);

	void get_quat_fused_raw(double* buff);
	void get_quat_fused(double* buff);
	void get_tb_fused_raw(double* buff);
	void get_tb_fused(double* buff);
	double get_continuous_yaw_fused(void);

	/* Reset and cleanup */
	char reset(void);
	void cleanup(void);	
};


/** @name Logging class for battery
	* Defines how logging should be done for this class
	*/
class battery_log_entry_t
{
private:
	double raw;
	double filtered;

	char print_vec(FILE* file, double* vec_in, int size);
	char print_header_vec(FILE* file, const char* prefix, const char* var_name, int size);
public:
	char update(battery_gen_t* new_state);
	char print_header(FILE* file, const char* prefix);
	char print_entry(FILE* file);
};

/** @name Logging class for barometer
* Defines how logging should be done for this class
*/
class barometer_log_entry_t
{
private:
	double pressure_raw; // (Pa)
	double altitude_raw; // (m)
	double temperature_raw; // (c)

	double alt_ground; // (m) altitude from the initialization point

	char print_vec(FILE* file, double* vec_in, int size);
	char print_header_vec(FILE* file, const char* prefix, const char* var_name, int size);
public:
	char update(barometer_gen_t* new_state);
	char print_header(FILE* file, const char* prefix);
	char print_entry(FILE* file);
};


/** @name Logging class for gyro
* Defines how logging should be done for this class
*/
class gyro_log_entry_t
{
private:
	uint64_t time;
	
	double raw[3];
	double raw_NED[3];
	double filtered_NED[3];

	char print_vec(FILE* file, double* vec_in, int size);
	char print_header_vec(FILE* file, const char* prefix, const char* var_name, int size);
public:
	char update(gyro_gen_t* new_state);
	char print_header(FILE* file, const char* prefix);
	char print_entry(FILE* file);
};


/** @name Logging class for accel
* Defines how logging should be done for this class
*/
class accel_log_entry_t
{
private:
	uint64_t time;

	double raw[3];
	double raw_NED[3];
	double filtered_NED[3];

	char print_vec(FILE* file, double* vec_in, int size);
	char print_header_vec(FILE* file, const char* prefix, const char* var_name, int size);
public:
	char update(accel_gen_t* new_state);
	char print_header(FILE* file, const char* prefix);
	char print_entry(FILE* file);
};


/** @name Logging class for compass
* Defines how logging should be done for this class
*/
class compass_log_entry_t
{
private:
	uint64_t time;

	double raw[3];
	double raw_NED[3];

	char print_vec(FILE* file, double* vec_in, int size);
	char print_header_vec(FILE* file, const char* prefix, const char* var_name, int size);
public:
	char update(compass_gen_t* new_state);
	char print_header(FILE* file, const char* prefix);
	char print_entry(FILE* file);
};



/** @name Logging class for IMU-9DOF
* Defines how logging should be done for this class
*/
class IMU_9DOF_log_entry_t
{
private:
	uint64_t time;

	/*
					DMP based attitude
	* Stands for Digital Motion Processor which is a feature of the MPU9250.
	* in this mode, the DMP will sample the sensors internally and fill a FIFO
	* buffer with the data at a fixed rate. Furthermore, the DMP will also
	* calculate a filtered orientation quaternion which is placed in the same
	* buffer. When new data is ready in the buffer, the IMU sends an interrupt to
	* the BeagleBone triggering the buffer read followed by the execution of a
	* function of your choosing set with the rc_mpu_set_dmp_callback() function.
	*/
	/* Gyroscope + Accelerometer only */
	double att_quat_dmp_raw[4];		///< quaternion from DMP based on ONLY Accel/Gyro
	double att_quat_dmp_NED[4];		///< normalized quaternion from DMP based on ONLY Accel/Gyro in NED
	double att_tb_dmp_raw[3];		///< Tait-Bryan angles (roll pitch yaw) in radians from DMP based on ONLY Accel/Gyro
	double att_tb_dmp_NED[3];		///< Tait-Bryan angles (roll pitch yaw) in radians from DMP based on ONLY Accel/Gyro in NED
	double continuous_heading_dmp_NED;

	/* Gyroscope + Accelerometer + Magnetometer */
	double att_quat_fused_raw[4];	///< quaternion from DMP based on Accel + Gyro + Mag
	double att_quat_fused_NED[4];	///< normalized quaternion from DMP based on Accel + Gyro + Mag in NED
	double att_tb_fused_raw[3];		///< Tait-Bryan angles (roll pitch yaw) in radians from DMP based on Accel + Gyro + Mag
	double att_tb_fused_NED[3];		///< Tait-Bryan angles (roll pitch yaw) in radians from DMP based on Accel + Gyro + Mag in NED
	double continuous_heading_fused_NED;

	gyro_log_entry_t gyro{};
	accel_log_entry_t accel{};
	compass_log_entry_t compass{};

	char print_vec(FILE* file, double* vec_in, int size);
	char print_header_vec(FILE* file, const char* prefix, const char* var_name, int size);
public:
	char update(IMU_9DOF_gen_t* new_state);
	char print_header(FILE* file, const char* prefix);
	char print_entry(FILE* file);
};

#endif // !SENSORS_GEN_HPP
