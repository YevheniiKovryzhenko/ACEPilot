/*
 * IMU_9DOF_gen.hpp
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
 * Last Edit:  09/03/2022 (MM/DD/YYYY)
 *
 * Summary :
 * This contains the nessesary framework for operating IMU:
 *		- IMU-9DOF: combination of gyro + accel + mag
 */

#ifndef SENSORS_GEN_HPP
#define SENSORS_GEN_HPP
#include "gyro_gen.hpp"
#include "accel_gen.hpp"
#include "compass_gen.hpp"

typedef struct IMU_9DOF_gen_settings_t
{
	coordinate_frames_gen_t frame_type;// = ENU;
	gyro_gen_settings_t gyro;
	accel_settings_t accel;
	bool use_compass;
	compass_gen_settings_t compass;
	bool enable_logging;
	bool log_raw;
	bool log_dmp;
	bool log_fused;
}IMU_9DOF_gen_settings_t;

/* General class for all IMU-9DOF instances */
class IMU_9DOF_gen_t
{
private:
	bool initialized = false;
	uint64_t time = rc_nanos_since_boot();

	IMU_9DOF_gen_settings_t settings;

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
	char init(IMU_9DOF_gen_settings_t& new_settings);
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
extern IMU_9DOF_gen_t imu;


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
	char update(IMU_9DOF_gen_t& new_state, IMU_9DOF_gen_settings_t& new_settings);
	char print_header(FILE* file, const char* prefix, IMU_9DOF_gen_settings_t& new_settings);
	char print_entry(FILE* file, IMU_9DOF_gen_settings_t& new_settings);
};

int parse_IMU_9DOF_gen_settings(json_object* in_json, const char* name, IMU_9DOF_gen_settings_t& sensor);
#endif // !SENSORS_GEN_HPP
