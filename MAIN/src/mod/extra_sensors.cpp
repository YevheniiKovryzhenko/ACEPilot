/*
 * extra_sensors.cpp
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
 * Contains functionality for additional sensors, like external IMU
 */

#include "extra_sensors.hpp"
#include <rc/start_stop.h>

#include "thread_defs.hpp"
#include "settings.hpp"
#include "tools.h"

//#include "Adafruit_Sensor.hpp"
extra_sensors_t extra_sensors{};
Adafruit_BNO055 bno;
IMU_9DOF_gen_t IMU1{};

/* test functions for BNO055 */
int bno_init(void)
{
	if (unlikely(bno.begin(OPERATION_MODE_NDOF, -1, settings.servo_i2c_driver_id, BNO055_ADDRESS_A) < 0))
	{
		printf("WARNING in bno_init: failed to open on default i2c address, trying backup...\n");
		if (unlikely(bno.begin(OPERATION_MODE_NDOF, -1, settings.servo_i2c_driver_id, BNO055_ADDRESS_B) < 0))
		{
			fprintf(stderr, "ERROR in bno_init: failed to connec to bno055 i2c port\n");
			return -1;
		}
	}
	return 0;
}

char extra_sensors_t::bno_march(void)
{
	if (!settings.IMU1.enable) return 0;

	double tmp[3];
	if (finddt_s(time) > 1.0 / 100.0)
	{
		double tmp_quat[4];
		bno.getQuat(tmp_quat);
		IMU1.update_att_from_quat_est(tmp_quat);
		//bno.getVector(tmp, Adafruit_BNO055::VECTOR_EULER);
		//printEvent(tmp, SENSOR_TYPE_ORIENTATION);

		bno.getVector(tmp, Adafruit_BNO055::VECTOR_GYROSCOPE);
		IMU1.gyro.update(tmp);
		//printEvent(tmp, SENSOR_TYPE_GYROSCOPE);

		bno.getVector(tmp, Adafruit_BNO055::VECTOR_LINEARACCEL);
		IMU1.update_accel_lin_est(tmp);
		//printEvent(tmp, SENSOR_TYPE_LINEAR_ACCELERATION);

		bno.getVector(tmp, Adafruit_BNO055::VECTOR_ACCELEROMETER);
		IMU1.accel.update(tmp);
		//printEvent(tmp, SENSOR_TYPE_ACCELEROMETER);

		bno.getVector(tmp, Adafruit_BNO055::VECTOR_GRAVITY);
		IMU1.update_accel_grav_est(tmp);
		//printEvent(tmp, SENSOR_TYPE_GRAVITY);
	}
	if (settings.IMU1.compass.enable) {
		if (finddt_s(time_mag) > 1.0 / 20.0)
		{
			bno.getVector(tmp, Adafruit_BNO055::VECTOR_MAGNETOMETER);
			IMU1.compass.update(tmp);
			//printEvent(tmp, SENSOR_TYPE_MAGNETIC_FIELD);
			time_mag = rc_nanos_since_boot();
		}
	}
	if (finddt_s(time_temp) > 1.0)
	{
		uint8_t tmp_calib[4];
		bno.getCalibration(&tmp_calib[0], &tmp_calib[1], &tmp_calib[2], &tmp_calib[3]);
		IMU1.update_calibration(tmp_calib);

		IMU1.update_Temp_est(bno.getTemp());
		//printf("Temperature: %i (C)\n", bno.getTemp());
		time_temp = rc_nanos_since_boot();
	}
	return 0;
}

static void* __main_thread_func(__attribute__((unused)) void* ptr)
{
	if (!settings.IMU1.enable) return NULL;
	while (rc_get_state() != EXITING)
	{
		if (extra_sensors.update_main_thread() < 0)
		{
			printf("ERROR in __main_thread_func: failed to update main thread\n");
			return NULL;
		}
	}
	return NULL;
}

char extra_sensors_t::init(void)
{
	if (!settings.IMU1.enable) return 0;
	if (unlikely(initialized))
	{
		fprintf(stderr, "ERROR in init: already initialized\n");
		return -1;
	}

	if (unlikely(bno_init() < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize BNO055\n");
		return -1;
	}
	if (unlikely(IMU1.init(settings.IMU1) < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize IMU1\n");
		return -1;
	}
	if (unlikely(thread.init(EXTRA_SENSORS_PRI, FIFO) < 0))
	{
		fprintf(stderr, "ERROR in init: failed to start extra_sensors thread\n");
		return -1;
	}

	if (unlikely(thread.start(__main_thread_func) < 0))
	{
		fprintf(stderr, "ERROR in init: failed to start extra_sensors thread\n");
		return -1;
	}

	initialized = true;
	time = rc_nanos_since_boot();
	time_mag = time;
	time_temp = time;
	return 0;
}

/**
* @brief   Update main thread.
*
* @return -1 on error 0 on success.
*/
char extra_sensors_t::update_main_thread(void)
{
	if (!settings.IMU1.enable) return 0;
	if (unlikely(bno_march() < 0))
	{
		fprintf(stderr, "ERROR in update_main_thread: failed to march BNO055\n");
		return -1;
	}


	double tmp = finddt_s(time);
	time = rc_nanos_since_boot();
	if (tmp < 1.0 / EXTRA_SENSORS_HZ)
	{
		rc_usleep(1000000 * (1.0 / EXTRA_SENSORS_HZ - tmp));
	}
	return 0;
}



/**
* @brief   proper exit routine for all the functions.
*
* @return non-zero on error.
*/
void extra_sensors_t::cleanup(void)
{
	if (!settings.IMU1.enable) return;
	if (thread.stop(MOCAP_TOUT) < 0)
	{
		printf("WARNING in cleanup: failed to close mocap thread\n");
	}
	bno.cleanup();
	IMU1.cleanup();

	initialized = false;

	return;
}