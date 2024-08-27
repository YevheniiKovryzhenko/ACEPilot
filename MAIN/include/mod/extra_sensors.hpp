/*
 * extra_sensors.hpp
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

#ifndef EXTRA_SENSORS_HPP
#define EXTRA_SENSORS_HPP
#include "Adafruit_BNO055.hpp"
#include "thread_gen.hpp"
#include <rc/time.h>
#include "IMU_9DOF_gen.hpp"

class extra_sensors_t
{
private:
	bool initialized = false;
	uint64_t time;
	uint64_t time_mag;
	uint64_t time_temp;

	thread_gen_t thread{};

	char bno_march(void);

public:
	char init(void);

	char update_main_thread(void);

	void cleanup(void);

};

extern extra_sensors_t extra_sensors;
extern IMU_9DOF_gen_t IMU1;

#endif // !EXTERNAL_SENSORS_HPP
