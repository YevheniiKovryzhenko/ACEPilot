/*
 * commms_manager.cpp
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
 * Last Edit:  02/22/2022 (MM/DD/YYYY)
 *
 * Object that governs all the high level logic related to communications.
 */


#include "comms_manager.hpp"
#include <rc/time.h>
#include "settings.h"
#include "benchmark.h"
//#include "serial_transmit.hpp"

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

 /*
 * Invoke defaut constructor for all the built in and exernal types in the class
 */
comms_manager_t comms_manager{};
xbee_packet_t mocap_msg;

char comms_manager_t::init(void)
{
	if (unlikely(initialized))
	{
		printf("ERROR in init: already initialized\n");
		return -1;
	}
	mocap_initialized = false;
	mocap_active = false;
	initialized = true;

	return 0;
}

char comms_manager_t::mocap_start(const char* port, const int baudRate, void* buff, size_t size_buff)
{
	if (unlikely(mocap_initialized))
	{
		printf("ERROR in mocap_start: already started\n");
		return -1;
	}

	if (unlikely(mocap_transmit_line.open(port, baudRate) < 0))
	{
		printf("ERROR in mocap_start: failed to open port\n");
		return -1;
	}

	mocap_transmit_line.set_RX_line(buff, size_buff);

	mocap_initialized = true;
	mocap_active = false;
	return 0;
}

char comms_manager_t::mocap_update(void)
{
	if (unlikely(!mocap_initialized))
	{
		printf("ERROR in mocap update: not initialized\n");
		return -1;
	}

	if (unlikely(mocap_transmit_line.read() < 0))
	{
		printf("WARNING in mocap_update: failed to read new data\n");
		return 0;
	}

	return 0;
}

char comms_manager_t::mocap_cleanup(void)
{
	if (unlikely(!mocap_initialized)) return 0;
	mocap_initialized = false;
	mocap_active = false;

	mocap_transmit_line.close();
	return 0;
}


char comms_manager_t::update(void)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in update: not initialized\n");
		return -1;
	}


	if (settings.enable_mocap)
	{
		mocap_update();
		if (settings.log_benchmark) benchmark_timers.tXBEE = rc_nanos_since_boot();
	}

	return 0;
}

char comms_manager_t::cleanup(void)
{
	if (settings.enable_mocap) mocap_cleanup();

	initialized = 0;

	return 0;
}