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
 * Last Edit:  05/22/2022 (MM/DD/YYYY)
 *
 * Object that governs all the logic related to communications.
 */


#include "comms_manager.hpp"
#include <rc/time.h>
#include "settings.h"
#include "benchmark.h"
#include "gps.h"
#include "telem_packet_t.h"
#include "state_estimator.h"
#include "setpoint_manager.hpp"
#include "input_manager.hpp"
#include "state_machine.hpp"
#include "tools.h"
#include "thread_defs.h"

// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

 /*
 * Invoke defaut constructor for all the built in and exernal types in the class
 */
comms_manager_t comms_manager{};
xbee_packet_t GS_RX;
telem_packet_t GS_TX;

static void* __main_thread_func(__attribute__((unused)) void* ptr)
{
	while (rc_get_state() != EXITING)
	{
		if (comms_manager.update_main_thread() < 0)
		{
			printf("ERROR in __main_thread_func: failed to update main thread\n");
			return NULL;
		}
	}
	return NULL;
}
static void* __mocap_thread_func(__attribute__((unused)) void* ptr)
{
	while (rc_get_state() != EXITING)
	{
		if (comms_manager.update_mocap_thread() < 0)
		{
			printf("ERROR in __mocap_thread_func: failed to update thread\n");
			return NULL;
		}
	}
	return NULL;
}

/**
* @brief   Initializes all related variables and objects.
*
* @return non-zero on error.
*/
char comms_manager_t::init(void)
{
	if (unlikely(initialized))
	{
		printf("ERROR in init: already initialized\n");
		return -1;
	}
	mocap_thread_active_fl = false;
	mocap_thread_check_fl = false;
	mocap_initialized = false;
	mocap_active = false;
	mocap_en_TX = true; //allows sending data back
	mocap_thread_terminate_fl = false;

	comms_manager_thread_active_fl = false;
	comms_manager_thread_check_fl = false;
	comms_manager_thread_terminate_fl = false;

	if (unlikely(comms_manager_thread.init(COMMS_MANAGER_PRI, FIFO) < 0))
	{
		printf("ERROR in init: failed to start comms manager thread\n");
		return -1;
	}

	if (unlikely(comms_manager_thread.start(__main_thread_func) < 0))
	{
		printf("ERROR in init: failed to start comms manager thread\n");
		return -1;
	}


	if (settings.enable_gps) //must re-write GPS lib using Serial_Tools
	{
		/* Init GPS */
		printf("Initializing gps serial link for serial port\
        \n%s\n with baudrate \n%d\n", settings.serial_port_gps, settings.serial_baud_gps);
		if (gps_init(settings.serial_port_gps, settings.serial_baud_gps))
		{
			printf("ERROR: Failed to initialize GPS\n");
			return -1;
		}
	}
	
	initialized = true;

	return 0;
}

/**
* @brief   Update main thread.
*
* @return -1 on error 0 on success.
*/
char comms_manager_t::update_main_thread(void)
{
	
	// MOCAP:
	if (settings.enable_mocap) 
	{
		if (!mocap_thread_terminate_fl && !mocap_thread_active_fl && !mocap_thread.is_started())
		{
			mocap_thread_check_fl = false;

			if (unlikely(mocap_thread.init(MOCAP_PRI, FIFO) < 0))
			{
				printf("ERROR in update_main_thread: failed to start mocap thread\n");
				return -1;
			}

			if (unlikely(mocap_thread.start(__mocap_thread_func) < 0))
			{
				printf("ERROR in update_main_thread: failed to start mocap thread\n");
				return -1;
			}
		}

		if (mocap_thread_check_fl) mocap_thread_check_fl = false, mocap_thread_active_fl = true;
		else
		{
			mocap_thread_active_fl = false;
			if (finddt_s(benchmark_timers.tMOCAP) > MOCAP_THREAD_TX_TOUT)
			{
				printf("WARNING in update_main_thread: mocap thread is not responsding, terminating...\n");
				mocap_thread_terminate_fl = true;

				if (mocap_thread.stop(MOCAP_TOUT) < 0)
				{
					printf("WARNING in update_main_thread: failed to close mocap thread\n");
				}
				if (mocap_cleanup() < 0)
				{
					printf("WARNING in update_main_thread: failed to cleanup mocap thread\n");
				}
			}
		}

		if (mocap_thread_terminate_fl && mocap_thread.is_started())
		{
			if (mocap_thread.stop(MOCAP_TOUT) < 0)
			{
				printf("WARNING in update_main_thread: failed to close mocap thread\n");
			}
			if (mocap_initialized && mocap_cleanup() < 0)
			{
				printf("WARNING in update_main_thread: failed to cleanup mocap thread\n");
			}
			mocap_thread_active_fl = false;
		}
		
	} // MOCAP
	
	comms_manager_thread_check_fl = true;
	double tmp = finddt_s(benchmark_timers.tCOMMS);
	benchmark_timers.tCOMMS = rc_nanos_since_boot();
	if (tmp < 1.0 / COMMS_MANAGER_HZ)
	{
		rc_usleep(1000000*(1.0 / COMMS_MANAGER_HZ - tmp));
	}
	
	return 0;
}

/**
* @brief   Update mocap thread.
*
* @return -1 on error 0 on success.
*/
char comms_manager_t::update_mocap_thread(void)
{
	// set up mocap serial link
	if (mocap_thread_terminate_fl)
	{
		if (mocap_cleanup() < 0)
		{
			printf("ERROR in update_mocap_thread: failed to cleanup mocap\n");
			return -1;
		}
	}
	else
	{
		if (!mocap_initialized)
		{
			printf("update_mocap_thread: Initializing mocap serial link\n");
			if (unlikely(mocap_start(settings.serial_port_1, settings.serial_baud_1, &GS_RX, sizeof(GS_RX), &GS_TX, sizeof(GS_TX)) < 0))
			{
				printf("ERROR in update_mocap_thread: failed to init mocap serial link\n");
				return -1;
			}
		}
		

	}
	if (unlikely(mocap_update() < 0))
	{
		printf("ERROR in update_mocap_thread: failed to update mocap\n");
		mocap_thread_terminate_fl = true;
		return -1;
	}
	
	
	mocap_thread_check_fl = true; //always reset this to true at the end of the loop

	double tmp = finddt_s(benchmark_timers.tMOCAP);
	benchmark_timers.tMOCAP = rc_nanos_since_boot();
	if (tmp < 1.0 / MOCAP_HZ)
	{
		rc_usleep(1000000 * (1.0 / MOCAP_HZ - tmp));
	}

	return 0;
}

/**
* @brief   Enables mocap communication
*
* @return non-zero on error.
*/
char comms_manager_t::mocap_start(const char* port, const int baudRate, void* buff_RX, size_t size_buff_RX, void* buff_TX, size_t size_buff_TX)
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
	if (unlikely(mocap_transmit_line.en_low_latency_mode() < 0))
	{
		printf("ERROR in mocap_start: failed to enable low latency mode\n");
		return -1;
	}
	mocap_transmit_line.set_RX_line(buff_RX, size_buff_RX);
	mocap_transmit_line.set_TX_line(buff_TX, size_buff_TX);

	TX_time = rc_nanos_since_boot();
	mocap_initialized = true;
	mocap_active = false;

	return 0;
}


/**
	 * @brief   Updates mocap data tructure (setpoint).
	 *
	 * @return non-zero on error.
	 */
char comms_manager_t::mocap_save_data_sp(void)
{	
	GS_TX.x_sp = setpoint.X;
	GS_TX.y_sp = setpoint.Y;
	GS_TX.z_sp = setpoint.Z;
	GS_TX.x_dot_sp = setpoint.X_dot;
	GS_TX.y_dot_sp = setpoint.Y_dot;
	GS_TX.z_dot_sp = setpoint.Z_dot;
	
	
	GS_TX.roll_sp = setpoint.roll;
	GS_TX.pitch_sp = setpoint.pitch;
	GS_TX.yaw_sp = setpoint.yaw;
	
	GS_TX.roll_dot_sp = setpoint.roll_dot;
	GS_TX.pitch_dot_sp = setpoint.pitch_dot;
	GS_TX.yaw_dot_sp = setpoint.yaw_dot;

	return 0;
}

/**
 * @brief   Updates mocap data tructure (state).
 *
 * @return non-zero on error.
 */
char comms_manager_t::mocap_save_data_st(void)
{
	GS_TX.x = state_estimate.X;
	GS_TX.y = state_estimate.Y;
	GS_TX.z = state_estimate.Z;
	
	GS_TX.x_dot = state_estimate.X_dot;
	GS_TX.y_dot = state_estimate.Y_dot;
	GS_TX.z_dot = state_estimate.Z_dot;

	GS_TX.roll = state_estimate.roll;
	GS_TX.pitch = state_estimate.pitch;
	GS_TX.yaw = state_estimate.continuous_yaw;

	GS_TX.roll_dot = state_estimate.roll_dot;
	GS_TX.pitch_dot = state_estimate.pitch_dot;
	GS_TX.yaw_dot = state_estimate.yaw_dot;
	return 0;
}

/**
* @brief   Updates mocap data tructure (all).
*
* @return non-zero on error.
*/
char comms_manager_t::mocap_save_data(void)
{
	GS_TX.time = rc_nanos_since_boot();
	GS_TX.st_f = (uint8_t)user_input.flight_mode;
	GS_TX.st_SM = (uint8_t)waypoint_state_machine.get_current_state();


	mocap_save_data_st();
	mocap_save_data_sp();

	return 0;
}



/**
* @brief   Updates mocap related data.
*
* @return non-zero on error.
*/
char comms_manager_t::mocap_update(void)
{
	if (unlikely(!mocap_initialized))
	{
		printf("ERROR in mocap update: not initialized\n");
		return -3;
	}

	mocap_save_data();
	
	if (unlikely(mocap_transmit_line.read() < 0))
	{
		printf("ERROR in mocap_update: failed to read new data\n");
		return -2;
	}
	
	if (mocap_en_TX && finddt_s(TX_time) > 1.0 / MOCAP_THREAD_TX_HZ)
	{
		if (unlikely(mocap_transmit_line.write() < 0))
		{
			printf("ERROR in mocap_update: failed to write new data\n");
			mocap_en_TX = false;
			return -1;
		}
		if (unlikely(mocap_transmit_line.sync() < 0))
		{
			printf("ERROR in mocap_update: failed to sync");
			return -1;
		}
		TX_time = rc_nanos_since_boot();
	}
	return 0;
}

/**
* @brief   proper exit routine for mocap comm line.
*
* @return non-zero on error.
*/
char comms_manager_t::mocap_cleanup(void)
{
	if (unlikely(!mocap_initialized)) return 0;
	mocap_initialized = false;
	mocap_active = false;
	mocap_en_TX = false;

	mocap_transmit_line.close();
	return 0;
}

/**
* @brief   Updates all related data.
*
* @return non-zero on error.
*/
char comms_manager_t::update(void)
{
	if (unlikely(!initialized))
	{
		printf("ERROR in update: not initialized\n");
		return -1;
	}

	// check if the main thread is alive:
	if (!comms_manager_thread_terminate_fl)
	{
		if (comms_manager_thread_check_fl) comms_manager_thread_check_fl = false, comms_manager_thread_active_fl = true;
		else
		{
			comms_manager_thread_active_fl = false;
			if (finddt_s(benchmark_timers.tCOMMS) > 5.0 / COMMS_MANAGER_HZ)
			{
				printf("WARNING in update: comms_manager thread is not responsding, terminating...\n");
				comms_manager_thread_terminate_fl = true;
				mocap_thread_terminate_fl = true;
			}
		}
	}
	

	//Read from GPS sensor
	if (settings.enable_gps)
	{
		if (unlikely(gps_getData() < 0))
		{
			printf("ERROR in update: failed to update GPS\n");
			return -1;
		}
		else
		{
			if (settings.log_benchmark) benchmark_timers.tGPS = rc_nanos_since_boot();
		}		
	}

	return 0;
}

/**
* @brief   proper exit routine for all the functions.
*
* @return non-zero on error.
*/
char comms_manager_t::cleanup(void)
{
	if (settings.enable_mocap)
	{
		mocap_thread_terminate_fl = true;		
		if (mocap_thread.stop(MOCAP_TOUT) < 0)
		{
			printf("WARNING in cleanup: failed to close mocap thread\n");
		}
		if (mocap_cleanup() < 0)
		{
			printf("WARNING in cleanup: failed to cleanup mocap\n");
		}
	}

	initialized = 0;

	return 0;
}