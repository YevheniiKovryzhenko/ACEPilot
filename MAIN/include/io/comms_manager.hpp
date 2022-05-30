 /*
  * commms_manager.hpp
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
  * Last Edit:  05/29/2022 (MM/DD/YYYY)
  *
  * Object that governs all the high level logic related to communications.
  */

#ifndef COMMS_MANAGER
#define COMMS_MANAGER
//#include <serial_transmit.hpp>
#include <stdint.h>
#include "serial_transmit.hpp"
#include "comms_tmp_data_packet.h"
#include "thread_gen.hpp"

#define MOCAP_THREAD_TX_HZ 20
#define MOCAP_THREAD_TX_TOUT 1

class comms_manager_t
{
private:
	bool initialized;
	
	bool mocap_initialized;
	bool mocap_active;
	serial_transmit_t mocap_transmit_line;

	bool comms_manager_thread_active_fl;
	bool comms_manager_thread_check_fl;
	bool comms_manager_thread_terminate_fl;
	thread_gen_t comms_manager_thread;

	bool mocap_thread_active_fl;
	bool mocap_thread_check_fl;
	bool mocap_thread_terminate_fl;
	bool mocap_en_TX;
	uint64_t TX_time;
	thread_gen_t mocap_thread;

	/**
	 * @brief   proper exit routine for mocap comm line.
	 *
	 * @return -1 on error 0 on success.
	 */
	char mocap_cleanup(void);
	
	/**
	 * @brief   Updates mocap related data.
	 *
	 * @return -1 on error 0 on success.
	 */
	char mocap_update(void);

	/**
	 * @brief   Updates mocap data tructure (setpoint).
	 *
	 * @return -1 on error 0 on success.
	 */
	char mocap_save_data_sp(void);

	/**
	 * @brief   Updates mocap data tructure (state).
	 *
	 * @return -1 on error 0 on success.
	 */
	char mocap_save_data_st(void);

	/**
	* @brief   Updates mocap data tructure (all).
	*
	* @return -1 on error 0 on success.
	*/
	char mocap_save_data(void);

	/**
	 * @brief   Enables mocap communication
	 *
	 * @return -1 on error 0 on success.
	 */
	char mocap_start(const char* port, const int baudRate, void* buff_RX, size_t size_buff_RX, void* buff_TX, size_t size_buff_TX);

public:
	/**
	 * @brief   Initializes all related variables and objects.
	 *
	 * @return -1 on error 0 on success.
	 */
	char init(void);

	/**
	 * @brief   Updates funtions within main loop.
	 *
	 * @return -1 on error 0 on success.
	 */
	char update(void);
	
	/**
	 * @brief   Update main thread.
	 *
	 * @return -1 on error 0 on success.
	 */
	char update_main_thread(void);

	/**
	 * @brief   Update mocap thread.
	 *
	 * @return -1 on error 0 on success.
	 */
	char update_mocap_thread(void);
	
	
	/**
	 * @brief   proper exit routine for all the functions.
	 *
	 * @return -1 on error 0 on success.
	 */
	char cleanup(void);
};

extern comms_manager_t comms_manager;
#endif // COMMS_MANAGER

