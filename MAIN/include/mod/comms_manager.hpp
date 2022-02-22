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
  * Last Edit:  02/22/2022 (MM/DD/YYYY)
  *
  * Object that governs all the high level logic related to communications.
  */

#ifndef COMMS_MANAGER
#define COMMS_MANAGER
//#include <serial_transmit.hpp>
#include <stdint.h>
#include "serial_transmit.hpp"
#include "comms_tmp_data_packet.h"

class comms_manager_t
{
private:
	bool initialized;
	
	bool mocap_initialized;
	bool mocap_active;
	serial_transmit_t mocap_transmit_line;
	uint64_t mocap_time;
public:
	char init(void);
	
	char mocap_start(const char* port, const int baudRate);
	char mocap_cleanup(void);

	char cleanup(void);
};
#endif // COMMS_MANAGER

