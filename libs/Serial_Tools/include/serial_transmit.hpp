/*
 * serial_transmit.hpp
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
 * Last Edit:  02/21/2022 (MM/DD/YYYY)
 *
 * Summary :
 * This code is intended for simple serial communication using pre-defined start bytes and
 * end bytes with the main message (data) in between. Currently uses fletcher16 checksum,
 * no payload size and fixed start bytes. Here's the message from the original code:
 *
 * Using data structure for packets with:
 * Two start bytes:  0x81, 0xA1
 * [Not included: Message ID (one byte), Message payload size (one byte) since we only have one message type]
 * Message data (xbee_packet_t length)
 * Fletcher-16 checksum (two bytes) computed starting with Message payload size
 *
 * Note:  This MBin protocol is commonly used on embedded serial devices subject to errors
 *
 */


#ifndef SERIAL_TRANSMIT
#define SERIAL_TRANSMIT

#include <stdio.h>
#include <stdlib.h>	//one of these two is for memcpy
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <iostream>
#include <vector>

#include "crc16.h"
#include "serialib.h"

#define NUM_START_BYTES 2
#define NUM_END_BYTES 2
#define RING_BUFSIZE 256
#define DEF_START_BYTE_1 0x81
#define DEF_START_BYTE_2 0xA1

//#define SERIAL_TRASMIT_DEBUG //uncomment for debugging output

using namespace std;
class serial_transmit_t
{
private:
	//Transmit line:
	void* TX_data_buff;
	bool TX_en_checksum;
	bool TX_initialized;
	size_t TX_data_size;
	size_t TX_packet_size;
	int TX_num_start_bytes;
	vector<uint8_t> TX_start_bytes;
	vector<unsigned char> TX_packet;
	vector<unsigned char> send_data;

	//Receive line:
	void* RX_data_buff;
	bool RX_en_checksum;
	bool RX_initialized;
	size_t RX_data_size;
	size_t RX_packet_size;
	int RX_num_start_bytes;
	vector<uint8_t> RX_start_bytes;
	bool ring_overflow;
	int rdIndex;
	int wrIndex;
	unsigned char ringbuffer[RING_BUFSIZE];
	vector<unsigned char> receive_data; //temporary storrage for received binary data
	vector<uint8_t> ck_bytes; //checksum bytes
	unsigned char msgState;
	unsigned char msglength;
	unsigned char start_byte_counter;
	unsigned char end_byte_counter;

	int ring_inc(int a);  // Increment ring buffer index
	void readRingBuffer(void);
	
	//Serial Port:
	bool opened;
	serialib serial;
public:
	char open(const char* port, const int baudRate);
	
	char set_TX_line(void* buff, size_t buff_size);
	void enable_checksum_TX(void);
	void disable_checksum_TX(void);
	char set_TX_start_bytes(uint8_t* buff, int num_of_bytes);
	char write(void* data, size_t buff_size);
	char write(void);

	char set_RX_line(void* buff, size_t buff_size);
	void enable_checksum_RX(void);
	void disable_checksum_RX(void);
	char read_bytes(unsigned char* buff, size_t buff_size);
	char set_RX_start_bytes(uint8_t* buff, int num_of_bytes);
	char read(void* buff, size_t buff_size);
	char read(void);

	char sync(void);
	
	
	void close(void);
};
#endif // SERIAL_TRANSMIT
