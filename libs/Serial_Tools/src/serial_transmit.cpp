/*
 * serial_transmit.cpp
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

#include "serial_transmit.hpp"

int serial_transmit_t::ring_inc(int a)
{
    return (a < (RING_BUFSIZE - 1)) ? (a + 1) : 0;
}

char serial_transmit_t::open(const char *port, const int baudRate)
{
    if (opened)
    {
        printf("ERROR: trying to open receive line when already opened\n");
        return 0;
    }
    if (serial.openDevice(port, baudRate) != 1)
    {
        printf("Failed to open Serial Port\n");
        return -1;
    }
    serial.flushReceiver();

    opened = true;
    TX_en_checksum = true;
    RX_en_checksum = true;
    return 0;
}

void serial_transmit_t::enable_checksum_TX(void)
{
    TX_en_checksum = true;
    return;
}

void serial_transmit_t::disable_checksum_TX(void)
{
    TX_en_checksum = false;
    return;
}

char serial_transmit_t::set_TX_start_bytes(uint8_t* buff, int num_of_bytes)
{
    if (num_of_bytes < 1)
    {
        printf("ERROR: Buffer size must be positive\n");
        return -1;
    }
    if (!TX_initialized)
    {
        printf("ERROR: must initialize first\n");
        return -1;
    }

    TX_num_start_bytes = num_of_bytes;
    TX_start_bytes.clear();
    TX_start_bytes.reserve(TX_num_start_bytes);

    TX_packet.clear();
    TX_packet.reserve(TX_packet_size);

    for (int i = 0; i < TX_num_start_bytes; i++)
    {
        TX_start_bytes.push_back(*buff);
        TX_packet.push_back(TX_start_bytes[i]);

        buff++;
    }

    TX_packet_size = sizeof(uint8_t) * TX_start_bytes.size() + TX_data_size + NUM_END_BYTES;

    return 0;
}

char serial_transmit_t::set_TX_line(void* buff, size_t buff_size)
{
    if (buff_size < 1)
    {
        printf("ERROR: Buffer size must be positive\n");
        TX_initialized = false;
        return -1;
    }
    
    //Receive line:
    TX_data_buff = buff;
    TX_data_size = buff_size;
    
    if (!TX_initialized)
    {
        TX_num_start_bytes = NUM_START_BYTES;
        TX_start_bytes.clear();
        TX_start_bytes.reserve(TX_num_start_bytes);
        TX_start_bytes.push_back(DEF_START_BYTE_1);
        TX_start_bytes.push_back(DEF_START_BYTE_2);
    }
    
    TX_packet_size = sizeof(uint8_t)*TX_start_bytes.size() + TX_data_size + NUM_END_BYTES;

    if (!TX_initialized)
    {
        TX_packet.clear();
        TX_packet.reserve(TX_packet_size);
        for (int i = 0; i < TX_num_start_bytes; i++)
        {
            TX_packet.push_back(TX_start_bytes[i]);
        }
    }    

    TX_initialized = true;
    return 0;
}

void serial_transmit_t::enable_checksum_RX(void)
{
    RX_en_checksum = true;
    return;
}

void serial_transmit_t::disable_checksum_RX(void)
{
    RX_en_checksum = false;
    return;
}

char serial_transmit_t::set_RX_start_bytes(uint8_t* buff, int num_of_bytes)
{
    if (num_of_bytes < 1)
    {
        printf("ERROR: Buffer size must be positive\n");
        return -1;
    }
    if (!RX_initialized)
    {
        printf("ERROR: must initialize first\n");
        return -1;
    }

    RX_num_start_bytes = num_of_bytes;
    RX_start_bytes.clear();
    RX_start_bytes.reserve(RX_num_start_bytes);
    for (int i = 0; i < RX_num_start_bytes; i++)
    {
        RX_start_bytes.push_back(*buff);
        buff++;
    }

    RX_packet_size = sizeof(uint8_t) * RX_start_bytes.size() + RX_data_size + NUM_END_BYTES;

    return 0;
}

char serial_transmit_t::set_RX_line(void* buff, size_t buff_size)
{
    if (buff_size < 1)
    {
        printf("ERROR: Buffer size must be positive\n");
        RX_initialized = false;
        return -1;
    }


    //Receive line:
    RX_data_buff = buff;
    RX_data_size = buff_size;

    RX_num_start_bytes = NUM_START_BYTES;
    RX_start_bytes.clear();
    RX_start_bytes.reserve(RX_num_start_bytes);
    RX_start_bytes.push_back(DEF_START_BYTE_1);
    RX_start_bytes.push_back(DEF_START_BYTE_2);
    ring_overflow = 0;
    rdIndex = 0;
    wrIndex = 0;
    msgState = 0;
    msglength = 0;

    RX_packet_size = sizeof(uint8_t) * RX_start_bytes.size() + RX_data_size + NUM_END_BYTES;

    receive_data.reserve(RX_data_size);
    ck_bytes.reserve(NUM_END_BYTES);

    RX_initialized = true;
    return 0;
}

char serial_transmit_t::write(void)
{
    if (!opened || !TX_initialized)
    {
        printf("ERROR: trying to write to a serial before initialization\n");
        return -1;
    }
    memcpy(TX_packet.data()+TX_num_start_bytes, TX_data_buff, TX_data_size);
    if (TX_en_checksum)
    {
        fletcher16_append(TX_packet.data() + TX_num_start_bytes, TX_data_size, TX_packet.data() + TX_data_size + TX_num_start_bytes);
        serial.writeBytes(TX_packet.data(), TX_packet_size);
    }
    else
    {
        serial.writeBytes(TX_packet.data(), TX_data_size + TX_num_start_bytes);
    }
    
    return 0;
}
char serial_transmit_t::write(void* buff, size_t buff_size)
{
    if (!opened || !TX_initialized)
    {
        printf("ERROR: trying to write to a serial before initialization\n");
        return -1;
    }
    TX_data_buff = buff;
    TX_data_size = buff_size;
    TX_packet_size = sizeof(uint8_t) * TX_start_bytes.size() + TX_data_size + NUM_END_BYTES;
    return write();
}

char serial_transmit_t::sync(void)
{
    if (!opened || !TX_initialized)
    {
        printf("ERROR: trying to write to a serial before initialization\n");
        return -1;
    }
    serial.sync();
    return 0;
}

// Read message received; use ring buffer to assure no data loss
char serial_transmit_t::read(void)
{
    if (!opened || !RX_initialized)
    {
        printf("ERROR: trying to read from serial before initialization\n");
        return -1;
    }
    int k;
    unsigned char buffer;

    // Populate ring buffer
    for (k = 0; k < RING_BUFSIZE; k++) {
        if (ring_overflow) { // Overflow condition on last read attempt
            if (rdIndex == wrIndex) { // Indices equal:  overflow still in effect
                printf("ERROR: data overflow\n");
                return -1;  // Do not read data; reading data would cause overflow
                //break;
            }
            else
                ring_overflow = false;  // Reset overflow flag
        }
        if (serial.available() > 0) { // Read 1 byte from serial port into buffer: returns number of bytes available
            serial.readBytes(&buffer, 1);
            ringbuffer[wrIndex] = buffer;
            wrIndex = ring_inc(wrIndex);
            if (wrIndex == rdIndex) ring_overflow = true;
        }
        else
            break;
    }

    // Read and process data in ring buffer
    readRingBuffer();

    return 0;
}

// Read message received directly; use ring buffer (if stored any data) or directly read from the serial
char serial_transmit_t::read_bytes(unsigned char* buff,size_t buff_size)
{
    if (!opened)
    {
        printf("ERROR: trying to read from serial before initialization\n");
        return -1;
    }
    for (int i = 0; i < buff_size; i++)
    {
        if (rdIndex < wrIndex) //we have stored some bytes in the ring buffer
        {
            *buff = ringbuffer[rdIndex];
            buff++;
            rdIndex = ring_inc(rdIndex);
        }
        else if (serial.available() > 0)
        {
            serial.readBytes(buff, 1);
            buff++;
        }
        else
        {
            printf("ERROR: no bytes available to read\n");
            return -1;
        }
    }
        

    return 0;
}

char serial_transmit_t::read(void* buff, size_t buff_size)
{
    if (!opened || !RX_initialized)
    {
        printf("ERROR: trying to read from a serial before initialization\n");
        return -1;
    }
    RX_data_buff = buff;
    RX_data_size = buff_size;
    RX_packet_size = sizeof(uint8_t) * RX_start_bytes.size() + RX_data_size + NUM_END_BYTES;
    return read();
}


void serial_transmit_t::readRingBuffer(void)
{ 

  while(ring_overflow || (rdIndex != wrIndex)) { //Don't get ahead of the receiving data 
    if (ring_overflow) ring_overflow = false; // Reset buffer overflow flag
#ifdef SERIAL_TRASMIT_DEBUG
    if (msgState == 0)
    {
        printf("SERIAL_TRASMIT_DEBUG: Waiting for start byte...\n");
        printf("SERIAL_TRASMIT_DEBUG: Read #: %i\t Byte: %X\t Start Byte: %X\n", rdIndex, ringbuffer[rdIndex], RX_start_bytes[0]);
    }
#endif // SERIAL_TRASMIT_DEBUG

    // Case 0:  Current character is first message header byte
    if((ringbuffer[rdIndex] == RX_start_bytes[0]) && !msgState) {
#ifdef SERIAL_TRASMIT_DEBUG
        printf("\nSERIAL_TRASMIT_DEBUG: Received Start byte!\n"); //test if serial is working and receiving start byte
        printf("SERIAL_TRASMIT_DEBUG: Read index\t Byte #\t Byte val\t Start Byte\n");
#endif // SERIAL_TRASMIT_DEBUG

        if (RX_num_start_bytes > 1) msgState = 1;
        else msgState = 2;
        start_byte_counter = 1;
        end_byte_counter = 0;
        msglength = 0;
    }
     
    // Case 1:  Current character is second message header byte
    else if (msgState == 1) {
#ifdef SERIAL_TRASMIT_DEBUG
        printf("SERIAL_TRASMIT_DEBUG: %i\t\t %i/%i\t\t %X\t\t %X\n", rdIndex, start_byte_counter + 1, RX_num_start_bytes, ringbuffer[rdIndex], RX_start_bytes[start_byte_counter]);
#endif // SERIAL_TRASMIT_DEBUG

        if (ringbuffer[rdIndex] == RX_start_bytes[start_byte_counter])
        {
#ifdef SERIAL_TRASMIT_DEBUG
            printf("SERIAL_TRASMIT_DEBUG: matched %i/%i start bytes, start reading the data...\n", start_byte_counter+1, RX_num_start_bytes);
#endif // SERIAL_TRASMIT_DEBUG

            start_byte_counter++;
            if (RX_num_start_bytes <= start_byte_counter)
            {
#ifdef SERIAL_TRASMIT_DEBUG
                printf("SERIAL_TRASMIT_DEBUG: Read index\t Byte #\t Byte val\n");
#endif // SERIAL_TRASMIT_DEBUG

                start_byte_counter = 0;
                end_byte_counter = 0;
                msgState = 2;
                receive_data.clear();
            }
        }
        else
        {
#ifdef SERIAL_TRASMIT_DEBUG
            printf("\n SERIAL_TRASMIT_DEBUG: Second start byte did not match! Discarding the message... \n");
#endif // SERIAL_TRASMIT_DEBUG

            
            msgState = 0;  // Bad message
            start_byte_counter = 0;
            end_byte_counter = 0;
        }
        msglength = 0;  // Initialize number of message payload data bytes read thusfar
    }
       
    // Case 2:  Read data bytes into receive_packet[] array
    else if (msgState == 2) {
        receive_data[msglength++] = ringbuffer[rdIndex];
#ifdef SERIAL_TRASMIT_DEBUG
        
        printf("SERIAL_TRASMIT_DEBUG: %i\t\t %i/%i\t\t %X\n", rdIndex, msglength, RX_data_size, ringbuffer[rdIndex]);
#endif // SERIAL_TRASMIT_DEBUG

        if (msglength == RX_data_size) {
            end_byte_counter = 0;
            if (RX_en_checksum)
            {
#ifdef SERIAL_TRASMIT_DEBUG
                printf("SERIAL_TRASMIT_DEBUG: done reading the message, calculating checksum...\n");
                printf("SERIAL_TRASMIT_DEBUG: Read index\t Byte #\t Byte val\t checksum\n");
#endif // SERIAL_TRASMIT_DEBUG

                msgState = 3; // Done reading data
                uint16_t tmp = fletcher16(receive_data.data(), RX_data_size);
                for (int j = 0; j < NUM_END_BYTES; j++)
                {
                    ck_bytes[j] = ((uint16_t)tmp >> (8 * j)) & 0xFF;  // shift by 0 not needed, of course, just stylistic
                }
            }
            else
            {
#ifdef SERIAL_TRASMIT_DEBUG
                printf("SERIAL_TRASMIT_DEBUG: no checksum option, so finished reading the message!\n");
#endif // SERIAL_TRASMIT_DEBUG

                //no checksum, so finished
                msgState = 0;
                end_byte_counter = 0;
                start_byte_counter = 0;
                memcpy(RX_data_buff, receive_data.data(), RX_data_size);
            }
            
        }
    }
    
    // Cases 3:  Read & check the checksum bytes
    // Throw away data if full checksum doesn't match
    else if (msgState == 3) {
#ifdef SERIAL_TRASMIT_DEBUG
        printf("SERIAL_TRASMIT_DEBUG: %i\t\t %i/%i\t\t %X\t\t %X\n", rdIndex, end_byte_counter+1, NUM_END_BYTES, ringbuffer[rdIndex], ck_bytes[end_byte_counter]);
        if (ck_bytes[end_byte_counter] != ringbuffer[rdIndex]) printf("SERIAL_TRASMIT_DEBUG: failed to match %i checksum byte\n", end_byte_counter);
#endif // SERIAL_TRASMIT_DEBUG

        if (ck_bytes[end_byte_counter] != ringbuffer[rdIndex]) msgState = 0;
        else
        {
            end_byte_counter++;
            if (NUM_END_BYTES <= end_byte_counter)
            {
#ifdef SERIAL_TRASMIT_DEBUG
                printf("SERIAL_TRASMIT_DEBUG: Checksum matched, accepting message!\n");
#endif // SERIAL_TRASMIT_DEBUG
                msgState = 0;
                end_byte_counter = 0;
                start_byte_counter = 0;
                memcpy(RX_data_buff, receive_data.data(), RX_data_size);
            }
        }
    }
    rdIndex = ring_inc(rdIndex);
  }  
  return;
}

void serial_transmit_t::close(void)
{
    if (opened)
    {
        if (serial.isDeviceOpen()) serial.closeDevice();
        opened = false;
    }
    else
    {
        return;
    }
    return;
}





