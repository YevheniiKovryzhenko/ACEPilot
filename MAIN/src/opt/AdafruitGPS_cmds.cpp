/*
 * AdafruitGPS_cmds.hpp
 *
 * Copyright Yevhenii Kovryzhenko, Department of Aerospace Engineering, Auburn University.
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
 */

#include "AdafruitGPS_cmds.hpp"

AdafruitGPS GPS;     // Create an object of MyClass

void AdafruitGPS::test_print() 
{  // Method/function defined inside the class
	std::cout << endl << "TEST print: " << UPDATE_10_sec;
}

int AdafruitGPS::set_baud_57600()
{
    if (initialized == 0)
    {
        printf("\n ERROR, GPS not initilized, run init_GPS(int device) first");
    }
    if (write(device_ID, SET_BAUD_57600.c_str(), SET_BAUD_57600.size())) //send request to change baudare of the GPS
    {
        printf("\n Setting GPS baudrate to 57600");
        sleep(1);
        serial_set_baud(device_ID, 57600); //change receiving serial baud
        return 1;
    }
    else
    {
        printf("\n Error Setting GPS baudrate to 57600");
        return -1;
    }
}
int AdafruitGPS::set_baud_9600()
{
    if (initialized == 0)
    {
        printf("\n ERROR, GPS not initilized, run init_GPS(int device) first");
    }
    if (write(device_ID, SET_BAUD_9600.c_str(), SET_BAUD_9600.size())) //send request to change baudare of the GPS
    {
        printf("\n Setting GPS baudrate to 9600");
        sleep(1);
        serial_set_baud(device_ID, 9600); //change receiving serial baud
        return 1;
    }
    else
    {
        std::cout << "\n Error Setting GPS baudrate to 9600";
        return -1;
    }
}

int AdafruitGPS::set_measure_200msec()
{
    if (initialized == 0)
    {
        printf("\n ERROR, GPS not initilized, run init_GPS(int device) first");
    }
    if (write(device_ID, MEAS_200_msec.c_str(), MEAS_200_msec.size())) //send request to change measurement rate of the GPS
    {
        printf("\n Setting GPS measurement rate to 10Hz");
        sleep(1);
        return 1;
    }
    else
    {
        printf("\n Error Setting GPS measurement rate to 10Hz");
        return -1;
    }
}

int AdafruitGPS::set_update_200msec()
{
    if (initialized == 0)
    {
        printf("\n ERROR, GPS not initilized, run init_GPS(int device) first");
    }
    if (write(device_ID, UPDATE_200_msec.c_str(), UPDATE_200_msec.size())) //send request to change update rate of the GPS
    {
        printf("\n Setting GPS update rate to 10Hz");
        sleep(1);
        return 1;
    }
    else
    {
        printf("\n Error Setting GPS update rate to 10Hz");
        return -1;
    }
}

int AdafruitGPS::set_measure_1sec()
{
    if (initialized == 0)
    {
        printf("\n ERROR, GPS not initilized, run init_GPS(int device) first");
    }
    if (write(device_ID, MEAS_1_sec.c_str(), MEAS_1_sec.size())) //send request to change measurement rate of the GPS
    {
        printf("\n Setting GPS measurement rate to 1Hz");
        sleep(1);
        return 1;
    }
    else
    {
        printf("\n Error Setting GPS measurement rate to 1Hz");
        return -1;
    }
}

int AdafruitGPS::set_update_1sec()
{
    if (initialized == 0)
    {
        printf("\n ERROR, GPS not initilized, run init_GPS(int device) first");
    }
    if (write(device_ID, UPDATE_1_sec.c_str(), UPDATE_1_sec.size())) //send request to change update rate of the GPS
    {
        printf("\n Setting GPS update rate to 1Hz");
        sleep(1);
        return 1;
    }
    else
    {
        printf("\n Error Setting GPS update rate to 1Hz");
        return -1;
    }
}

int AdafruitGPS::init_GPS(int device)
{
    device_ID = device;
    initialized = 1;
    set_baud_57600();
    set_measure_200msec();
    set_update_200msec();
    return 1;
}

int AdafruitGPS::send_all()
{
    if (initialized == 0)
    {
        printf("\n ERROR, GPS not initilized, run init_GPS(int device) first");
    }
    if (write(device_ID, SEND_ALL.c_str(), SEND_ALL.size())) //send request to change update rate of the GPS
    {
        sleep(1);
    }
    else
    {
        return -1;
    }
    return 1;
}

int AdafruitGPS::send_none()
{
    if (initialized == 0)
    {
        printf("\n ERROR, GPS not initilized, run init_GPS(int device) first");
    }
    if (write(device_ID, SEND_NOTHING.c_str(), SEND_NOTHING.size())) //send request to change update rate of the GPS
    {
        sleep(1);
    }
    else
    {
        return -1;
    }
    return 1;
}

int AdafruitGPS::send_GPRMC_ONLY()
{
    if (initialized == 0)
    {
        printf("\n ERROR, GPS not initilized, run init_GPS(int device) first");
    }
    if (write(device_ID, GPRMC_ONLY.c_str(), GPRMC_ONLY.size())) //send request to change update rate of the GPS
    {
        sleep(1);
    }
    else
    {
        return -1;
    }
    return 1;
}

int AdafruitGPS::send_GPRMC_GPGGA()
{
    if (initialized == 0)
    {
        printf("\n ERROR, GPS not initilized, run init_GPS(int device) first");
    }
    if (write(device_ID, GPRMC_GPGGA.c_str(), GPRMC_GPGGA.size())) //send request to change update rate of the GPS
    {
        sleep(1);
    }
    else
    {
        return -1;
    }
    return 1;
}