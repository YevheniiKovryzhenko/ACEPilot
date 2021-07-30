/*
 * AdafruitGPS_cmds.hpp
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
 */

#ifndef ADAFRUITGPS_CMNDS_HPP
#define ADAFRUITGPS_CMNDS_HPP

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <iostream>
#include <cstdlib>
#include "serial_com.h"
using namespace std;

class AdafruitGPS {        // Class for controlling Adafruit Ultimate GPS breakout board
private:              // Access specifier
    //This sets up variables for useful commands.
    //This set is used to set the rate the GPS reports
    std::string UPDATE_10_sec = "$PMTK220,10000*2F\r\n"; //#Update Every 10 Seconds
    std::string UPDATE_5_sec = "$PMTK220,5000*1B\r\n";   //Update Every 5 Seconds
    std::string UPDATE_1_sec = "$PMTK220,1000*1F\r\n";   //Update Every One Second
    std::string UPDATE_200_msec = "$PMTK220,200*2C\r\n"; //Update Every 200 Milliseconds
    //This set is used to set the rate the GPS takes measurements
    std::string MEAS_10_sec = "$PMTK300,10000,0,0,0,0*2C\r\n"; //Measure every 10 seconds
    std::string MEAS_5_sec = "$PMTK300,5000,0,0,0,0*18\r\n";   //Measure every 5 seconds
    std::string MEAS_1_sec = "$PMTK300,1000,0,0,0,0*1C\r\n";   //Measure once a second
    std::string MEAS_200_msec = "$PMTK300,200,0,0,0,0*2F\r\n";  //Meaure 5 times a second
    //Set the Baud Rate of GPS
    std::string SET_BAUD_57600 = "$PMTK251,57600*2C\r\n";          //Set Baud Rate at 57600
    std::string SET_BAUD_9600 = "$PMTK251,9600*17\r\n";             //Set 9600 Baud Rate
    //Commands for which NMEA Sentences are sent
    std::string GPRMC_ONLY = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"; //Send only the GPRMC Sentence
    std::string GPRMC_GPGGA = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"; //Send GPRMC AND GPGGA Sentences
    std::string SEND_ALL = "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"; //Send All Sentences
    std::string SEND_NOTHING = "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"; //Send Nothing

    int device_ID; //save GPS COM port internally just in case
    int initialized = 0;

public:
    void test_print();
    int set_baud_57600();
    int set_baud_9600();
    int set_measure_200msec();
    int set_update_200msec();
    int set_measure_1sec();
    int set_update_1sec();
    int init_GPS(int device);
    int send_all();
    int send_none();
    int send_GPRMC_ONLY();
    int send_GPRMC_GPGGA();
};

extern AdafruitGPS GPS;     // Create an object of MyClass

#endif // !ADAFRUITGPS_CMNDS_HPP