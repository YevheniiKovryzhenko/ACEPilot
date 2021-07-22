/*
 * gps.h
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

#ifndef __GPS__
#define __GPS__

#include <stdint.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <rc/time.h>
#include "serial_com.h"

#include "lwgps.h"
#include "lwrb.h"
#include "coordinates.h"
#include "tools.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief   Gps fix type
 */
typedef enum
{
    NO_FIX = 0,
    FIX_2D = 1,
    FIX_DGPS = 2
} fixType_t;

/**
 *  @brief      gps data structure
 */
typedef struct gps_data_t
{
    lla_t lla;         ///< Lattitude longitude altitude coordinates
    ned_waypoint_t ned;
    double spd;        ///< speed in m/s
    fixType_t fix;     ///< fix type
    uint8_t sat;       ///< number of satellites
    double headingNc;  ///< heading (not tilt compensated) in degrees
    double cog;        ///< course over ground
    double gpsVsi;     ///< vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
    double hdop;       ///< horizontal dilution of precision
    double vdop;       ///< vertical dilution of precision
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint64_t gps_data_received_ns;  ///< time on beaglebond board that gps data is processed
    uint64_t gps_last_time_valid_ns;///< time on beaglebond board when a valid gps signal was received
    double gps_update_Hz;           ///< update frequency of updates from hardware (all, not just valid)
    uint8_t gps_valid;
} gps_data_t;

extern gps_data_t gps_data;
//extern lwgps_t hgps;

/**
 * @brief   Set up serial connection to the naza gps unit
 *
 * The serial port is defined in the code to be the GPS (uart2) port on the beaglebone
 *
 * @return  0 on success, -1 on failure (not good at detecting failure)
 */
int 
gps_init(const char* GPS_Serial, int GPS_baudRate);
//int gps_init();

/**
 * @brief   attempt to extract data from naza gps
 *
 * Function attempts to read one byte at a time from the naza gps unit
 * until all bytes are read.  The data is parsed one byte at a time.
 *
 * @return  0 on success, -1 on failure
 */
int gps_getData();


int gps_PrintData();

int gps_cleanup();
#ifdef __cplusplus
}
#endif

#endif /* __GPS__ */

/* @} end group Gps */