/*
 * gps.c
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

#include "gps.h"

int gps_portID;
/* GPS handle */
lwgps_t hgps;

/* GPS buffer */
lwrb_t hgps_buff;
uint8_t hgps_buff_data[12];

/* Conversions */
#define knot2ms 1.9438444924406     ///< knot to meter/second conversion factor

gps_data_t gps_data;

int
gps_init(const char* GPS_Serial, int GPS_baudRate)
{
    gps_portID = serial_open(GPS_Serial, GPS_baudRate, 0);
    //gps_portID = serial_open("/dev/ttyUSB1", 9600, 0);
    if (gps_portID == -1)
    {
        printf("\nFailed to open Serial Port");
        return -1;
    }

    coordinates_init();
    // Set gps data to intialilly be invalid
    gps_data.gps_valid = 0;

    /* Init GPS Librray */
    lwgps_init(&hgps);

    /* Create buffer for received data */
    lwrb_init(&hgps_buff, hgps_buff_data, sizeof(hgps_buff_data));

    return gps_portID;
}



int gps_getData()
{
    char buffer;
    uint8_t rx;

    if (read(gps_portID, &buffer, 1) > 0)
    {
        //printf("\nByte: %x Char: %c\n", buffer, buffer);

        /* Write to buffer only */
        lwrb_write(&hgps_buff, &buffer, 1);
        //return 0;
    }

    /* Process all input data */
    /* Read from buffer byte-by-byte and call processing function */
    if (lwrb_get_full(&hgps_buff)) {        /* Check if anything in buffer now */
        while (lwrb_read(&hgps_buff, &rx, 1) == 1) {
            //printf("\nReading from buffer...");
            lwgps_process(&hgps, &rx, 1);   /* Process byte-by-byte */
        }
    }
    else {
        // Write data to the main GPS structure 
        gps_data.lla.alt    = hgps.altitude;
        gps_data.lla.lat    = hgps.latitude;
        gps_data.lla.lon    = hgps.longitude;

        gps_data.ned        = lla2ned(&gps_data.lla);
        gps_data.spd        = hgps.speed * knot2ms;     ///< speed in m/s
        gps_data.fix        = hgps.fix;                 ///< fix type
        gps_data.sat        = hgps.sats_in_use;         ///< number of satellites
        //gps_data.headingNc  = 0.0;                      ///< heading (not tilt compensated) in degrees from compass
        gps_data.cog        = hgps.course;              ///< course over ground
        //gps_data.gpsVsi     = 0.0;                      ///< vertical speed indicator (from GPS) in m/s (a.k.a. climb speed)
        gps_data.hdop       = hgps.dop_h;               ///< horizontal dilution of precision
        gps_data.vdop       = hgps.dop_v;               ///< vertical dilution of precision
        gps_data.year       = hgps.year;
        gps_data.month      = hgps.month;
        gps_data.day        = hgps.date;
        gps_data.hour       = hgps.hours;
        gps_data.minute     = hgps.minutes;
        gps_data.second     = hgps.seconds;
        gps_data.gps_update_Hz          = 1.0/finddt_s(gps_data.gps_data_received_ns);
        gps_data.gps_data_received_ns   = rc_nanos_since_boot();  ///< time on beaglebond board that gps is received
        gps_data.gps_valid              = hgps.is_valid;
        if (gps_data.gps_valid)
        {
            gps_data.gps_last_time_valid_ns = rc_nanos_since_boot(); ///< time on beaglebond board when a valid gps signal was received
        }

        /* Print GPS data */
        gps_PrintData();
    }

    //--------------------------------//

    if ((gps_data.gps_valid && gps_data.fix >= FIX_2D) || gps_data.fix == FIX_DGPS)
    {
        if (!origin.initialized)
        {
            set_origin(&gps_data.lla);
            origin.initialized = 1;
        }
        return 0;
    }

    return -1;
}

int gps_PrintData()
{
    /* Print all data after successful processing */
    if (gps_data.fix)
    {
        printf("\n\nFix: %d", gps_data.fix);
        printf("\nYear/Month/Day of Fix: %d/ %d / %d", gps_data.year, gps_data.month, gps_data.day);
        printf("\nCurrent Time: %d:%d:%d", gps_data.hour, gps_data.minute, gps_data.second);
        printf("\nSats in use: %d", gps_data.sat);
        printf("\nLatitude: %f deg", gps_data.lla.lat);
        printf("\nLongitude: %f deg", gps_data.lla.lon);
        printf("\nAltitude: %f m", gps_data.lla.alt);
        printf("\nGround Speed: %f m/s", gps_data.spd);
        printf("\nCourse over ground: %f deg", gps_data.cog);
        printf("\nHeading from Mag: %f deg", gps_data.headingNc);
        //printf("\nVertical speed from?: %f deg", gps_data.gpsVsi);
        printf("\nLocation in NED frame relative to origin: x=%f y=%f z=%f m", gps_data.ned.x, gps_data.ned.y, gps_data.ned.z);
        printf("\nOrigin location in ECEF frame: x=%f y=%f z=%f m", origin.ecef.x, origin.ecef.y, origin.ecef.z);
    }
    else
    {
        printf("\n\nFix: %d", gps_data.fix);
        printf("\nCurrent Time: %d:%d:%d", gps_data.hour, gps_data.minute, gps_data.second);
    }
    printf("\nUpdate frequency: %f Hz", gps_data.gps_update_Hz);
    printf("\nTime since last valid update: %f s", finddt_s(gps_data.gps_last_time_valid_ns));
    return 0;
}

int gps_cleanup()
{
    serial_close(gps_portID);

    return 0;
}