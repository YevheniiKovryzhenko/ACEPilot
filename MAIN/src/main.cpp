/*
 * main.cpp
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

#include "main.hpp"

using namespace std;
struct stat buffer;
//uint8_t RUNNING_ON_BBB = stat("/sys/devices/platform/leds/leds/",&buffer); //check if running on BBB
bool RUNNING_ON_BBB = (stat("/sys/devices/platform/leds/leds/", &buffer) == 0 && S_ISDIR(buffer.st_mode));

/**
 *  @brief      Standard exit for initialization failures
 */
#define FAIL(str)                       \
    fprintf(stderr, str);               \
    rc_led_set(RC_LED_GREEN, 0);        \
    rc_led_blink(RC_LED_RED, 8.0, 2.0); \
    return -1;


uint64_t t_init     = rc_nanos_since_boot();

int
main() {

    // start with both LEDs off
    if (RUNNING_ON_BBB)
    {
        if (rc_led_set(RC_LED_GREEN, 0) == -1)
        {
            fprintf(stderr, "\nERROR in main(), failed to set RC_LED_GREEN");
            return -1;
        }
        if (rc_led_set(RC_LED_RED, 0) == -1)
        {
            fprintf(stderr, "\nERROR in main() failed to set RC_LED_RED");
            return -1;
        }
    }

    /* Init GPS */
    /*
    printf("\nInitializing gps serial link");
    int portID = gps_init("/dev/ttyS11", 57600);
    if (RUNNING_ON_BBB == 1)
    {
        if (portID == -1) FAIL("\nTerminating...");
    }
    else
    {
        if (portID == -1) return -1;
    }

    //GPS.init_GPS(portID);
    //GPS.send_all();
    */

    if (RUNNING_ON_BBB)
    {
        if (ss.init(1, 0x40) == -1)
        {
            printf("\nERROR: failed to initialize servos");
            return -1;
        }
    }
    
    if (RUNNING_ON_BBB)
    {
        if (ss.test_min_max() == -1)
        {
            printf("\nERROR: failed to run servo test");
            return -1;
        }
    }

    /*
    while (finddt_s(t_init) <= 20.0)
    {
        //gps_getData();
        
    }
    */
    
    if (RUNNING_ON_BBB)
    {
        ss.cleanup();
    }

    //gps_cleanup();
    return 0;
}