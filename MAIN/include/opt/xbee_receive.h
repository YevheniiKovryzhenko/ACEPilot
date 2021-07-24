/**
 * <xbee_receive.h>
 */


#ifndef XBEE_RECEIVE_H
#define XBEE_RECEIVE_H

#include <stdio.h>
#include <unistd.h> // read / write
#include <stdlib.h>	//one of these two is for memcpy
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include <rc/time.h> // for nanos


#include "serial_com.h"
#include "settings.h"
//#include "xbee_packet_t.h"
#include "xbee_packet_Long_t.h"


#ifdef __cplusplus
extern "C" {
#endif
    extern xbee_packet_t xbeeMsg;
    extern int xbee_portID;

    int XBEE_init(const char* xbee_port, const int baudrate);
    int XBEE_getData();
    void XBEE_printData();

#ifdef __cplusplus
}
#endif
#endif // !XBEE_RECEIVE_H
