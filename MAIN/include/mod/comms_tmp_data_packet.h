/**
 * @file commms_tmp_data_packet.h
 *
 * @brief      temporary file for C/C++ compatibility
 *
 * contains the currently used packet for mocap data
 */



#ifndef COMMS_TMP_DATA_PACKET
#define COMMS_TMP_DATA_PACKET
#include "xbee_packet_Long_t.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern xbee_packet_t mocap_msg;

#ifdef __cplusplus
}
#endif
#endif // COMMS_TMP_DATA_PACKET