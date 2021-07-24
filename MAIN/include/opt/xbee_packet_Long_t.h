#include <stdint.h>

#ifndef __XBEE_PACKET__
#define __XBEE_PACKET__
#ifdef __cplusplus
extern "C"
{
#endif

    // XBee Packet
    typedef struct xbee_packet_t
    {
        uint32_t time;   ///< Unique id for the rigid body being described
        float x;        ///< x-position in the Optitrack frame
        float y;        ///< y-position in the Optitrack frame
        float z;        ///< z-position in the Optitrack frame
        float qx;       ///< qx of quaternion
        float qy;       ///< qy of quaternion
        float qz;       ///< qz of quaternion
        float qw;       ///< qw of quaternion
        uint16_t trackingValid;   // (bool) of whether or not tracking was valid (0 or 1)
        uint16_t sm_event;       ///< event (or input) for state machine
        int32_t GainCH;
        float GainN0;
        float GainN1;
        float GainN2;
        float GainD0;
        float GainD1;
        float GainD2;
    } xbee_packet_t;

#define NUM_FRAMING_BYTES 4                 // 2 START bytes + 2 Fletcher-16 checksum bytes
#define OPTI_DATA_LENGTH            sizeof(xbee_packet_t)      // Actual Packet Being Sent
#define OPTI_PACKET_LENGTH	    OPTI_DATA_LENGTH + NUM_FRAMING_BYTES

    //extern xbee_packet_t xbeeMsg;
    //extern int xbee_portID;

#ifdef __cplusplus
}
#endif
#endif  //__XBEE_PACKET__