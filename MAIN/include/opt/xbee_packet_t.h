/**
 * <xbee_packet.h>
 */

#include <stdint.h>

#ifndef __XBEE_PACKET__
#define __XBEE_PACKET__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

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
        uint8_t trackingValid;   // (bool) of whether or not tracking was valid (0 or 1)
        uint16_t sm_event;      ///< event (or input) for state machine
    } xbee_packet_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  //__XBEE_PACKET__