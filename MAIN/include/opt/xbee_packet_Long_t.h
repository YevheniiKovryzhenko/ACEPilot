#include <stdint.h>

#ifndef __XBEE_PACKET__
#define __XBEE_PACKET__
#ifdef __cplusplus
extern "C"
{
#endif

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
        uint8_t sm_event;       ///< event (or input) for state machine
        bool en_tunning; // (bool) enables remote tunning 
        uint8_t GainCH; //< tunning channel 
        float GainN1_i; //< N0 is always 0; D0 is always 1; D1 is always -1
        float GainN0_pd;
        float GainN1_pd;
        float GainD1_pd; //< D0 is always  1
    }__attribute__((packed)) xbee_packet_t;

#ifdef __cplusplus
}
#endif
#endif  //__XBEE_PACKET__