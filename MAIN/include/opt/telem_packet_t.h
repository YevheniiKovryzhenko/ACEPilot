#include <stdint.h>

#ifndef __TELEM_PACKET__
#define __TELEM_PACKET__
#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct telem_packet_t
    {
        uint64_t time;          ///< Unique id for the rigid body being described
        uint8_t st_SM;          ///< state machine state
        uint8_t st_f;           ///< flight state
        float x;                ///< x-position state
        float y;                ///< y-position state
        float z;                ///< z-position state
        float x_sp;             ///< x-position setpoint
        float y_sp;             ///< y-position setpoint
        float z_sp;             ///< z-position setpoint
        float x_dot;            ///< x-velocity state
        float y_dot;            ///< y-velocity state
        float z_dot;            ///< z-velocity state
        float x_dot_sp;         ///< x-velocity setpoint
        float y_dot_sp;         ///< y-velocity setpoint
        float z_dot_sp;         ///< z-velocity setpoint
        float roll;             ///< roll state
        float pitch;            ///< pitch state
        float yaw;              ///< yaw state
        float roll_sp;          ///< roll setpoint
        float pitch_sp;         ///< pitch setpoint
        float yaw_sp;           ///< yaw setpoint
        float roll_dot;         ///< roll rate state
        float pitch_dot;        ///< pitch rate state
        float yaw_dot;          ///< yaw rate state
        float roll_dot_sp;      ///< roll rate setpoint
        float pitch_dot_sp;     ///< pitch rate setpoint
        float yaw_dot_sp;       ///< yaw rate setpoint
    }__attribute__((packed)) telem_packet_t;

#ifdef __cplusplus
}
#endif
#endif  //__TELEM_PACKET__