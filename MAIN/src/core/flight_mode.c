#include <stdbool.h>
#include "flight_mode.h"


bool mode_needs_mocap(flight_mode_t mode)
{
    if(mode == AUTONOMOUS || mode == ALT_HOLD_SS || mode == ALT_HOLD_FS || mode == ALT_HOLD_FF ||\
        mode == POSITION_CONTROL_SSS || mode == POSITION_CONTROL_FSS || mode == POSITION_CONTROL_FFS\
        || mode == POSITION_CONTROL_FFF)
    {
        return true;
    }

    return false;
}