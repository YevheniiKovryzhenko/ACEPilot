#include <stdbool.h>
#include "flight_mode.h"


bool mode_needs_mocap(flight_mode_t mode)
{
    if(mode > MANUAL_FFxxxx && mode < ZEPPELIN)
    {
        return true;
    }

    return false;
}