/**
 * @file state_machine.c
 */

// NOTE: All functionality for LOITER will be added at a later time if at all.
// LOITER was not used much in a previous version of this project


#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <rc/time.h>

#include "setpoint_manager.hpp"
#include "settings.h"
#include "setpoint_guidance.hpp"
#include "input_manager.hpp"
#include "state_estimator.h"
#include "xbee_receive.h"

#include "state_machine.hpp"


static const char* sm_alph_strings[] = {
    "ENTER_PARKED",
    "ENTER_STANDBY",
    "ENTER_TAKEOFF",
    "ENTER_GUIDED",
    "ENTER_LANDING",
    "ENTER_LOITER",
    "ENTER_SQUARE",
    "ENTER_RETURN",
    "NO_EVENT",
};

state_machine_t waypoint_state_machine{};

//static char waypoint_filename[200];

/**
 * @brief Concatennates 'folder' and 'file' strings and stores them in 'dest' string
 */
static void state_machine_t::build_waypoit_filename(char* dest, char* folder, char* file)
{
    dest[0] = '\0';
    strcat(dest, folder);
    strcat(dest, file);
}

/**
 * @brief Initialize statemachine to PARKED state
 */
int state_machine_t::init(void)
{
    current_state               = PARKED;
    state_transition_time       = 0;
    changedState                = false;
    en_update                   = false;
    return 1;
}

int state_machine_t::enable_update(void)
{
    if (!en_update)
    {
        en_update       = true;
        changedState    = false;
    }
    return 0;
}

int state_machine_t::diable_update(void)
{
    if (en_update)
    {
        en_update = false;
        //changedState = false;
    }
    return 0;
}

bool state_machine_t::is_en(void)
{
    return en_update;
}

int state_machine_t::march(void)
{
    if (en_update)
    {
        transition(user_input.flight_mode, (sm_alphabet)xbeeMsg.sm_event);
    }
    return 0;
}


/**
 * @brief Parse the input and transition to new state if applicable
 */
void state_machine_t::transition(flight_mode_t flight_mode, sm_alphabet input)
{

    // Unique things that should be done for each state
    switch (current_state)
    {
        case PARKED:
            switch (input)
            {
                case ENTER_PARKED:
                    if (flight_mode == AUTONOMOUS)
                    {
                        setpoint.yaw = state_estimate.continuous_yaw;
                    }
                    if (setpoint_guidance.is_XY_en()) setpoint_guidance.reset_XY();
                    if (setpoint_guidance.is_Z_en()) setpoint_guidance.reset_Z();
                    break;
                case ENTER_TAKEOFF:
                    current_state = TAKEOFF;
                    changedState = true;
                    break;
                case ENTER_LANDING:
                    //do nothing
                    break;
                case ENTER_GUIDED:
                    current_state = GUIDED;
                    changedState = true;
                    break;

                default:
                    fprintf(stderr, "\nPARKED cannot transition with event %s\n",
                        sm_alph_strings[input]);
                    break;
            }
            break;
        /** STANDBY: waits until further instructions. Kills all guidance if active.
         * Valid Transitions: All, but PARKED
         */
        case STANDBY:
            switch (input)
            {
                case ENTER_STANDBY:
                    if (setpoint_guidance.is_XY_en()) setpoint_guidance.reset_XY();
                    if (setpoint_guidance.is_Z_en()) setpoint_guidance.reset_Z();
                    break;

                case ENTER_TAKEOFF:
                    current_state       = TAKEOFF;
                    changedState        = true;
                    break;
                case ENTER_GUIDED:
                    current_state   = GUIDED;
                    changedState    = true;
                    break;

                case ENTER_LANDING:
                    current_state   = LANDING;
                    changedState    = true;
                    break;

                case ENTER_LOITER:
                    current_state   = LOITER;
                    changedState    = true;
                    break;

                case ENTER_SQUARE:
                    current_state   = SQUARE;
                    changedState    = true;
                    break;

                case ENTER_RETURN:
                    current_state   = RETURN;
                    changedState    = true;
                    break;

                default:
                    fprintf(stderr, "\nSTANDBY cannot transition with event %s\n",
                        sm_alph_strings[input]);
                    break;
            }
            break;

        /** TAKEOFF: Assumes vehicle is on the ground. Activates procedure for automated ascent to get from
         * ground into the air. Valid Transitions: STANDBY, GUIDED, LANDING, LOITER, SQUARE,
         * RETURN
         */
        case TAKEOFF:

            // Actions associated with this state
            if (changedState)
            {
                changedState = false;

                //terminate all the previous guidance jobs and clean up
                setpoint_guidance.reset_XY();
                setpoint_guidance.reset_Z();
                //start new job
                setpoint_guidance.restart_takeoff();
            }

            // State transition
            switch (input)
            {
                case ENTER_TAKEOFF:
                    if (setpoint_guidance.get_state_Z() && setpoint_guidance.is_Z_en())
                    {
                        setpoint_guidance.reset_Z(); //should never get here, but just in case
                    }
                    break;

                case ENTER_STANDBY:
                    current_state   = STANDBY;
                    changedState    = true;
                    break;

                case ENTER_GUIDED:
                    current_state   = GUIDED;
                    changedState    = true;
                    break;

                case ENTER_LANDING:
                    current_state   = LANDING;
                    changedState    = true;
                    break;

                case ENTER_LOITER:
                    current_state   = LOITER;
                    changedState    = true;
                    break;

                case ENTER_SQUARE:
                    current_state   = SQUARE;
                    changedState    = true;
                    break;

                case ENTER_RETURN:
                    current_state   = RETURN;
                    changedState    = true;
                    break;

                default:
                    fprintf(stderr, "\nTAKEOFF cannot transition with event %s\n",
                        sm_alph_strings[input]);
                    break;
            }
            break;

        /** GUIDED: Assumes vehicle is in the air. Parses through waypoints of desried
         * trajectory. Valid Transitions: STANDBY, LANDING, LOITER, SQUARE, RETURN
         */
        case GUIDED:
            // Actions associated with this state
            if (changedState)
            {
                //terminate all the previous guidance jobs and clean up
                setpoint_guidance.reset_XY();
                setpoint_guidance.reset_Z();
                //start new job
                build_waypoit_filename(
                    waypoint_filename, settings.wp_folder, settings.wp_guided_filename);

                if (setpoint.set_new_path(waypoint_filename) == -1)
                {
                    printf("\nERROR: failed to set new path");
                    break;
                }

                if (!waypoint_state_machine.is_en()) waypoint_state_machine.enable_update();
            }

            // State transition
            switch (input)
            {
                case ENTER_GUIDED:
                    break;

                case ENTER_STANDBY:
                    current_state = STANDBY;
                    changedState = true;
                    break;

                case ENTER_LANDING:
                    current_state   = LANDING;
                    changedState    = true;
                    break;

                case ENTER_LOITER:
                    current_state   = LOITER;
                    changedState    = true;
                    break;

                case ENTER_SQUARE:
                    current_state   = SQUARE;
                    changedState    = true;
                    // TODO: Load waypoints from SQUARE file
                    break;

                case ENTER_RETURN:
                    current_state = RETURN;
                    changedState = true;
                    // TODO: Load waypoints from RETURN file
                    break;

                default:
                    fprintf(stderr, "\nGUIDED cannot transition with event %s\n",
                        sm_alph_strings[input]);
                    break;
            }
            break;

        /** LANDING: Assumes vehicle is in the air. Starts automater landing procedure to get from the air
         * to the ground. Valid Transitions: STANDBY, TAKEOFF
         */
        case LANDING:
            // Actions associated with this state
            if (changedState)
            {
                //terminate all the previous guidance jobs and clean up
                setpoint_guidance.reset_XY();
                setpoint_guidance.reset_Z();
                //start new job
                setpoint_guidance.restart_land();

                changedState = false;
            }

            // State transition
            switch (input)
            {
                case ENTER_STANDBY:
                    current_state = STANDBY;
                    changedState = true;
                    break;

                case ENTER_TAKEOFF:
                    current_state = TAKEOFF;
                    changedState = true;
                    break;

                case ENTER_LANDING:
                    if (setpoint_guidance.get_state_Z())
                    {
                        if (setpoint_guidance.is_Z_en())
                        {
                            setpoint_guidance.reset_Z(); //should never get here, but just in case
                        }
                        current_state               = PARKED;
                        changedState                = true;
                        user_input.requested_arm_mode   = DISARMED;
                        // should be on the ground at this point
                    }
                    break;

                default:
                    fprintf(stderr, "\nLANDING cannot transition with event %s\n",
                        sm_alph_strings[input]);
                    break;
            }
            break;

            /** LOITER: Assumes vehicle is in the air. Activates automated circular guidance script
             *  about the point of activation. Valid Transitions: STANDBY, LANDING
             */
        case LOITER:
            // Actions associated with this state
            if (changedState)
            {
                //terminate all the previous guidance jobs and clean up
                setpoint_guidance.reset_XY();
                setpoint_guidance.reset_Z();
                //start new job
                setpoint_guidance.restart_circ(); 

                changedState = false;
            }

            // State transition
            switch (input)
            {
            case ENTER_STANDBY:
                current_state = STANDBY;
                changedState = true;
                break;
            case ENTER_LANDING:
                current_state = LANDING;
                changedState = true;
                break;

            case ENTER_LOITER:
                if (setpoint_guidance.get_state_XY() && setpoint_guidance.is_XY_en())\
                    setpoint_guidance.reset_XY();
                break;

            default:
                fprintf(stderr, "\nLOITER cannot transition with event %s\n",
                    sm_alph_strings[input]);
                break;
            }
            break;
        /** SQUARE: Similar to LOITER, but square. Assumes vehicle is in the air. Activates automated 
        *  square guidance script aboubt the point of activation. Valid Transitions: STANDBY, LANDING
        */
        case SQUARE:
            // Actions associated with this state
            if (changedState)
            {
                //terminate all the previous guidance jobs and clean up
                setpoint_guidance.reset_XY();
                setpoint_guidance.reset_Z();
                //start new job
                setpoint_guidance.restart_square();

                changedState = false;
            }

            // State transition
            switch (input)
            {
            case ENTER_STANDBY:
                current_state = STANDBY;
                changedState = true;
                break;
            case ENTER_LANDING:
                current_state = LANDING;
                changedState = true;
                break;

            case ENTER_SQUARE:
                if (setpoint_guidance.get_state_XY() && setpoint_guidance.is_XY_en())\
                    setpoint_guidance.reset_XY();
                break;

            default:
                fprintf(stderr, "\nSQUARE cannot transition with event %s\n",
                    sm_alph_strings[input]);
                break;
            }
            break;
        // States that have not yet been implemented
        case RETURN:
            fprintf(stderr,
                "\nRETURN mode not yet implemented. Switching state to STANDBY. Input: %s\n",
                sm_alph_strings[input]);
            current_state = STANDBY;
            changedState = true;
            break;
    }
}
