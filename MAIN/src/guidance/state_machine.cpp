/*
 * state_machine.cpp
 *
 * Author:	Yevhenii Kovryzhenko, Department of Aerospace Engineering, Auburn University.
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
 *
 * Last Edit:  05/28/2022 (MM/DD/YYYY)
 *
 * Summary :
 * Data structures and functions related to using a state machine to manage waypoints and
 * actions
 *
 * Based on the original work of
 * Prince Kuevor (kuevpr@umich.edu), Derek Luckacs, Owen Marshall, and Matthew Romano
 */


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
#include "comms_tmp_data_packet.h"

#include "state_machine.hpp"

#include "thread_defs.h"
#include <rc/pthread.h>


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

static void* __state_machine_func(__attribute__((unused)) void* ptr)
{
    while (rc_get_state() != EXITING)
    {
        if (waypoint_state_machine.update_thread() < 0)
        {
            printf("ERROR in __state_machine_func: failed to update thread\n");
            return NULL;
        }
    }
    return NULL;
}

int state_machine_t::update_thread(void)
{
    if (load_file_fl)
    {

        build_waypoit_filename(settings.wp_folder, settings.wp_guided_filename);
        path.cleanup();

        if (path.set_new_path_NH(get_waypoint_filename()) < 0)
        {
            path.cleanup();
            printf("ERROR in update_thread: failed to set new path\n");
        }

        if (path.start_waypoint_counter_NH(setpoint) < 0)
        {
            path.cleanup();
            printf("ERROR in update_thread: failed to start the counter\n");
        }
        load_file_fl = false;;
    }
    else
    {
        rc_usleep(1000000 / STATE_MACHINE_HZ);
    }
    return 0;
}

char* state_machine_t::get_waypoint_filename(void)
{
    return waypoint_filename;
}

sm_states state_machine_t::get_current_state(void)
{
    return current_state;
}

bool state_machine_t::check_load_file(void)
{
    return load_file_fl;
}

int state_machine_t::request_load_file(void)
{
    load_file_fl = true;
    return 0;
}

/**
 * @brief Concatennates 'folder' and 'file' strings and stores them in 'dest' string
 */
void state_machine_t::build_waypoit_filename(char* folder, char* file)
{
    waypoint_filename[0] = '\0';
    strcat(waypoint_filename, folder);
    strcat(waypoint_filename, file);
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
    load_file_fl = false;

    if (thread.init(STATE_MACHINE_PRI,FIFO) < 0)
    {
        printf("ERROR in init: failed to initialize the thread\n");
        return -1;
    }

    if (thread.start(__state_machine_func) < 0)
    {
        printf("ERROR in init: failed to start the thread\n");
        return -1;
    }
    return 1;
}

int state_machine_t::enable_update(void)
{
    if (!en_update)
    {
        en_update       = true;
        changedState    = true;
    }
    return 0;
}

int state_machine_t::disable_update(void)
{
    if (en_update)
    {
        en_update = false;
        changedState = false;
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
        transition(user_input.get_flight_mode(), (sm_alphabet)GS_RX.sm_event);
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
                        setpoint.ATT.z.reset();
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
                    fprintf(stderr, "PARKED cannot transition with event %s\n",
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
                    fprintf(stderr, "STANDBY cannot transition with event %s\n",
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
                    fprintf(stderr, "TAKEOFF cannot transition with event %s\n",
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
                if (request_load_file() < 0)
                {
                    printf("ERROR in transition: failed to start loading the file\n");
                    path.cleanup();
                }

                changedState = false;
            }

            // State transition
            switch (input)
            {
                case ENTER_GUIDED:
                    
                    if (path.is_en())
                    {
                        if (path.update_setpoint_from_waypoint_NH(setpoint, *this) == -1)
                        {
                            printf("ERROR in transition: failed to update setpoint from waypoint\n");
                            path.cleanup();
                        }
                        
                    }
                    
                    break;

                case ENTER_STANDBY:
                    current_state = STANDBY;
                    changedState = true;
                    path.cleanup();
                    break;

                case ENTER_LANDING:
                    current_state   = LANDING;
                    changedState    = true;
                    path.cleanup();
                    break;

                case ENTER_LOITER:
                    current_state   = LOITER;
                    changedState    = true;
                    path.cleanup();
                    break;

                case ENTER_SQUARE:
                    current_state   = SQUARE;
                    changedState    = true;
                    path.cleanup();
                    break;

                case ENTER_RETURN:
                    current_state = RETURN;
                    changedState = true;
                    path.cleanup();
                    break;

                default:
                    fprintf(stderr, "GUIDED cannot transition with event %s\n",
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
                    fprintf(stderr, "LANDING cannot transition with event %s\n",
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
                fprintf(stderr, "LOITER cannot transition with event %s\n",
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
                fprintf(stderr, "SQUARE cannot transition with event %s\n",
                    sm_alph_strings[input]);
                break;
            }
            break;
        // States that have not yet been implemented
        case RETURN:
            fprintf(stderr,
                "RETURN mode not yet implemented. Switching state to STANDBY. Input: %s\n",
                sm_alph_strings[input]);
            current_state = STANDBY;
            changedState = true;
            break;
    }
}


int state_machine_t::clean_up(void)
{
    if (thread.is_started() && thread.stop(STATE_MACHINE_TOUT) < 0)
    {
        printf("ERROR in clean_up: failed to stop the thread\n");
        return -1;
    }
    return 0;
}
