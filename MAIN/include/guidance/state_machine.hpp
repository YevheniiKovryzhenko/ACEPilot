/*
 * state_machine.hpp
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
 * Last Edit:  06/30/2022 (MM/DD/YYYY)
 *
 * Summary :
 * Data structures and functions related to using a state machine to manage waypoints and
 * actions
 *
 * Based on the original work of 
 * Prince Kuevor (kuevpr@umich.edu), Derek Luckacs, Owen Marshall, and Matthew Romano
 */

#ifndef __STATE_MACHINE__
#define __STATE_MACHINE__
#include "flight_mode.h"
#include "thread_gen.hpp"

    /**
     * @brief List of possible states for the state machine. States can be added as needed for new
     * functionality.
     */
    typedef enum sm_states
    {
        PARKED = 0,
        STANDBY = 1,
        TAKEOFF = 2,
        GUIDED = 3,
        LANDING = 4,
        LOITER = 5,
        SQUARE = 6,
        RETURN = 7,
    } sm_states;

    /**
     * @brief Alphabet of possible transition inputs for the state machine. Inputs can be added as
     * needed for new functionality.
     */
    typedef enum sm_alphabet
    {
        ENTER_PARKED = 0,
        ENTER_STANDBY = 1,
        ENTER_TAKEOFF = 2,
        ENTER_GUIDED = 3,
        ENTER_LANDING = 4,
        ENTER_LOITER = 5,
        ENTER_SQUARE = 6,
        ENTER_RETURN = 7,
        NO_EVENT,
    } sm_alphabet;

    /**
     * @brief
     */
    class state_machine_t
    {
    private:
        uint32_t state_transition_time;  // Time this state was transitioned into
        sm_states current_state;
        bool changedState;  // True if state has transitioned based on input event
        bool en_update;
        char waypoint_filename[200];

        bool load_file_fl;
        
        thread_gen_t thread;
        thread_gen_t thread_load;

        /**
        * @brief Parse the input and transition to new state if applicable
        */
        void transition(sm_alphabet input);

        /**
        * @brief Concatennates 'folder' and 'file' strings and stores them in 'dest' string
        */
        void build_waypoit_filename(char* folder, char* file);

        /**
         * @brief      Sets the parameters for the thread.
         *
         * @return     0 on success, -1 on failure
         */
        bool check_load_file(void);

        /**
         * @brief      Sends a request to a thread to load a new file
         *
         * @return     0 on success, -1 on failure
         */
        int request_load_file(void);

        /**
        * @brief return waypoint filename used
        */
        char* get_waypoint_filename(void);

        

    public:
        

        /**
         * @brief      Initializes everyhting. Called once.
         *
         * @return     0 on success, -1 on failure
         */
        int init(void);

        /**
         * @brief      Sends a request to State Machine thread to update setpoints
         *
         * @return     0 on success, -1 on failure
         */
        int request_update(void);

        /**
         * @brief      Disables State Machine Autonomous Updates
         *
         * @return     0 on success, -1 on failure
         */
        //int disable_update(void);

        

        /**
         * @brief      returns if State Machine update is enabled
         */
        bool is_en(void);

        /**
         * @brief      Main update of the State Machine, called in main loop.
         *
         * @return     0 on success, -1 on failure
         */
        int march(void);

        /**
         * @brief      State Machine thread update loop.
         *
         * @return     0 on success, -1 on failure
         */
        int update_thread(void);

        /**
         * @brief      State Machine loading thread update loop.
         *
         * @return     0 on success, -1 on failure
         */
        int update_load_thread(void);

        /**
         * @brief      returns current state of state machine
         */
        sm_states get_current_state(void);     

        /**
         * @brief      State Machine reset function.
         *
         * @return     0 on success, -1 on failure
         */
        int reset(void);

        /**
         * @brief      State Machine clean exit.
         *
         * @return     0 on success, -1 on failure
         */
        int clean_up(void);
        
    };
    extern state_machine_t waypoint_state_machine;

#endif /*__STATE_MACHINE__ */

/* @} end group StateMachine */