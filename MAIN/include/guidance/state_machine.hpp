/**
 * <state_machine.h>
 *
 * @brief Data structures and functions related to using a state machine to manage waypoints and
 * actions
 *
 * @author Prince Kuevor (kuevpr@umich.edu), Derek Luckacs, Owen Marshall, and Matthew Romano
 *
 * @addtogroup StateMachine
 * @{
 */

#ifndef __STATE_MACHINE__
#define __STATE_MACHINE__

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

        /**
        * @brief Concatennates 'folder' and 'file' strings and stores them in 'dest' string
        */
        void build_waypoit_filename(char* dest, char* folder, char* file);

        /**
        * @brief Parse the input and transition to new state if applicable
        */
        void transition(flight_mode_t flight_mode, sm_alphabet input);
    public:
        int init(void);
        int enable_update(void);
        int disable_update(void);
        bool is_en(void);
        int march(void);
        
    };
    extern state_machine_t waypoint_state_machine;

#endif /*__STATE_MACHINE__ */

/* @} end group StateMachine */
