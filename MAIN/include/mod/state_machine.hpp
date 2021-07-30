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

#ifdef __cplusplus
extern "C"
{
#endif

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
        NAILING = 6,
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
        ENTER_NAILING = 6,
        ENTER_RETURN = 7,
        NO_EVENT,
    } sm_alphabet;

    /**
     * @brief
     */
    typedef struct state_machine_t
    {
        uint32_t state_transition_time;  // Time this state was transitioned into
        sm_states current_state;
        bool changedState;  // True if state has transitioned based on input event
    } state_machine_t;

    /**
     * @brief       Initial values for state_machine_t
     */
#define STATE_MACHINE_INITIALIZER                                                   \
    {                                                                               \
        .state_transition_time = 0, .current_state = STANDBY, .changedState = false \
    }
    extern state_machine_t waypoint_state_machine;

    int sm_init(state_machine_t* sm);
    void sm_transition(state_machine_t* sm, sm_alphabet input);


#ifdef __cplusplus
}
#endif

#endif /*__STATE_MACHINE__ */

/* @} end group StateMachine */
