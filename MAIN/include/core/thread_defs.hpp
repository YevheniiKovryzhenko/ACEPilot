

#ifndef THREAD_DEFS_HPP
#define THREAD_DEFS_HPP

// thread speeds, prioritites, and close timeouts
#define INPUT_MANAGER_HZ	20
#define INPUT_MANAGER_PRI	80
#define INPUT_MANAGER_TOUT	0.5

#define LOG_MANAGER_HZ		300
#define LOG_MANAGER_PRI		50
#define LOG_MANAGER_TOUT	2.0

#define PRINTF_MANAGER_HZ	20
#define PRINTF_MANAGER_PRI	60
#define PRINTF_MANAGER_TOUT	0.5

#define BUTTON_EXIT_CHECK_HZ	10
#define BUTTON_EXIT_TIME_S	2

#define STATE_MACHINE_HZ 400
#define STATE_MACHINE_PRI 79
#define STATE_MACHINE_TOUT 0.5
#define STATE_MACHINE_LOAD_HZ 20
#define STATE_MACHINE_LOAD_PRI 40
#define STATE_MACHINE_LOAD_TOUT 0.5


#define COMMS_MANAGER_HZ 2
#define COMMS_MANAGER_PRI 55
#define COMMS_MANAGER_TOUT 2

#define MOCAP_HZ 300
#define MOCAP_PRI 75
#define MOCAP_TOUT 1.0

#define ARM_FEEDBACK_HZ 200
#define ARM_FEEDBACK_PRI 55
#define ARM_FEEDBACK_TOUT 1.0

#define EXTRA_SENSORS_HZ 100
#define EXTRA_SENSORS_PRI 45
#define EXTRA_SENSORS_TOUT 1.0


#endif