/**
 * <rc_pilot_defs.h>
 *
 * @brief constants and parameters
 */

#ifndef RC_PILOT_DEFS_H
#define RC_PILOT_DEFS_H
/**
 * @brief      ARMED or DISARMED to indicate if the feedback controller is
 *             allowed to output to the motors
 */
typedef enum arm_state_t{
	DISARMED,
	ARMED
} arm_state_t;

/**
	 * @brief      determines how the dsm radio indicates an arm/disarm kill switch
	 */
typedef enum dsm_kill_mode_t {
	/**
	 * A dedicated channel is used as a kill switch. Carefully set the
	 * dsm_kill_ch and dsm_kill_pol channel and polarity settings.
	 */
	DSM_KILL_DEDICATED_SWITCH,
	/**
	 * Some radios, such as Spektrum DXe have an ARM/DISARM switch which
	 * forces the throttle channel down below normal range to disarm. This
	 * frees up a channel for other use and is the preffered method. When
	 * using this mode, dsm_kill_ch and dsm_kill_pol are ignored.
	 */
	 DSM_KILL_NEGATIVE_THROTTLE
} dsm_kill_mode_t;

/**
 *  @brief      Standard exit for initialization failures
 */
#define FAIL(str)                       \
    fprintf(stderr, str);               \
    rc_led_set(RC_LED_GREEN, 0);        \
    rc_led_blink(RC_LED_RED, 8.0, 2.0); \
    return -1;

// Speed of feedback loop
#define FEEDBACK_HZ		200
#define DT			0.005

//IMU Parameters
#define IMU_PRIORITY    51
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

// top safety
#define ARM_TIP_THRESHOLD	0.2	///< radians from level to allow arming sequence
#define TIP_ANGLE		1.5	///< radiands of roll or pitch to consider tipped over

// math constants
#define GRAVITY			9.80665	///< one G m/s^2

// order of control inputs
// throttle(Z), roll, pitch, YAW, sideways (X),forward(Y)
#define VEC_X			0
#define VEC_Y			1
#define VEC_Z			2
#define VEC_ROLL		3
#define VEC_PITCH		4
#define VEC_YAW			5

// user control parameters
#define MAX_YAW_RATE		2.5	// rad/s
#define MAX_ROLL_SETPOINT	0.2	// rad
#define MAX_PITCH_SETPOINT	0.2	// rad
#define MAX_CLIMB_RATE		1.0	// m/s
#define YAW_DEADZONE		0.02
#define THROTTLE_DEADZONE	0.02
#define SOFT_START_SECONDS	1.0	// controller soft start seconds
#define ALT_CUTOFF_FREQ		2.0
#define BMP_RATE_DIV		10	// optionally sample bmp less frequently than mpu

// controller absolute limits
#define MAX_ROLL_COMPONENT	0.4
#define MAX_PITCH_COMPONENT	0.4
#define MAX_YAW_COMPONENT	0.4
#define MAX_X_COMPONENT		1.0
#define MAX_Y_COMPONENT		1.0
#define XYZ_MAX_ERROR		0.5  ///< meters.
/**
 * MAX_THRUST_COMPONENT is really "lowest power state" or idle value. Note that
 * after the thrust mapping a different value will actually be sent to the motors.
 * The sign is inverted because these are control values in NED coordinates
 */
#define MAX_THRUST_COMPONENT	-0.05
#define MIN_THRUST_COMPONENT	-0.75

// Files
//#define LOG_DIR		"/home/debian/rc_pilot_logs/"
#define LOG_DIR		"../rc_pilot_logs/"

// for future modes, not used yet
#define LAND_TIMEOUT		0.3
#define DISARM_TIMEOUT		4.0

// terminal emulator control sequences
#define WRAP_DISABLE	"\033[?7l"
#define WRAP_ENABLE	"\033[?7h"
#define KNRM		"\x1B[0m"	// "normal" to return to default after colour
#define KRED		"\x1B[31m"
#define KGRN		"\x1B[32m"
#define KYEL		"\x1B[33m"
#define KBLU		"\x1B[34m"
#define KMAG		"\x1B[35m"
#define KCYN		"\x1B[36m"
#define KWHT		"\x1B[37m"

//#define DEBUG
#endif // RC_PILOT_DEFS_H
