/**
 * <setpoint_manager.h>
 *
 * @brief      Setpoint manager runs at the same rate as the feedback controller
 *             and is the interface between the user inputs (input manager) and
 *             the feedback controller setpoint. currently it contains very
 *             simply logic and runs very quickly which is why it's okay to run
 *             in the feedback ISR right before the feedback controller. In the
 *             future this is where go-home and other higher level autonomy will
 *             live.
 *
 *             This serves to allow the feedback controller to be as simple and
 *             clean as possible by putting all high-level manipulation of the
 *             setpoints here. Then feedback-controller only needs to march the
 *             filters and zero them out when arming or enabling controllers
 */

#ifndef SETPOINT_MANAGER_H
#define SETPOINT_MANAGER_H
#ifdef __cplusplus
extern "C"
{
#endif
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h> // for memset
#include <stdint.h> // for uint64_t
#include <inttypes.h> // for PRIu64

#include <rc/time.h> // for nanos
#include <rc/start_stop.h>
#include <rc/math/quaternion.h>

#include "rc_pilot_defs.h"
#include "input_manager.h"
#include "settings.h"
#include "path.h"
#include "state_estimator.h"
#include "flight_mode.h"
#include "xbee_receive.h"
#include "trajectories_common.h"
#include "tools.h"
/**
 * Setpoint for the feedback controllers. This is written by setpoint_manager
 * and primarily read in by fly_controller. May also be read by printf_manager
 * and log_manager for telemetry
 */
typedef struct setpoint_t{

	/** @name general */
	///< @{
	int initialized;	///< set to 1 once setpoint manager has initialized
	int en_6dof;		///< enable 6DOF control features
	///< @}

	/** @name direct passthrough
	 * user inputs tranlate directly to mixing matrix
	 */
	///< @{
	double Z_throttle;	///< used only when altitude controller disabled
	double X_throttle;	///< only used when 6dof is enabled, positive forward
	double Y_throttle;	///< only used when 6dof is enabled, positive right
	double roll_throttle;	///< only used when roll_pitch_yaw controllers are disabled
	double pitch_throttle;	///< only used when roll_pitch_yaw controllers are disabled
	double yaw_throttle;	///< only used when roll_pitch_yaw controllers are disabled
	///< @}

	/** @name attitude setpoint */
	///< @{
	int en_rpy_ctrl;	///< enable the roll pitch yaw controllers
	double roll;		///< roll angle (positive tip right) (rad)
	double pitch;		///< pitch angle (positive tip back) (rad)
	double yaw;			///< global yaw angle, positive right
	double yaw_dot;		///< desired rate of change in yaw rad/s
	///< @}

	/** @name altitude */
	///< @{
	int en_Z_ctrl;		///< enable altitude feedback.
	double Z;		///< vertical distance from where controller was armed
	double Z_dot;		///< vertical velocity m/s^2, remember Z points down
	double Z_throttle_0; /// hover throttle
	/** @name auto-land/take-off logic */
	///< @{
	int en_Z_land;				///< enable landing algorithm.
	int en_Z_takeoff;			///< enable take-off algorithm.
	int st_land;				///< landing status (failure/success) - use it externally for automated algorithms
	int st_takeoff;				///< take-off status (failure/success) - use it externally for automated algorithms
	int last_en_land;    		///< landing algorithm running flag
	int last_en_takeoff;    	///< take-off algorithm running flag
	uint64_t land_time_ns;		///< time since boot when landing was initiated
	uint64_t takeoff_time_ns;	///< time since boot when take-off was initiated
	double t_land;				///< time to land (seconds)
	double t_takeoff;			///< time for take-off (seconds)	
	double V_max_land;			///< vertical velocity for landing
	double Z_init_takeoff;		///< altitude when take-off is initiated
	double dZ_takeoff;			///< take off increment from the initial altitude  		
	///< @}
	///< @}

	/** @name horizontal velocity setpoint */
	///< @{
	int en_XY_vel_ctrl;
	double X_dot;
	double Y_dot;
	
	//double Vel_vec_ctrl[3];
	///< @}

	/** @name horizontal position setpoint */
	///< @{
	int en_XY_pos_ctrl;
	double X;
	double Y;
	
	/** @name automated trajectory */
	///< @{
	//int en_XY_AUTO;					///< enable/disable automated trajectory
	//int en_XY_AUTO_TEST;			///< enable/disable automated testing trajectory
    int en_AUTO_LIFTOFF_HOWER_TEST; ///< enable/disable automated liftoff trajectory
    int en_XY_SQUARE_TEST;			///< enable/disable automated square trajectory
    int en_XY_CIRC_TEST;			///< enable/disable automated circular trajectory

	int st_AUTO;				///< algorithm status (falure/success) - use it externally for automated algorithms
	uint64_t AUTO_time_ns;		///< time since boot when algorithm was initiated
	int	AUTO_wp;				///< waypoint progress flag (used internally)
	double dX_AUTO;				///< increment in X
	double dY_AUTO;				///< increment in Y	
	double t_AUTO;				///< total tome for auto

	//Square:
    int en_XY_SQUARE;
    int st_XY_SQUARE;

	//Circular:
	int en_XY_CIRC;				///< enable/disable automated trajectory
    int st_XY_CIRC;				///< algorithm status (falure/success) - use it externally for automated algorithms
    double dX_CIRC;             ///< increment in X
    double dY_CIRC;             ///< increment in Y
    double dYaw_CIRC;			///< increment in Yaw
    double T_CIRC;				///< time/period for circular trajectory
    double R_CIRC;				///< radius of a circle to make

	/**
     * @name Waypoint Management
     *
     */
    ///< @{
    uint64_t time_auto_set;     ///< time autonomous mode is set and armed
    double waypoint_time;       ///< current time to compare to waypoint times
    int auto_armed_set;         ///< flag to manage time auto + armed is set
    int en_waypoint_update;		///< flag to control whether path command is actually being applied to the setpoint 
    uint64_t cur_waypoint_num;  ///< current waypoint to control to
    // int en_hover;               ///< enable hover feedback controller mode
    ///< @}
} setpoint_t;

extern setpoint_t setpoint;

/**
 * @brief      Initializes the setpoint manager.
 *
 * @return     0 on success, -1 on failure
 */
int setpoint_manager_init(void);

/**
 * @brief      updates the setpoint manager, call this before feedback loop
 *
 * @return     0 on success, -1 on failure
 */
int setpoint_manager_update(void);

/**
 * @brief      cleans up the setpoint manager, not really necessary but here for
 *             completeness
 *
 * @return     0 on clean exit, -1 if exit timed out
 */
int setpoint_manager_cleanup(void);

/**
 * @brief   Externally order setpoint manager to follow new path from path_file
 *
 * @return  0 on success, -1 if unsuccessful
 */
int set_new_path(const char* path_file);

/**
 * @brief Update the setpoint form the next waytpoint
 */
void setpoint_update_setpoint_from_waypoint();

#ifdef __cplusplus
}
#endif
#endif // SETPOINT_MANAGER_H
