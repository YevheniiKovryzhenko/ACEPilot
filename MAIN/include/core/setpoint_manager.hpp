/*
 * setpoint_manager.hpp
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
 * Last Edit:  08/13/2020 (MM/DD/YYYY)
 *
 * Summary :
 * Setpoint manager runs at the same rate as the feedback controller
 * and is the interface between the user inputs (input manager) and
 * the feedback controller setpoint. currently it contains very
 * simply logic and runs very quickly which is why it's okay to run
 * in the feedback ISR right before the feedback controller. In the
 * future this is where go-home and other higher level autonomy will
 * live.
 *
 * This serves to allow the feedback controller to be as simple and
 * clean as possible by putting all high-level manipulation of the
 * setpoints here. Then feedback-controller only needs to march the
 * filters and zero them out when arming or enabling controllers
 *
 */


/**
 * <setpoint_manager.h>
 *
 * @brief      
 */

#include <stdint.h> // for uint64_t

#include <rc/time.h> // for nanos
#include <rc/start_stop.h>
#include <rc/math/quaternion.h>

#ifndef SETPOINT_MANAGER_H
#define SETPOINT_MANAGER_H

	
/**
	* Setpoint for the feedback controllers. This is written by setpoint_manager
	* and primarily read in by feedback_controller. May also be read by printf_manager
	* and log_manager for telemetry
	*/
class setpoint_t {
private:
	/** @name general */
	///< @{
	bool initialized;	///< set to 1 once setpoint manager has initialized
	///< @}


	/**
	* Function only used locally
	*/
	int update_setpoints(void);
public:
	/** @name general */
	///< @{
	bool en_6dof;		///< enable 6DOF control features
	///< @}

	/** @name direct passthrough
	* user inputs tranlate directly to mixing matrix
	*/
	///< @{
	double Z_throttle;				///< final value which is being actually used for control mixing
	double X_throttle;				///< final value which is being actually used for control mixing
	double Y_throttle;				///< final value which is being actually used for control mixing
	double roll_throttle;			///< final value which is being actually used for control mixing
	double pitch_throttle;			///< final value which is being actually used for control mixing
	double yaw_throttle;			///< final value which is being actually used for control mixing
	///< @}
	///< @{
	double Z_servo_throttle;		///< final value which is being actually used for control mixing
	double X_servo_throttle;		///< final value which is being actually used for control mixing
	double Y_servo_throttle;		///< final value which is being actually used for control mixing
	double roll_servo_throttle;		///< final value which is being actually used for control mixing
	double pitch_servo_throttle;	///< final value which is being actually used for control mixing
	double yaw_servo_throttle;		///< final value which is being actually used for control mixing
	///< @}

	/** @name attitude rate setpoint */
	///< @{
	bool en_rpy_rate_ctrl;		///< enable the roll pitch yaw rate controllers
	double roll_dot;			///< roll angle rate (rad/s)
	double pitch_dot;			///< pitch angle rate (rad/s)
	double yaw_dot;				///< yaw angle rate (rad/s)
	double roll_dot_ff;			///< feedforward roll rate (rad/s)
	double pitch_dot_ff;		///< feedforward pitch rate (rad/s)
	double yaw_dot_ff;			///< feedforward yaw rate (rad/s)
	///< @}

	/** @name attitude setpoint */
	///< @{
	bool en_rpy_ctrl;	///< enable the roll pitch yaw controllers
	double roll;		///< roll angle (positive tip right) (rad)
	double pitch;		///< pitch angle (positive tip back) (rad)
	double yaw;               ///< glabal yaw angle, positive left
	double roll_ff;           ///< feedforward roll angle (rad)
	double pitch_ff;          ///< feedforward pitch angle (rad)
	///< @}

	/** @name acceleration setpoint */
	///< @{
	double X_ddot;
	double Y_ddot;
	double Z_ddot;
	///< @}

	/** @name altitude */
	///< @{
	bool en_Z_ctrl;			///< enable altitude feedback.
	bool en_Z_rate_ctrl;	///< enable altitude rate feedback.
	double Z;				///< vertical distance from where controller was armed
	double Z_dot;			///< vertical velocity m/s^2, remember Z points down
	double Z_dot_ff;		///< feedforward vertical velocity (m/s)
	double Z_throttle_0;	/// hover throttle

	/** @name horizontal velocity setpoint */
	///< @{
	bool en_XY_vel_ctrl;
	double X_dot;
	double Y_dot;
	double X_dot_ff;    ///< feedforward x velocity (m/s)
	double Y_dot_ff;    ///< feedforward y velocity (m/s)
	///< @}

	/** @name horizontal position setpoint */
	///< @{
	bool en_XY_pos_ctrl;
	double X;
	double Y;


	/**
	* @brief      Initializes the setpoint manager.
	*
	* @return     0 on success, -1 on failure
	*/
	int init(void);
	bool is_initialized(void);

	/**
	* @brief      updates the setpoint manager, call this before feedback loop
	*
	* @return     0 on success, -1 on failure
	*/
	int update(void);

	/**
	* @brief      cleans up the setpoint manager, not really necessary but here for
	*             completeness
	*
	* @return     0 on clean exit, -1 if exit timed out
	*/
	int cleanup(void);

	/**
	* @brief   Externally order setpoint manager to follow new path from path_file
	*
	* @return  0 on success, -1 if unsuccessful
	*/
	int set_new_path(const char* path_file);

	/* Functions which should be called exterally to update setpoints based on radio input:*/
	void update_yaw(void);
	void update_Z(void);
	void update_XY_pos(void);

};

extern setpoint_t setpoint;

#endif // SETPOINT_MANAGER_H
