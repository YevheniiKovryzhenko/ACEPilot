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
 * Last Edit:  05/28/2022 (MM/DD/YYYY)
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
#include <rc/math/filter.h>
#include "setpoint_gen.hpp"
#include "flight_mode.h"

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
	flight_mode_t flight_mode_prev;
	bool flight_mode_switching = true;
	///< @}

	/* stick trim flags */
	/*
	bool last_en_trans;
	bool last_en_stick_trim;
	bool last_en_stick_filter_lp;
	bool last_en_stick_filter_hp;
	*/

	/* stick trims */
	/*
	double roll_stick_trim;
	double pitch_stick_trim;
	double yaw_stick_trim;
	rc_filter_t roll_stick_int;
	rc_filter_t pitch_stick_int;
	rc_filter_t yaw_stick_int;
	rc_filter_t trans_stick_int;
	*/

	/* stick filters */
	/*
	double roll_stick_lp;
	double pitch_stick_lp;
	double yaw_stick_lp;
	double roll_stick_hp;
	double pitch_stick_hp;
	double yaw_stick_hp;
	rc_filter_t roll_stick_lpf;
	rc_filter_t pitch_stick_lpf;
	rc_filter_t yaw_stick_lpf;
	rc_filter_t roll_stick_hpf;
	rc_filter_t pitch_stick_hpf;
	rc_filter_t yaw_stick_hpf;
	*/
	/**
	* Function only used locally
	*/
	int init_all_filters(void);
	/*
	int init_trans(void);
	int init_stick_trim(void);
	int init_stick_filter(void);
	
	int update_trans(void);
	int update_stick_trim(double roll_st, double pitch_st, double yaw_st);
	int update_stick_filter_lp(void);
	int update_stick_filter_hp(void);
	*/
	int update_setpoints(void);

	/**
	* @brief      Configure setpoints to reset to state_estimate.
	*
	* @return     0 on success, -1 on failure
	*/
	int set_reset_sources(void);

	/**
	* @brief      Set setpoints to input values from the radio.
	*
	* @return     0 on success, -1 on failure
	*/
	void update_th(void);
	void update_rp(void);
	void update_rpy_rate(void);
	void update_yaw(void);
	void update_Z(void);
	void update_Z_dot(void);
	void update_XY_pos(void);
	void update_rpy_servo(void);
public:
	/** @name general */
	///< @{
	bool en_6dof;		///< enable 6DOF control features
	bool en_6dof_servo;
	///< @}

	/** @name direct passthrough
	* user inputs tranlate directly to mixing matrix
	*/
	///< @{
	setpoint_gen_3D_t POS_throttle{};	///< final value which is being actually used for control mixing
	setpoint_gen_3D_t ATT_throttle{};	///< final value which is being actually used for control mixing
	///< @}
	///< @{
	double Z_servo_throttle;		///< final value which is being actually used for control mixing
	double X_servo_throttle;		///< final value which is being actually used for control mixing
	double Y_servo_throttle;		///< final value which is being actually used for control mixing
	double roll_servo_throttle;		///< final value which is being actually used for control mixing
	double pitch_servo_throttle;	///< final value which is being actually used for control mixing
	double yaw_servo_throttle;		///< final value which is being actually used for control mixing
	///< @}

	/** @name attitude rate setpoints */
	///< @{
	setpoint_gen_3D_t ATT_dot{};	///< roll pitch yaw derivative setpoints
	///< @}

	/** @name attitude rate setpoints transition rates */
	///< @{
	/*
	bool en_rpy_rate_trans;			///< enable the roll pitch yaw rate transition functions
	double roll_dot_tr;				///< roll angle rate transition gain
	double pitch_dot_tr;			///< pitch angle rate transition gain
	double yaw_dot_tr;				///< yaw angle rate transition gain
	double roll_dot_ff_tr;			///< feedforward roll rate transition gain
	double pitch_dot_ff_tr;			///< feedforward pitch rate transition gain
	double yaw_dot_ff_tr;			///< feedforward yaw rate transition gain
	*/
	///< @}

	/** @name attitude rate servo setpoints */
	///< @{
	bool en_rpy_rate_servo_ctrl;	///< enable the roll pitch yaw rate controllers
	double roll_dot_servo;			///< roll angle rate (rad/s)
	double pitch_dot_servo;			///< pitch angle rate (rad/s)
	double yaw_dot_servo;			///< yaw angle rate (rad/s)
	double roll_dot_ff_servo;		///< feedforward roll rate (rad/s)
	double pitch_dot_ff_servo;		///< feedforward pitch rate (rad/s)
	double yaw_dot_ff_servo;		///< feedforward yaw rate (rad/s)
	///< @}

	/** @name attitude rate servo setpoints transition rates */
	///< @{
	bool en_rpy_rate_servo_trans;	///< enable the roll pitch yaw rate transition functions
	double roll_dot_servo_tr;		///< roll angle rate transition gain
	double pitch_dot_servo_tr;		///< pitch angle rate transition gain
	double yaw_dot_servo_tr;		///< yaw angle rate transition gain
	double roll_dot_ff_servo_tr;	///< feedforward roll rate transition gain
	double pitch_dot_ff_servo_tr;	///< feedforward pitch rate transition gain
	double yaw_dot_ff_servo_tr;		///< feedforward yaw rate transition gain
	///< @}

	/** @name attitude setpoints */
	///< @{
	setpoint_gen_3D_t ATT{};		///< roll pitch yaw setpoints
	///< @}

	/** @name attitude setpoint transition rates */
	///< @{
	/*
	bool en_rpy_trans;				///< enable the roll pitch yaw transition functions
	double roll_tr;					///< roll angle transition gain
	double pitch_tr;				///< pitch angle transition gain
	double yaw_tr;					///< yaw angle transition gain
	double roll_ff_tr;				///< feedforward roll transition gain
	double pitch_ff_tr;				///< feedforward pitch transition gain
	double yaw_ff_tr;				///< feedforward yaw transition gain
	*/
	///< @}
	 
	/** @name attitude servo setpoints*/
	///< @{
	bool en_rpy_servo_ctrl;			///< enable the roll pitch yaw controllers
	double roll_servo;				///< roll angle (positive tip right) (rad)
	double pitch_servo;				///< pitch angle (positive tip back) (rad)
	double yaw_servo;				///< glabal yaw angle, positive left
	double roll_ff_servo;			///< feedforward roll angle (rad)
	double pitch_ff_servo;			///< feedforward pitch angle (rad)
	double yaw_ff_servo;
	///< @}

	/** @name attitude servo setpoint transition rates */
	///< @{
	bool en_rpy_servo_trans;		///< enable the roll pitch yaw transition functions
	double roll_servo_tr;			///< roll angle transition gain
	double pitch_servo_tr;			///< pitch angle transition gain
	double yaw_servo_tr;			///< yaw angle transition gain
	double roll_ff_servo_tr;		///< feedforward roll transition gain
	double pitch_ff_servo_tr;		///< feedforward pitch transition gain
	double yaw_ff_servo_tr;			///< feedforward yaw transition gain
	///< @}

	/** @name acceleration setpoints */
	///< @{
	setpoint_gen_3D_t XYZ_ddot{};		///< X Y Z acceleration setpoints
	///< @}

	/** @name altitude */
	///< @{
	setpoint_gen_1D_t Z{};		///< Z setpoints
	setpoint_gen_1D_t Z_dot{};		///< Z derivative setpoints
	double Z_throttle_0;	/// hover throttle

	/** @name horizontal velocity setpoint */
	///< @{
	setpoint_gen_2D_t XY_dot{};		///< XY derivative setpoints
	///< @}

	/** @name horizontal position setpoint */
	///< @{
	setpoint_gen_2D_t XY{};		///< XY setpoints
	///< @}
	/** @name horizontal position setpoint transition rates */
	///< @{
	/*
	bool en_XY_pos_ctrl_trans;		///< enable the XY transition functions
	double X_tr;					///< X transition gain
	double Y_tr;					///< Y transition gain
	*/
	///< @}
	 
	/** @name horizontal position servo setpoint */
	///< @{
	bool en_XY_servo_pos_ctrl;
	double X_servo;
	double Y_servo;
	///< @}
	/** @name horizontal position setpoint transition rates */
	///< @{
	bool en_XY_pos_servo_ctrl_trans;///< enable the XY transition functions
	double X_servo_tr;				///< X transition gain
	double Y_servo_tr;				///< Y transition gain
	///< @}


	/**
	* @brief      Initializes the setpoint manager.
	*
	* @return     0 on success, -1 on failure
	*/
	int init(void);

	/**
	* @brief      Functions to retreve inernal boolean vals.
	*
	* @return     true or false
	*/
	bool is_initialized(void);
	

	/**
	* @brief      Reset setpoints to state_estimate.
	*
	* @return     0 on success, -1 on failure
	*/

	int reset_all(void);


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

	

};

extern setpoint_t setpoint;

#endif // SETPOINT_MANAGER_H
