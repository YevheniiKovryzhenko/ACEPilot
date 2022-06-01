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
 * Last Edit:  06/01/2022 (MM/DD/YYYY)
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

	/**
	* Function only used locally
	*/
	int init_all_filters(void);
	int update_setpoints(void);

	/**
	* @brief      Configure setpoints to reset to state_estimate.
	*
	* @return     0 on success, -1 on failure
	*/
	int set_reset_sources(void);
	int set_reset_sources_all_defs(void); //all defs to reset to when armed

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
	void update_XY_vel(void);
	void update_rpy_servo(void);
public:

	/** @name direct passthrough
	* user inputs tranlate directly to mixing matrix
	*/
	///< @{
	setpoint_gen_3D_t POS_throttle{};	///< final value which is being actually used for control mixing
	setpoint_gen_3D_t ATT_throttle{};	///< final value which is being actually used for control mixing

	setpoint_gen_3D_t POS_throttle_servo{};	///< final value which is being actually used for control mixing
	setpoint_gen_3D_t ATT_throttle_servo{};	///< final value which is being actually used for control mixing
	///< @}

	/** @name attitude rate setpoints */
	///< @{
	setpoint_gen_3D_t ATT_dot{};	///< roll pitch yaw derivative setpoints
	setpoint_gen_3D_t ATT_dot_servo{};	///< roll pitch yaw derivative setpoints
	///< @}

	/** @name attitude setpoints */
	///< @{
	setpoint_gen_3D_t ATT{};		///< roll pitch yaw setpoints
	setpoint_gen_3D_t ATT_servo{};		///< roll pitch yaw setpoints
	///< @}

	/** @name acceleration setpoints */
	///< @{
	setpoint_gen_3D_t XYZ_ddot{};			///< X Y Z acceleration setpoints
	setpoint_gen_3D_t XYZ_ddot_servo{};		///< X Y Z acceleration setpoints
	///< @}

	/** @name altitude */
	///< @{
	setpoint_gen_1D_t Z{};					///< Z setpoints
	setpoint_gen_1D_t Z_dot{};				///< Z derivative setpoints
	double Z_throttle_0;					/// hover throttle

	setpoint_gen_1D_t Z_servo{};			///< Z setpoints
	setpoint_gen_1D_t Z_dot_servo{};		///< Z derivative setpoints

	/** @name horizontal velocity setpoint */
	///< @{
	setpoint_gen_2D_t XY_dot{};				///< XY derivative setpoints
	setpoint_gen_2D_t XY_dot_servo{};		///< XY derivative setpoints
	///< @}

	/** @name horizontal position setpoint */
	///< @{
	setpoint_gen_2D_t XY{};		///< XY setpoints
	setpoint_gen_2D_t XY_servo{};			///< XY setpoints
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
	* @brief      Reset setpoint mannager to its starting state.
	* 
	*	Intended to be called each time the system is armed.
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
