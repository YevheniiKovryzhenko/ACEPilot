/*
 * input_mannager.hpp
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
 * Last Edit:  07/29/2020 (MM/DD/YYYY)
 * 
 * Functions to start and stop the input manager thread which is the translation
 * beween control inputs from DSM to the user_input struct which is read by the
 * setpoint manager.
 */

#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H


#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>


#include "rc_pilot_defs.h"
#include "flight_mode.h"

/**
	* Represents current command by the user. This is populated by the
	* input_manager thread which decides to read from mavlink or DSM depending on
	* what it is receiving.
	*/
class user_input_t {
private:
	/* Most of the variables were set as private to avoid axidental overwrite 
	* Indivisual function for accessing these variables have been written.
	*/
	pthread_t thread;
	bool initialized;				///< set to 1 after input_manager_init(void)
	bool en_emergency_land;	///< on/off emergency landing procedure
	// All sticks scaled from -1 to 1
	double thr_stick;		///< positive forward
	double yaw_stick;		///< positive to the right, CW yaw
	double roll_stick;		///< positive to the right
	double pitch_stick;		///< positive forward

	arm_state_t arm_switch; ///< actual position of the physical switch
public:
	
	flight_mode_t flight_mode;		///< this is the user commanded flight_mode.
	int input_active;				///< nonzero indicates some user control is coming in
	arm_state_t requested_arm_mode;	///< set to ARMED after arming sequence is entered.
	

	/* Reading/Writing Control Sticks From Radio
	* No other functions shold be changing sticks other 
	* than input mannager. Reading can be done externally. 
	*/
	double get_thr_stick(void);
	double get_yaw_stick(void);
	double get_roll_stick(void);
	double get_pitch_stick(void);
	
	void set_thr_stick(double thr_st);
	void set_yaw_stick(double yaw_st);
	void set_roll_stick(double roll_st);
	void set_pitch_stick(double pitch_st);

	void reset_sticks(void); //sets all the sticks to their default positions

	void emergency_land(bool val);
	bool is_emergency_land_active(void);

	/* Accessing the actual switch value */
	arm_state_t get_arm_switch(void);
	void set_arm_switch(arm_state_t val); // don'y use outside input mannager

	/**
	* @brief      Starts an input manager thread.
	*
	*             Watch for new DSM data and translate into local user mode. Used
	*             in input_manager.c
	*
	* @return     0 on success, -1 on failure
	*/
	int input_manager_init(void);
	bool is_initialized(void); // a way for other functions to ask if input mannager is initialized
	void set_initialized(bool val); // use this only inside input mannager

	/**
		* @brief      Waits for the input manager thread to exit
		*
		*             This should only be called after the program flow state is set to
		*             EXITING as that's the only thing that will cause the thread to
		*             exit on its own safely. Used in input_manager.c
		*
		* @return     0 on clean exit, -1 if exit timed out.
		*/
	int input_manager_cleanup(void);
};

extern user_input_t user_input;



#endif // INPUT_MANAGER_H
