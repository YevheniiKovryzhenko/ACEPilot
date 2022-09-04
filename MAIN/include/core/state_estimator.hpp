/*
 * state_estimator.hpp
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
 * Last Edit:  09/03/2022 (MM/DD/YYYY)
 *
 * Summary :
 * This contains all the primary functionality and framework for state estimation rountines
 * 
 * This runs at the same rate as the feedback controller.
 * state_estimator_march() is called immediately before  feedback_march() in the
 * IMU interrupt service routine.
 */

#ifndef STATE_ESTIMATOR_HPP
#define STATE_ESTIMATOR_HPP
#include <stdint.h> // for uint64_t
#include <stdio.h>
#include <math.h>
#include <rc/math/filter.h>
#include <rc/mpu.h>
#include <rc/bmp.h>

//#include "coordinate_frames_gen.hpp"
#include "signal_filter_gen.hpp"
#include "settings.hpp"
	/**
	 * This is the output from the state estimator. It contains raw sensor values
	 * and the outputs of filters. Everything is in NED coordinates defined as:
	 *
	 * - X pointing Rorward
	 * - Y pointing Right
	 * - Z pointing Down
	 *
	 * right hand rule applies for angular values such as tait bryan angles and gyro
	 * - Positive Roll to the right about X
	 * - Positive Pitch back about Y
	 * - Positive Yaw right about Z
	 */
	class state_estimate_t
	{
	private:
		bool initialized = false;
		uint64_t time;
		rc_bmp_data_t bmp_data;

		int bmp_sample_counter = 0;		

		/** @name Primary Estimate
		 * This is the global estimated position, velocity, acceleration, attitude, etc.
		 * These are the values that are actually used for control system 
		 * and other susbystems.
		 *
		 * These values come from variety of sourses: IMU, mocap, GPS, etc.
		 * But are cosidered FINAL and whatever the sensorfusion/estimation
		 * algorithm was used, one should chosed and recorded here
		 * 
		 * All assume NED coordinate frame, except for altitude (it is up, obviously)
		 */
		 ///@{
		double pos_global[3];		///< (m)
		double vel_global[3];		///< (m/s)
		double accel_global[3];		///< (m/s2)
		double pos_relative[3];		///< (m)
		double vel_relative[3];		///< (m/s)
		double accel_relative[3];	///< (m/s2)
	
		double quat[4];				///< attitude in quaternians qw qx qy qz
		double att[3];				///< Euler roll pitch yaw (rad)
		double continuous_yaw;		///< Heading/Yaw angle which can be greater than +-pi
		double att_rates[3];		///< Euler angle rates (rad/s)
		double omega[3];			///< Attitude rates about the body frame / gyro (rad/s)

		double v_batt;				///< Battery voltage (V)
		double Temp;				///< Ambient Temperature (c)
		
		/* Vertical parameters */
		double alt;					///< Altitude (from initialization point) (m)
		double alt_vel;				///< Vertical velocity (m/s)
		double alt_acc;				///< Vertical acceleration (m)
		///@}


		//int counter;
		//int rev[4]; //for encoders

		/**
		* @brief       Updates and Marches all the sourses
		*
		* Runns update routines for all the sensors and external sourses
		* of data for state estimation. This does not actually update
		* state estimator itself, but makes sure new data is available.
		*
		* @param[in]   void
		*
		* @return      0 on success, -1 on failure
		*/
		char update_all_sourses(void);

		/**
		* @brief       Fetches data from IMU and internal sensors
		*
		* Updates estimator from IMU and on-board sensors, 
		* also selects proper sourses.
		* Since this is the first update, it assumed to be the least accurate,
		* so the subsequent ones might overwrite these.
		* We want to update as many things as possible, in case we don't
		* have any other sourses.
		*
		* @param[in]   void
		*
		* @return      void
		*/
		void fetch_internal_sourses(void);

		/**
		* @brief       Fetches data from MOCAP, GPS and other external sources
		*
		* Updates estimator from external sourses such as GPS and mocap, 
		* also selects proper sourses.
		*
		* @param[in]   void
		*
		* @return      void
		*/
		void fetch_external_sourses(void);		
		

		/**
		* @brief       Initializes internal filters
		*
		* @param[in]   void
		*
		* @return      0 on success, -1 on failure
		*/
		char init_internal_filters(void);

		/**
		* @brief       Updates internal KF and EKF algorithms
		*
		* @param[in]   void
		*
		* @return      void
		*/
		void update_internal_filters(void);


		/**
		* @brief       Cleanup routine for internal filters
		*
		* @param[in]   void
		*
		* @return      void
		*/
		void cleanup_internal_filters(void);

	public:

		/**
		* @brief      Initial setup of the state estimator
		*
		* barometer must be initialized first
		*
		* @return     0 on success, -1 on failure
		*/
		int init(void);
		bool is_initialized(void);


		/**
		 * @brief      March state estimator forward one step
		 *
		 * Called immediately before feedback_march
		 *
		 * @return     0 on success, -1 on failure
		 */
		int march(void);


		/**
		 * @brief      jobs the state estimator must do after feedback_controller
		 *
		 * Called immediately after feedback_march in the ISR. Currently this reads
		 *
		 * @return     0 on success, -1 on failure
		 */
		int march_jobs_after_feedback(void);


		/**
		 * @brief      Cleanup the state estimator, freeing memory
		 *
		 * @return     0 on success, -1 on failure
		 */
		int cleanup(void);



		/* Output functions
		* These should always return the most accurate and most recent value.
		* This also makes sure that all internal values are read-only. No
		* external function can modify state estimator values.
		*/
		uint64_t get_time(void);

		double get_roll(void);
		double get_pitch(void);
		double get_yaw(void);
		void get_att_tb(double* buff);
		void get_att_quat(double* buff);
		double get_continuous_heading(void);

		double get_roll_dot(void);
		double get_pitch_dot(void);
		double get_yaw_dot(void);
		void get_att_rates(double* buff);

		double get_p(void);
		double get_q(void);
		double get_r(void);
		void get_omega(double* buff);

		double get_X(void);
		double get_Y(void);
		double get_Z(void);
		void get_pos_glob(double* buff);

		double get_x(void);
		double get_y(void);
		double get_z(void);
		void get_pos_rel(double* buff);

		double get_X_vel(void);
		double get_Y_vel(void);
		double get_Z_vel(void);
		void get_vel_glob(double* buff);

		double get_x_vel(void);
		double get_y_vel(void);
		double get_z_vel(void);
		void get_vel_rel(double* buff);

		double get_X_acc(void);
		double get_Y_acc(void);
		double get_Z_acc(void);
		void get_acc_glob(double* buff);

		double get_x_acc(void);
		double get_y_acc(void);
		double get_z_acc(void);
		void get_acc_rel(double* buff);

		double get_alt(void);
		double get_alt_vel(void);
		double get_alt_acc(void);

		double get_v_batt(void);

		double get_temp(void);

	};

	extern state_estimate_t state_estimate;
	extern rc_mpu_data_t mpu_data;


	/** @name Logging class for state estimator
	* Defines how logging should be done for this class
	*/
	class state_estimator_log_entry_t
	{
	private:
		uint64_t time;

		double pos_global[3];		///< (m)
		double vel_global[3];		///< (m/s)
		double accel_global[3];		///< (m/s2)
		double pos_relative[3];		///< (m)
		double vel_relative[3];		///< (m/s)
		double accel_relative[3];	///< (m/s2)

		double quat[4];				///< attitude in quaternians qw qx qy qz
		double att[3];				///< Euler roll pitch yaw (rad)
		double continuous_yaw;		///< Heading/Yaw angle which can be greater than +-pi
		double att_rates[3];		///< Euler angle rates (rad/s)
		double omega[3];			///< Attitude rates about the body frame / gyro (rad/s)

		double v_batt;				///< Battery voltage (V)
		double Temp;				///< Ambient Temperature (c)

		/* Vertical parameters */
		double alt;					///< Altitude (from initialization point) (m)
		double alt_vel;				///< Vertical velocity (m/s)
		double alt_acc;				///< Vertical acceleration (m)
		///@}

		char print_vec(FILE* file, double* vec_in, int size);
		char print_header_vec(FILE* file, const char* var_name, int size);
	public:
		char update(state_estimate_t* new_state);
		char print_header(FILE* file);
		char print_entry(FILE* file);
	};

#endif //  STATE_ESTIMATOR_HPP