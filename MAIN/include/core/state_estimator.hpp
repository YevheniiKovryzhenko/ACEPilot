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
 * Last Edit:  08/29/2022 (MM/DD/YYYY)
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
#include "settings.h"
#include "sensors_gen.hpp"
#include "mocap_gen.hpp"
#include "EKF.hpp"
#include "EKF2.hpp"

	typedef struct ext_mag_t
	{
		double x;
		double y;
		double z;
		double norm;
	}ext_mag_t;

	extern ext_mag_t ext_mag;

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

		// altitude filter components
		rc_kalman_t alt_kf = RC_KALMAN_INITIALIZER;
		rc_vector_t u = RC_VECTOR_INITIALIZER;
		rc_vector_t y = RC_VECTOR_INITIALIZER;
		double alt_kf_est = 0.0;
		double vertical_speed_kf_est = 0.0;
		double vertical_acc_kf_est = 0.0;

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

		//EOL{
		//int counter;
		//int rev[4]; //for encoders

		/** @name IMU (accel gyro)
		 * Normalized Quaternion is straight from the DMP but converted to NED
		 * coordinates. Tait-Bryan angles roll pitch and yaw angles are then
		 * converted from the quaternion.
		 * the roll_pitch_yaw values in the taid bryan angles tb_imu are bounded
		 * by +-pi since they come straight from the quaternion. the state estimator
		 * keeps track of these rotations and generates continuous_yaw which is
		 * unbounded and keeps track of multiple rotations. This provides a continuously
		 * differentiable variable with no jumps between +-pi
		 */
		 ///@{
		//double gyro[3];		///< gyro roll pitch yaw (rad/s)
		//double accel[3];	///< accel XYZ NED coordinates (m/s^2)
		//double quat_imu[4];	///< DMP normalized quaternion
		//double tb_imu[3];	///< tait bryan roll pitch yaw angle (rad)
		//double imu_continuous_yaw; ///< continuous yaw from imu only (multiple turns)
		//double accel_ground_frame[3]; ///< imu accel rotated with gravity subtracted off
		///@}

		/** @name IMU (magnetometer)
		 * these values are only set when magnetometer is enabled in settings.
		 * right now these aren't used and we don't suggest turning the magnetometer on
		 */
		 ///@{
		/*
		double mag[3];		///< magnetometer XYZ NED coordinates ()
		double mag_heading_raw;	///< raw compass heading
		double mag_heading;	///< compass heading filtered with IMU
		double mag_heading_continuous;
		double quat_mag[4];	///< quaterion filtered
		double tb_mag[3];	///< roll pitch yaw with magetometer heading fixed (rad)
		*/
		///@}

		/** @name selected values for feedback
		* these are copoies of other values in this state estimate used for feedback
		* this is done so we can easily chose which source to get feedback from (mag or no mag)
		*/
		///@{
		//double roll;
		//double pitch;
		//double yaw;
		//double continuous_yaw;	///<  keeps increasing/decreasing above +-2pi
		//double roll_dot;
		//double pitch_dot;
		//double yaw_dot;
		/*
		double roll_dot_raw;
		double pitch_dot_raw;
		double yaw_dot_raw;
		double X; //Inertial
		double Y; //Inertial
		double Z; //Inertial
		double X_dot;
		double Y_dot;
		double Z_dot;
		double X_dot_raw;	// d/dt position based estimate of velocity 
		double Y_dot_raw;	// d/dt position based estimate of velocity 
		double Z_dot_raw;	// d/dt position based estimate of velocity 
		double Z_ddot; // transformed z accel
		*/
		///@}


		/** @name filtered data from IMU & barometer
		 * Altitude estimates from kalman filter fusing IMU and BMP data.
		 * Alttitude, velocity, and acceleration are in units of m, m/s, m/s^2
		 * Note this is altitude so positive is upwards unlike the NED
		 * coordinate frame that has Z pointing down.
		 */
		 ///@{
		/*
		double bmp_pressure_raw;///< raw barometer pressure in Pascals
		double alt_bmp_raw;	///< altitude estimate using only bmp from sea level (m)
		double alt_bmp;		///< altitude estimate using kalman filter (IMU & bmp)
		double alt_bmp_vel;	///< z velocity estimate using kalman filter (IMU & bmp)
		double alt_bmp_accel;	///< z accel estimate using kalman filter (IMU & bmp)
		*/
		///@}

		/** @name Motion Capture data
		 * As mocap drop in and out the mocap_running flag will turn on and off.
		 * Old values will remain readable after mocap drops out.
		 */
		 ///@{
		bool mocap_running = false;	///< true if motion capture data is recent and valid
		uint64_t mocap_timestamp_ns; ///< timestamp of last received packet in nanoseconds since boot
		uint32_t mocap_time;	///< time receved from mocap
		/*
		double pos_mocap[3];	///< position in mocap frame, converted to NED if necessary
		double quat_mocap[4];	///< UAV orientation according to mocap
		double tb_mocap[3];	///< Tait-Bryan angles according to mocap
		int is_active;  ///< TODO used by mavlink manager, purpose unclear... (pg)
		double mocap_continuous_yaw;
		*/
		///@}

		//EOL}

		char init_altitude_kf(void);
		void march_altitude_kf(double* Z_est, double Z_acc, double Z);
		void cleanup_altitude_kf(void);

		void mocap_check_timeout(void);

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
		* @brief       Updates internal KF and EKF algorithms
		*
		* @param[in]   void
		*
		* @return      void
		*/
		void update_internal_filters(void);

	public:
		/* Define all the sensors and external input sources */
		// Assume these are primary:
		battery_gen_t batt{};	//battery voltage sensor
		barometer_gen_t bmp{};	//barometer
		IMU_9DOF_gen_t imu{};	//IMU-9DOF with Gyro + Accel + Mag
		mocap_gen_t mocap{};	//mocap system

		EKF_t EKF{};			//EKF for attitude estimation V-1
		EKF2_t EKF2{};			//EKF for attitude estimation V-2


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