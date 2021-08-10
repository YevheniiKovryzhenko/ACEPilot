/*
 * trajectories_common.hpp
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
 * Last Edit:  08/10/2020 (MM/DD/YYYY)
 *
 * Summary :
 * Contains all the automated trajectory guidance and related functionality
 *
 */

#ifndef TRAJECTORIES_COMMON_H
#define TRAJECTORIES_COMMON_H

class cubic_guide_t
{
private:
	bool started;
	bool completed;
	double xi, xf, xdoti, xdotf;
	uint64_t time_ns;
	float tt;
public:
	void set(double new_xi, double new_xf, double new_xdoti, double new_xdotf, float new_tt);
	bool march(double& current_pos);
	void restart(void);
};

class circ_guide_t
{
private:
	bool started;
	bool completed;
	bool dir_yaw_cw;
	double T, R, omega, tt, x_i, y_i, yaw_i;
	uint64_t time_ns;
public:
	void set(double T, double R, double new_tt, bool dir_yaw_cw, double x_i, double y_i, double yaw_i);
	bool march(double& X, double& Y, double& Yaw);
	void restart(void);
};


class setpoint_guidance_t
{
private:
	bool initialized;								// initialization flag


	/* Altitude/vertical/Z setpoint guidance variables */
	bool Z_initialized;
	bool en_Z;
	bool st_Z;
	bool last_en_Z;
	uint64_t Z_time_ns;
	double Z_dot;
	double Z_initial;
	cubic_guide_t Z_cubic;

	int init_Z(void);
	int march_Z(void);
	
public:
	bool is_Z_initialized(void);
	bool get_state_Z(void);
	bool is_Z_en(void);
	void reset_Z(void);
private:
	//-------------Auto-Land---------------//
	// Landing algorithm basic logic is the following:
	// 1. run only if activated externally using "setpoint.en_Z_land=1"
	// 2. check if the drone is already on the ground
	//	2.1 reset the start-up flag & finish landing sequence
	// 3. detect the time of initiation
	// 4. decrease the altitude using a set velocity after some delay
	// should work at any altitude and is entirely velocity dependant -
	// the altitude will be decreased at a constant rate until landing has been detected
	/* Landing algorithm variables */
	bool en_land;
	bool land_finished;
	double land_start_delay_s;
	const double land_start_def_delay_s = 1.0;		// default landing delay (for stabilization)
	double V_land;									// landing velocity (from settings)
	
public:
	void start_land(void);
	int restart_land(void);
private:
	int march_land(void);
	void init_land(void);
	//use this to reset all the landing flags and parameters to defaults
	void reset_land(void);
	


	//-------------Auto-Take-off---------------//
	// Take-off algorithm basic logic is the following:
	// 1. run only if activated externally using "setpoint.en_Z_takeoff=1"
	// 2. check if the drone is has taken off to target alt.
	//	2.1 reset the start-up flag & finish take-off sequence
	// 3. detect the time of initiation
	// 4. move the setpoint.Z
	/* Takeoff algorithm variables */
	bool en_takeoff;									// takeoff running flag
	bool takeoff_finished;
	double t_takeoff_s;									// time for takeoff algorithm
	double t_min_takeoff_s;								// minimum time for takeoff algorithm (from settings)
	double takeoff_height;								// takeoff height above Z_init_takeoff
	
	void init_takeoff(void);							// initialize takeoff algorithm, only run once
	// marching the takeoff algorithm
	int march_takeoff(void);
	// use this to reset all the takeoff flags and parameters to defaults
	void reset_takeoff(void);
public:
	void start_takeoff(void);
	int restart_takeoff(void);

private:
	/* Horizontal position setpoint guidance variables */
	bool XY_initialized;
	bool en_XY;
	bool st_XY;
	bool last_en_XY;
	bool en_yaw;
	uint64_t XY_time_ns;
	uint64_t XY_waypt;			//waypoint number
	double XY_start_delay_s;
	double XY_waypt_delay_s;

	double X_initial;
	double X_dot;
	cubic_guide_t X_cubic;
	
	double Y_initial;
	double Y_dot;
	cubic_guide_t Y_cubic;

	double Yaw_initial;
	double Yaw_dot;
	
	
	int init_XY(void);
	int march_XY(void);
public:
	void reset_XY(void);
	bool get_state_XY(void);
	bool is_XY_initialized(void);
	bool is_XY_en(void);
private:
	
	


	/* -------------- AUTONOMOUS XY TRACKING-----------//
	This algorithm performs simple square trajectory using cubic polynomials.
	The basic logic is as the following:
	1. Assume AUTONOMOUS has just been activated in air
	Assuming Hover state, execute the trajectory
		1.1 using initial time and position apply dv
		1.f stabilize to hover (might need to write a separate funtion for this)
	*/
	bool en_square;
	bool square_finished;
	double square_X_offset;
	double square_X_time_s;
	double square_Y_time_s;
	double square_Y_offset;
	void init_square(void);
	int march_square(void);
public:
	void reset_square(void);
	void start_square(void);
	int restart_square(void);
private:

	/* Circular/loiter trajectory */
	bool en_circ;
	bool circ_finished;
	bool turn_dir;
	double turn_radius, turn_period, turn_time_s;
	circ_guide_t circ_guide;

	void init_circ(void);
	int march_circ(void);
	void reset_circ(void);
public:
	void start_circ(void);
	int restart_circ(void);


	/* User functions for landing algorithm */
	//change vertical velocity (can't exceed maximum from the settings)
	void set_V_land(double new_V_max);		
	
	//change landing startup timer
	void set_land_start_delay(double new_delay_s);
	

	/* User functions for takeoff algorithm */
	//sets the takeoff height (if not running)
	void set_takeoff_height(double height);

	/* Initialization, call at the start of the program */
	int init(void);

	/* 
	Marching the auto guidance algorithms 
	Call this function at each itteration before feedback.
	Start deactivated, use flags to start a certain algorithm.
	Each algorithm dectivates once finished
	*/
	int march(void);

	/*
	* External reset of all the autonoumous functions, 
	* can be use to stop any autonous algorithm and/or
	* in emergency.
	*/
	void reset(void);
};

extern setpoint_guidance_t setpoint_guidance;

/**
	* @brief		circular trajectory
	*
	* @return     uses turn radius, period and time elapsed
	*/
	//void circ_traj(double R, double T, double dt);

	/**
	* @brief		automated descent to landing
	*
	* @return     uses velocity and maximum XY error
	*/
//int AUTO_LAND(void);


//int AUTO_TAKEOFF(void);

/**
	* @brief		guidance allong a rectangular trajectory
	*
	* @return     uses time and XY coordinates of each corner
	*/
//int XY_SQUARE(double dX, double dY, double t_X, double t_Y);

/**
	* @brief		guidance allong a circular trajectory
	*
	* @return     uses turn radius, period and time elapsed
	*/
//int XY_CIRC(double R, double T);

/**
	* @brief		simple liftoff and hover testing
	*
	* @return     uses automated liftoff to a set altitude
	*/
//void AUTO_LIFTOFF_HOWER_TEST(void);

/**
	* @brief		testing guidance allong a rectangular trajectory
	*
	* @return     uses automated rectangular trajectory with ascent and descent
	*/
//void AUTO_XY_SQUARE_TEST(void);

/**
	* @brief		testing guidance allong a circular trajectory
	*
	* @return     uses turn radius, period and time elapsed
	*/
//void AUTO_XY_CIRC_TEST(void);


#endif /* TRAJECTORIES_COMMON_H */