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
 * Last Edit:  07/29/2020 (MM/DD/YYYY)
 *
 * Summary :
 * Contains all the automated trajectory guidance and related functionality
 *
 */

#ifndef TRAJECTORIES_COMMON_H
#define TRAJECTORIES_COMMON_H


/**
	* @brief		uses a cubic pol. to connect two points in space using initial and final
	*				constraints
	*
	* @return     possition (m) based on time elapsed dt and total time tt_s (in seconds)
	*/
double cubicPol(double xi, double xf, double xdoti, double xdotf, float tt_s, double dt);


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
int AUTO_LAND(void);

/**
	* @brief		automated takeoff to a set elevation above the current altitude
	*
	* @return     altitude and time
	*/
int AUTO_TAKEOFF(void);

/**
	* @brief		guidance allong a rectangular trajectory
	*
	* @return     uses time and XY coordinates of each corner
	*/
int XY_SQUARE(double dX, double dY, double t_X, double t_Y);

/**
	* @brief		guidance allong a circular trajectory
	*
	* @return     uses turn radius, period and time elapsed
	*/
int XY_CIRC(double R, double T);

/**
	* @brief		simple liftoff and hover testing
	*
	* @return     uses automated liftoff to a set altitude
	*/
void AUTO_LIFTOFF_HOWER_TEST(void);

/**
	* @brief		testing guidance allong a rectangular trajectory
	*
	* @return     uses automated rectangular trajectory with ascent and descent
	*/
void AUTO_XY_SQUARE_TEST(void);

/**
	* @brief		testing guidance allong a circular trajectory
	*
	* @return     uses turn radius, period and time elapsed
	*/
void AUTO_XY_CIRC_TEST(void);


#endif /* TRAJECTORIES_COMMON_H */