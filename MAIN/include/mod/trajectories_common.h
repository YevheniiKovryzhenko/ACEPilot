/**
 * @file trajectories_common.h
 */

#ifndef TRAJECTORIES_COMMON_H
#define TRAJECTORIES_COMMON_H

#include <math.h>

#include <rc/time.h>

#include "rc_pilot_defs.h"
#include "setpoint_manager.h"
#include "settings.h"
#include "tools.h"
#include "input_manager.h"




#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

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


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* TRAJECTORIES_COMMON_H */