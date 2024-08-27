/**
 * @headerfile thrust_map.hpp
 *
 * @brief      Functions to start and stop the printf manager which is a
 *             separate thread printing data to the console for debugging.
 */


#ifndef THRUST_MAP_HPP
#define THRUST_MAP_HPP


#include <stdio.h>
#include <stdlib.h>

/**
 * enum thrust_map_t
 *
 * the user may select from the following preconfigured thrust maps
 */
typedef enum thrust_map_t{
    LINEAR_MAP,
	RS2205_2600KV_3S,
    RS2212_920_4S,
    RS2212_920_3S,
    TEAM2_PROP,
	MN1806_1400KV_4S,
	F20_2300KV_2S,
	RX2206_4S
} thrust_map_t;


/**
 * @brief      Check the thrust map for validity and populate data arrays.
 *
 * @return     0 on success, -1 on failure
 */
int thrust_map_init(thrust_map_t map);


/**
 * @brief      Corrects the motor signal m for non-linear thrust curve in place.
 *
 *
 * @param[in]  m     thrust input, must be between 0 and 1 inclusive
 *
 * @return     motor signal value on success, -1 on error
 */
double map_motor_signal(double m);


#endif // THRUST_MAP_HPP
