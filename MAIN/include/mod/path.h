/**
 * <path.h>
 *
 * @brief   Functions to read the waypoint file and handle the path
 *
 * Program expects waypoint file of the form (no line numbers in actual file):
 *
 * ~~~
 * 1:    t x y z xd yd zd r p y rd pd yd
 * 2:    t x y z xd yd zd r p y rd pd yd
 *         ...
 * n:    t x y z xd yd zd r p y rd pd yd
 * ~~~
 *
 * where line i contains waypoint i with position, velocity and time respectively.
 * Memory will be dynamically allocated for the path and the path will be stored
 * as a set of waypoints defined by the file
 *
 *
 * @author Glen Haggin (ghaggin@umich.edu)
 *
 * @{
 */

#ifndef __PATH__
#define __PATH__

#include <stddef.h>
#include <fcntl.h>  // for F_OK
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for access()

#include "coordinates.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief       Read waypoint file and initialize path
 *
 * Checks for a valid file and counts the number of waypoints specified.  Based
 * on the number of waypoints, memory for the path is dynamically allocated (freed in cleanup).
 * Waypoints are then sequentially read into the path.  If any invalid waypoints are specified,
 * i.e. not in the form <x y z xd yd zd t> (all floats), then the intialization fails and the
 * path stays unitialized.
 *
 * @param[in]   file_path   string containing the relative path to the waypoint file
 *
 * @return      0 on success, -1 on failure
 */
int path_load_from_file(const char* file_path);

/**
 * @brief       Frees memory allocated in path and "unitializes" path variable
 */
void path_cleanup();

typedef struct path_t
{
    waypoint_t* waypoints;  ///< pointer to head of the waypoint array
    waypoint_t waypoints_init; ///pointer to the head of the initial waypoint array
    size_t len;             ///< length of the path (number of waypoints)

    int initialized;  ///< 1 if initialized, 0 if uninitialized
} path_t;

/**
 * @brief       Initial values for path_t
 */
#define PATH_INITIALIZER                              \
    {                                                 \
        .waypoints = NULL, .len = 0, .initialized = 0 \
    }

extern path_t path;

#define TIME_TRANSITION_FLAG 0
#define POS_TRANSITION_FLAG 1

#ifdef __cplusplus
}
#endif

#endif /*__PATH__ */

/* @} Waypoints */