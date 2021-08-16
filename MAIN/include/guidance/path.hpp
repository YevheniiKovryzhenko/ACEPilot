/*
 * path.hpp
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
 * Last Edit:  08/16/2020 (MM/DD/YYYY)
 *
 * Summary :
 * Functions to read the waypoint file and handle the path
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
 * Where line i contains waypoint i with position, velocity and time respectively.
 * Memory will be dynamically allocated for the path and the path will be stored
 * as a set of waypoints defined by the file. 
 * Added a veson without dynamic memory allocation
 * 
 * Based on the original work of
 * Glen Haggin (ghaggin@umich.edu)
 */

#ifndef __PATH__
#define __PATH__
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "coordinates.h"
#include "setpoint_manager.hpp"
#include "state_machine.hpp"


class path_t
{
private:
    waypoint_t* waypoints;  ///< pointer to head of the waypoint array
    waypoint_t waypoint;  ///< single waypoint
    waypoint_t waypoints_init; ///pointer to the head of the initial waypoint array
    size_t len;             ///< length of the path (number of waypoints)
    int cur_waypoint_num;

    FILE* fd_NH;
    bool initialized;  ///< 1 if initialized, 0 if uninitialized
    bool loaded;
    bool reached_EOF;
    bool en;
    bool last_en;
    uint64_t time_ns;
    double waypoint_time_s;

    void reset(void);

    /**
    * @brief   Count the number of lines in a file, indicates number of waypoints
    *
    * @return  Number of lines in the file
    */
    int count_file_lines(const char* file_path);

    /**
    * @brief   Read all of the waypoints from a file into the path variable
    *
    * @return  0 on success, -1 on failure
    */
    int read_waypoints(FILE* fd);
    int read_waypoint_NH(void); //no heap version

    /**
    * @brief       Frees memory allocated in path and "unitializes" path variable
    */
    void path_cleanup(void);
    void cleanup_NH(void); //no heap version

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
    int load_file_NH(const char* file_path); //no heap version

    
public:

    /**
    * @brief   Externally order setpoint manager to follow new path from path_file
    *
    * @return  0 on success, -1 if unsuccessful
    */
    int set_new_path(const char* file_name);
    int set_new_path_NH(const char* file_name);
    
    /**
    * @brief Update the setpoint for the next waypoint
    */
    int update_setpoint_from_waypoint(setpoint_t& cur_setpoint, state_machine_t& state_machine);
    int update_setpoint_from_waypoint_NH(setpoint_t& cur_setpoint, state_machine_t& state_machine);
    
    int start_waypoint_counter(setpoint_t& init_setpoint);
    int start_waypoint_counter_NH(setpoint_t& init_setpoint);

    int init(void);

    bool is_en(void);
    void stop(void);
};

extern path_t path;

#define TIME_TRANSITION_FLAG 0
#define POS_TRANSITION_FLAG 1

#endif /*__PATH__ */

/* @} Waypoints */