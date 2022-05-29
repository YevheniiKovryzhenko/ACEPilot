/*
 * path.cpp
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
 * Last Edit:  05/28/2022 (MM/DD/YYYY)
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

#include <stddef.h>
#include <fcntl.h>  // for F_OK
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for access()

#include "coordinates.h"
#include "setpoint_manager.hpp"
#include "tools.h"
#include "state_machine.hpp"

#include "path.hpp"

 // preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

path_t path{};

/**
 * @brief   Count the number of lines in a file, indicates number of waypoints
 *
 * @return  Number of lines in the file
 */
int path_t::count_file_lines(const char* file_path)
{
    FILE* fd = fopen(file_path, "r");
    int c = 0;
    size_t count = 0;

    c = getc(fd);
    while (c != EOF)
    {
        if (c == '\n' || c == EOF)
        {
            ++count;
        }
        c = getc(fd);
    }

    fclose(fd);
    return count;
}

/**
 * @brief   Read all of the waypoints from a file into the path variable
 *
 * @return  0 on success, -1 on failure
 */
int path_t::read_waypoints(FILE* fd)
{
    int rcount = 0;
    int waypoint_num = 0;

    while (rcount != EOF)
    {
        // Read formated file line (13 doubles and 1 int)
        rcount = fscanf(fd, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
            &waypoints[waypoint_num].t, &waypoints[waypoint_num].x,
            &waypoints[waypoint_num].y, &waypoints[waypoint_num].z,
            &waypoints[waypoint_num].xd, &waypoints[waypoint_num].yd,
            &waypoints[waypoint_num].zd, &waypoints[waypoint_num].roll,
            &waypoints[waypoint_num].pitch, &waypoints[waypoint_num].yaw,
            &waypoints[waypoint_num].p, &waypoints[waypoint_num].q,
            &waypoints[waypoint_num].r, &waypoints[waypoint_num].flag);

        // If not end of file, but an invalid read (waypoints have 14 values)
        if (rcount != EOF && rcount != 14)
        {
            fprintf(stderr, "ERROR: invalid waypoint read from line: %i\n",
                waypoint_num + 1);  // lines 1 indexed, waypoints zero indexed
            return -1;
        }

        // Increment line number for next iteration
        ++waypoint_num;
    }
    return 0;
}

int path_t::read_waypoint_NH(FILE* fd)
{
    if (unlikely(!loaded))
    {
        printf("\nERROR in read_waypoint_NH not loaded");
        return -1;
    }

    if (reached_EOF)
    {
        return 0;
    }
    
    // Read formated file line (13 doubles and 1 int)
    int rcount = fscanf(fd, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ", \
        &waypoint.t, &waypoint.x, \
        &waypoint.y, &waypoint.z, \
        &waypoint.xd, &waypoint.yd, \
        &waypoint.zd, &waypoint.roll, \
        &waypoint.pitch, &waypoint.yaw, \
        &waypoint.p, &waypoint.q, \
        &waypoint.r, &waypoint.flag);

    // If not end of file, but an invalid read (waypoints have 14 values)
    if (rcount != EOF && rcount != 14)
    {
        fprintf(stderr, "ERROR: invalid waypoint read from line\n");  // lines 1 indexed, waypoints zero indexed
        return -1;
    }

    if (rcount == EOF)
    {
        reached_EOF = true;
    }

    return 0;
}

int path_t::path_load_from_file(const char* file_path)
{
    // Clear any previously stored path, set init to 0
    cleanup();

    // Check for valid file
    if (access(file_path, F_OK) != 0)
    {
        fprintf(stderr, "ERROR: waypoint file missing\n");
        return -1;
    }

    // Count number of waypoints contained in file
    len = count_file_lines(file_path);

    // Open file for waypoint reading
    FILE* fd = fopen(file_path, "r");

    // Read path size and allocate waypoint memory
    waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * len);
    if (waypoints == NULL)
    {
        fprintf(stderr, "ERROR: failed allocating memory for path\n");
        return -1;
    }

    // Read waypoints from file
    if (read_waypoints(fd) < 0)
    {
        cleanup(); //Added to prevent potential memory leak
        fprintf(stderr, "ERROR: failed reading waypoint file\n");
        return -1;
    }
    loaded = true;
    fclose(fd);
    return 0;
}

int path_t::load_file_NH(const char* file_path)
{
    // no heap version

    // Check for valid file
    if (access(file_path, F_OK) != 0)
    {
        fprintf(stderr, "ERROR: waypoint file missing\n");
        return -1;
    }

    // Count number of waypoints contained in file
    len = count_file_lines(file_path);

    // Open file for waypoint reading
    fd_NH = fopen(file_path, "r");
    

    loaded = true;
    //printf("\nSuccesfully loaded path %s", &file_path);
    return 0;
}

void path_t::cleanup(void)
{
    if (waypoints != NULL) free(waypoints);
    if (fd_NH != NULL) fclose(fd_NH);
    reset();
    return;
}

int path_t::init(void)
{
    if (!initialized)
    {
        reset();

        initialized = true;
        return 0;
    }
    else
    {
        printf("\nERROR in init: already initialized");
        return -1;
    }
}

void path_t::reset(void)
{
    len                 = 0;
    loaded              = false;
    en                  = false;
    last_en             = false;
    cur_waypoint_num    = 0;
    waypoint_time_s     = 0.0;
    reached_EOF         = false;
    waypoints           = NULL;
    fd_NH               = NULL;

    return;
}



/**
* @brief   Externally order setpoint manager to follow new path from path_file
*
* @return  0 on success, -1 if unsuccessful
*/
int path_t::set_new_path(const char* file_name)
{
    if (unlikely(!initialized))
    {
        printf("\nERROR in set_new_path: not initialized");
        return -1;
    }
    cleanup();

    if (path_load_from_file(file_name) == -1)
    {
        fprintf(stderr, "ERROR: could not load new path file\n");
        return -1;
    }
    return 0;
}

int path_t::set_new_path_NH(const char* file_name)
{
    if (unlikely(!initialized))
    {
        printf("\nERROR in set_new_path_NH: not initialized");
        return -1;
    }
    cleanup();

    if (load_file_NH(file_name) == -1)
    {
        fprintf(stderr, "ERROR: could not load new path file\n");
        return -1;
    }
    return 0;
}



/**
* @brief   Logic for starting to follow path, reset time and waypoint counter
*/
int path_t::start_waypoint_counter_NH(setpoint_t &init_setpoint)
{
    if (unlikely(!initialized))
    {
        printf("\nERROR in start_waypoint_counter_NH: not initialized");
        return -1;
    }
    if (unlikely(!loaded))
    {
        printf("\nERROR in start_waypoint_counter_NH: path not loaded");
        cleanup();
        return -1;
    }
    // If this is the first time in autonomous mode and armed, save the current time
    if (!last_en)
    {
        // If the system is armed and autonomous mode is set, record time in
        // time_auto_set
        last_en = true;
        time_ns = rc_nanos_since_boot();

        waypoints_init.x = init_setpoint.XY.x.value.get();
        waypoints_init.y = init_setpoint.XY.y.value.get();
        waypoints_init.z = init_setpoint.Z.value.get();
        waypoints_init.roll = init_setpoint.ATT.x.value.get();
        waypoints_init.pitch = init_setpoint.ATT.y.value.get();
        /*
        if (fabs(init_setpoint.yaw - state_estimate.continuous_yaw) >= M_PI / 18.0)
        {
            init_setpoint.yaw = state_estimate.continuous_yaw;
            printf("\nWARNING: High yaw error, overwriting setpoint");
        }
        */
        waypoints_init.yaw = init_setpoint.ATT.z.value.get();

        // read new waypoint from file:
        if (unlikely(read_waypoint_NH(fd_NH) == -1))
        {
            printf("ERROR in start_waypoint_counter_NH: failed to read 0th waypoint");
            cleanup();
        }
        en = true;
    }
    return 0;
}

int path_t::start_waypoint_counter(setpoint_t& init_setpoint)
{
    if (unlikely(!initialized))
    {
        printf("\nERROR in start_waypoint_counter: not initialized");
        return -1;
    }
    if (unlikely(!loaded))
    {
        printf("\nERROR in start_waypoint_counter: path not loaded");
        cleanup();
        return -1;
    }

    // If this is the first time in autonomous mode and armed, save the current time
    if (!last_en)
    {
        // If the system is armed and autonomous mode is set, record time in
        // time_auto_set
        last_en = true;
        time_ns = rc_nanos_since_boot();

        waypoints_init.x = init_setpoint.XY.x.value.get();;
        waypoints_init.y = init_setpoint.XY.y.value.get();;
        waypoints_init.z = init_setpoint.Z.value.get();
        waypoints_init.roll = init_setpoint.ATT.x.value.get();;
        waypoints_init.pitch = init_setpoint.ATT.y.value.get();;
        /*
        if (fabs(init_setpoint.yaw - state_estimate.continuous_yaw) >= M_PI / 18.0)
        {
            init_setpoint.yaw = state_estimate.continuous_yaw;
            printf("\nWARNING: High yaw error, overwriting setpoint");
        }
        */
        waypoints_init.yaw = init_setpoint.ATT.z.value.get();;
        en = true;
    }

    return 0;
}


/**
* @brief Update the setpoint for the next waypoint
*/
int path_t::update_setpoint_from_waypoint(setpoint_t& cur_setpoint, state_machine_t& state_machine)
{
    if (unlikely(!initialized))
    {
        printf("\nERROR in update_setpoint_from_waypoint: not initialized");
        return -1;
    }
    if (unlikely(!loaded))
    {
        printf("\nERROR in update_setpoint_from_waypoint: trying to update when not file is not loaded");
        cleanup();
        return 0;
    }
    
    // Break out of function if the current waypoint is the last point in the path
    if (cur_waypoint_num == len)
    {
        printf("\nTrajectory completed");
        cleanup();
        return 0;
    }

    if (unlikely(reached_EOF))
    {
        //should never reach this, just a failsafe
        printf("ERROR in update_setpoint_from_waypoint: reached EOF");
        cleanup();
        return -1;
    }

    //update time since activation:
    waypoint_time_s = finddt_s(time_ns);

    // Parse waypoint flag
    switch ((int)waypoints[cur_waypoint_num].flag)
    {
    case TIME_TRANSITION_FLAG:
        // Check if there are additional waypoints and advnace control
        // to the next waytoint if it is time to do so.  If there are no additional waypoints,
        // keep controlling to the previous point
        if (cur_waypoint_num < (len - 1) &&
            waypoint_time_s >= waypoints[cur_waypoint_num + 1].t)
        {
            ++cur_waypoint_num;

            // Set the desired x, y, and z if allowed
            if (state_machine.is_en())
            {
                cur_setpoint.XY.x.value.set(waypoints_init.x + waypoints[cur_waypoint_num].x);
                cur_setpoint.XY.y.value.set(waypoints_init.y + waypoints[cur_waypoint_num].y);
                cur_setpoint.Z.value.set(waypoints_init.z + waypoints[cur_waypoint_num].z);
                cur_setpoint.ATT.x.value.set(waypoints_init.roll + waypoints[cur_waypoint_num].roll);
                cur_setpoint.ATT.y.value.set(waypoints_init.pitch + waypoints[cur_waypoint_num].pitch);
                cur_setpoint.ATT.z.value.set(waypoints_init.yaw + waypoints[cur_waypoint_num].yaw);
            }

        }
        break;
    case POS_TRANSITION_FLAG:
        // TODO: determine position error and compare to convergence tolerance
        //       (? who sets/determines/stores convergence tolerance ?)
        //assert(0);
        break;
    default:
        fprintf(stderr, "ERROR: unrecognized waypoint flag\n");
        return -1;
    }
    return 0;
}

int path_t::update_setpoint_from_waypoint_NH(setpoint_t& cur_setpoint, state_machine_t& state_machine)
{
    if (unlikely(!initialized))
    {
        printf("\nERROR in update_setpoint_from_waypoint_NH: not initialized");
        return -1;
    }
    if (unlikely(!loaded))
    {
        printf("\nERROR in update_setpoint_from_waypoint_NH: trying to update when not file is not loaded");
        cleanup();
        return 0;
    }

    // Break out of function if the current waypoint is the last point in the path
    if (cur_waypoint_num >= len)
    {
        printf("\nTrajectory completed");
        cleanup();
        return 0;
    }

    if (unlikely(reached_EOF))
    {
        //should never reach this, just a failsafe
        printf("ERROR in update_setpoint_from_waypoint_NH: reached EOF");
        cleanup();
        return -1;
    }

    //update time since activation:
    waypoint_time_s = finddt_s(time_ns);

    // Parse waypoint flag
    switch ((int)waypoint.flag)
    {
    case TIME_TRANSITION_FLAG:
        // Check if there are additional waypoints and advnace control
        // to the next waytoint if it is time to do so.  If there are no additional waypoints,
        // keep controlling to the previous point
        if (waypoint_time_s >= waypoint.t)
        {

            // Set the desired x, y, and z if allowed
            if (state_machine.is_en())
            {
                cur_setpoint.XY.x.value.set(waypoints_init.x + waypoint.x);
                cur_setpoint.XY.y.value.set(waypoints_init.y + waypoint.y);
                cur_setpoint.Z.value.set(waypoints_init.z + waypoint.z);
                cur_setpoint.ATT.x.value.set(waypoints_init.roll + waypoint.roll);
                cur_setpoint.ATT.y.value.set(waypoints_init.pitch + waypoint.pitch);
                cur_setpoint.ATT.z.value.set(waypoints_init.yaw + waypoint.yaw);
            }
            // read new waypoint from file:
            read_waypoint_NH(fd_NH);
            ++cur_waypoint_num;
        }
        break;
    case POS_TRANSITION_FLAG:
        // TODO: determine position error and compare to convergence tolerance
        //       (? who sets/determines/stores convergence tolerance ?)
        //assert(0);
        break;
    default:
        fprintf(stderr, "ERROR: unrecognized waypoint flag\n");
        return -1;
    }
    return 0;
}


bool path_t::is_en(void)
{
    return en;
}

void path_t::stop(void)
{
    en = false;
    return;
}