/*
 * log_manager.cpp
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
 * Last Edit:  09/06/2022 (MM/DD/YYYY)
 *
 * Class to start, stop, and interact with the log manager thread.
 */
#include "log_manager.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <string.h>
 // to allow printf macros for multi-architecture portability
#define __STDC_FORMAT_MACROS 
#include <inttypes.h>

#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/encoder.h>

#include "rc_pilot_defs.hpp"
#include "thread_defs.hpp"
#include "settings.hpp"
#include "setpoint_manager.hpp"
#include "feedback.hpp"
#include "state_estimator.hpp"
#include "signal.h"
#include "tools.h"
#include "gps.hpp"
#include "benchmark.hpp"
#include "input_manager.hpp"
#include "KF.hpp"
#include "EKF.hpp"
#include "EKF2.hpp"
#include "voltage_sensor_gen.hpp"
#include "extra_sensors.hpp"
 
// preposessor macros
#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif // !unlikely

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif // !likely

 /*
 * Invoke defaut constructor for all the built in and exernal types in the class
 */
log_entry_t log_entry{};

static void* __log_manager_func(__attribute__((unused)) void* ptr)
{
    while (rc_get_state() != EXITING)
    {        
        if (log_entry.update() < 0)
        {
            printf("ERROR in __log_manager_func: failed to update log_mannager\n");
            return NULL;
        }
    }
    return NULL;
}

int log_entry_t::update(void)
{
    if (request_reset_fl)
    {
        if (reset() < 0)
        {
            printf("ERROR in update: failed to reset\n");
            return -1;
        }
        request_reset_fl = false;
    }
    if (new_data_available)
    {
        if (add_new() < 0)
        {
            printf("ERROR in update: failed to add new entry\n");
            return -1;
        }
        new_data_available = false;
    }
    else
    {
        rc_usleep(1000000 / LOG_MANAGER_HZ);
    }
    if (settings.log_benchmark) benchmark_timers.tLOG = rc_nanos_since_boot();
    return 0;
}


int log_entry_t::write_header(void)
{
    if (unlikely(!file_open))
    {
        printf("ERROR in write_header: file not opened\n");
        return -1;
    }

	// always print loop index
    fprintf(log_fd, "loop_index,last_step_ns,log_time_ns");
    
    /* state estimator */
    if (settings.log_state) state_estimator_entry.print_header(log_fd);
    
    if (settings.log_sensors)
    {
        battery_entry.print_header(log_fd, "batt_", settings.battery);
        bmp_entry.print_header(log_fd, "bmp_");
        IMU0_entry.print_header(log_fd, "IMU0_", settings.IMU0);
        IMU1_entry.print_header(log_fd, "IMU1_", settings.IMU1);
    }


    if (settings.mocap.enable) mocap_entry.print_header(log_fd, "mocap_",settings.mocap);

	if (settings.log_gps)
    {
        fprintf(log_fd,
            ",gps_lat,gps_lon,gps_gpsAlt,gps_ned_x,gps_ned_y,gps_ned_z,gps_spd,gps_fix,gps_sat,gps_"
            "headingNc,gps_cog,gps_gpsVsi,gps_hdop,gps_vdop,gps_year,gps_month,gps_day,gps_hour,"
            "gps_minute,gps_second,gps_time_received_ns");
    }

    KF_altitude_entry.print_header(log_fd, "KF_alt_");
    EKF1_entry.print_header(log_fd, "EKF1_");
    EKF2_entry.print_header(log_fd, "EKF2_");

    if (settings.log_setpoints)
    {
        if (settings.log_throttles)
        {
            fprintf(log_fd, ",X_thrt,Y_thrt,Z_thrt,roll_thrt,pitch_thrt,yaw_thrt");
        }

        if (settings.log_throttles_ff)
        {
            fprintf(log_fd, ",X_thrt_ff,Y_thrt_ff,Z_thrt_ff,roll_thrt_ff,pitch_thrt_ff,yaw_thrt_ff");
        }

        if (settings.log_attitude_rate_setpoint)
        {
            fprintf(log_fd, ",sp_roll_dot,sp_pitch_dot,sp_yaw_dot");
        }

        if (settings.log_attitude_rate_setpoint_ff)
        {
            fprintf(log_fd, ",sp_roll_dot_ff,sp_pitch_dot_ff,sp_yaw_dot_ff");
        }

        if (settings.log_attitude_setpoint)
        {
            fprintf(log_fd, ",sp_roll,sp_pitch,sp_yaw");
        }

        if (settings.log_attitude_setpoint_ff)
        {
            fprintf(log_fd, ",sp_roll_ff,sp_pitch_ff,sp_yaw_ff");
        }

        if (settings.log_acceleration_setpoint)
        {
            fprintf(log_fd, ",sp_Xddot,sp_Yddot,sp_Zddot");
        }

        if (settings.log_acceleration_setpoint_ff)
        {
            fprintf(log_fd, ",sp_Xddot,sp_Yddot,sp_Zddot");
        }

        if (settings.log_velocity_setpoint)
        {
            fprintf(log_fd, ",sp_Xdot,sp_Ydot,sp_Zdot");
        }

        if (settings.log_velocity_setpoint_ff)
        {
            fprintf(log_fd, ",sp_Xdot_ff,sp_Ydot_ff,sp_Zdot_ff");
        }

        if (settings.log_position_setpoint)
        {
            fprintf(log_fd, ",sp_X,sp_Y,sp_Z");
        }

        if (settings.log_position_setpoint_ff)
        {
            fprintf(log_fd, ",sp_X_ff,sp_Y_ff,sp_Z_ff");
        }
    }	
	

	if (settings.log_control_u)
    {
        fprintf(log_fd, ",u_roll,u_pitch,u_yaw,u_X,u_Y,u_Z");
    }

	if(settings.log_motor_signals && settings.num_rotors==8){
		fprintf(log_fd, ",mot_1,mot_2,mot_3,mot_4,mot_5,mot_6,mot_7,mot_8");
	}
	if(settings.log_motor_signals && settings.num_rotors==6){
		fprintf(log_fd, ",mot_1,mot_2,mot_3,mot_4,mot_5,mot_6");
	}
	if(settings.log_motor_signals && settings.num_rotors==4){
		fprintf(log_fd, ",mot_1,mot_2,mot_3,mot_4");
	}
	
	if (settings.log_dsm)
    {
        fprintf(log_fd, ",dsm_con");
    }
    if (settings.log_flight_mode)
    {
        fprintf(log_fd, ",flight_mode");
    }
	
	if(settings.log_benchmark){
        fprintf(log_fd, ",tIMU_END,tSM,tCOMMS,tMOCAP,tGPS,tPNI,tNAV,tGUI,tCTR,tLOG,tNTP");
    }

    if (settings.log_encoders) {
        fprintf(log_fd, ", rev1, rev2, rev3, rev4");
    }

	fprintf(log_fd, "\n");
	return 0;
}


int log_entry_t::write_log_entry(void)
{
    if (unlikely(!file_open))
    {
        printf("ERROR in write_log_entry: file not opened\n");
        return -1;
    }

	// always print loop index
    fprintf(log_fd, "%" PRIu64 ",%" PRIu64 ",%" PRIu64, 
            loop_index, last_step_ns, log_time_ns);
	
    if (settings.log_state) state_estimator_entry.print_entry(log_fd);
    
    if (settings.log_sensors)
    {
        battery_entry.print_entry(log_fd, settings.battery);
        bmp_entry.print_entry(log_fd);
        IMU0_entry.print_entry(log_fd, settings.IMU0);
        IMU1_entry.print_entry(log_fd, settings.IMU1);
    }
    
    if (settings.mocap.enable) mocap_entry.print_entry(log_fd, settings.mocap);

    if (settings.log_gps)
    {
        fprintf(log_fd,
            ",%.8f,%.8f,%.3f,%.3f,%.3f,%.3f,%.3f,%i,%i,%.3f,%.3f,%.3f,%.3f,%.3f,%i,%i,%i,%i,%i,%i",
            gps_lat, gps_lon, gps_gpsAlt, gps_ned_x, gps_ned_y, gps_ned_z, gps_spd,
            gps_fix, gps_sat, gps_headingNc, gps_cog, gps_gpsVsi, gps_hdop, gps_vdop,
            gps_year, gps_month, gps_day, gps_hour, gps_minute, gps_second);
        fprintf(log_fd, ",%" PRIu64, gps_time_received_ns);
    }

    KF_altitude_entry.print_entry(log_fd);
    EKF1_entry.print_entry(log_fd);
    EKF2_entry.print_entry(log_fd);

    if (settings.log_setpoints)
    {
        if (settings.log_throttles)
        {
            fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", X_throttle, Y_throttle, Z_throttle,
                roll_throttle, pitch_throttle, yaw_throttle);
        }

        if (settings.log_throttles_ff)
        {
            fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", X_throttle_ff, Y_throttle_ff, Z_throttle_ff,
                roll_throttle_ff, pitch_throttle_ff, yaw_throttle_ff);
        }

        if (settings.log_attitude_rate_setpoint)
        {
            fprintf(log_fd, ",%.4F,%.4F,%.4F", sp_roll_dot,
                sp_pitch_dot, sp_yaw_dot);
        }

        if (settings.log_attitude_rate_setpoint_ff)
        {
            fprintf(log_fd, ",%.4F,%.4F,%.4F", sp_roll_dot_ff,
                sp_pitch_dot_ff, sp_yaw_dot_ff);
        }

        if (settings.log_attitude_setpoint)
        {
            fprintf(log_fd, ",%.4F,%.4F,%.4F", sp_roll, sp_pitch, sp_yaw);
        }

        if (settings.log_attitude_setpoint_ff)
        {
            fprintf(log_fd, ",%.4F,%.4F,%.4F", sp_roll_ff, sp_pitch_ff, sp_yaw_ff);
        }

        if (settings.log_acceleration_setpoint)
        {
            fprintf(log_fd, ",%.4F,%.4F,%.4F", sp_Xddot,
                sp_Yddot, sp_Zddot);
        }

        if (settings.log_acceleration_setpoint_ff)
        {
            fprintf(log_fd, ",%.4F,%.4F,%.4F", sp_Xddot_ff,
                sp_Yddot_ff, sp_Zddot_ff);
        }

        if (settings.log_velocity_setpoint)
        {
            fprintf(log_fd, ",%.4F,%.4F,%.4F", sp_Xdot,
                sp_Ydot, sp_Zdot);
        }

        if (settings.log_velocity_setpoint_ff)
        {
            fprintf(log_fd, ",%.4F,%.4F,%.4F", sp_Xdot_ff,
                sp_Ydot_ff, sp_Zdot_ff);
        }

        if (settings.log_position_setpoint)
        {
            fprintf(log_fd, ",%.4F,%.4F,%.4F", sp_X,
                sp_Y, sp_Z);
        }

        if (settings.log_position_setpoint_ff)
        {
            fprintf(log_fd, ",%.4F,%.4F,%.4F", sp_X_ff,
                sp_Y_ff, sp_Z_ff);
        }
    }
    

    if (settings.log_control_u)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", u_roll, u_pitch, u_yaw, u_X,
            u_Y, u_Z);
    }

	if(settings.log_motor_signals && settings.num_rotors==8){
		fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F",	mot_1,\
							mot_2,\
							mot_3,\
							mot_4,\
							mot_5,\
							mot_6,\
							mot_7,\
							mot_8);
	}
	if(settings.log_motor_signals && settings.num_rotors==6){
		fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F",	mot_1,\
							mot_2,\
							mot_3,\
							mot_4,\
							mot_5,\
							mot_6);
	}
	if(settings.log_motor_signals && settings.num_rotors==4){
		fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F",		mot_1,\
							mot_2,\
							mot_3,\
							mot_4);
	}
	
	if (settings.log_dsm)
    {
        fprintf(log_fd, ",%i", dsm_con);
    }
    if (settings.log_flight_mode)
    {
        fprintf(log_fd, ",%i", flight_mode);
    }
	
	if(settings.log_benchmark){
        fprintf(log_fd, ",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64",%" PRIu64,
            tIMU_END, tSM, tCOMMS, tMOCAP, tGPS, tPNI, tNAV, tGUI, tCTR, tLOG, tNTP);
    }
    /*
    if (settings.log_encoders) {
        fprintf(log_fd, ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64, \
            rev1, \
            rev2, \
            rev3, \
            rev4);
    }
	*/

	fprintf(log_fd, "\n");
	return 0;
}

int log_entry_t::init(void)
{
    if (unlikely(initialized))
    {
        printf("ERROR in init: already initialized\n");
        return -1;
    }
    if (unlikely(thread.init(LOG_MANAGER_PRI, FIFO) < 0))
    {
        printf("ERROR in init: failed to initialized the thread.\n");
        logging_enabled = false;
        initialized = false;
        return -1;
    }
    request_reset_fl = false;
    new_data_available = false;
    file_open = false;



    if (unlikely((request_reset() < 0)))
    {
        printf("ERROR in init: failed to reset\n");
        initialized = false;
        logging_enabled = false;
        return -1;
    }

    if (unlikely(thread.start(__log_manager_func) < 0))
    {
        printf("ERROR in init: failed to start the thread.\n");
        logging_enabled = false;
        initialized = false;
        return -1;
    }

    initialized = true;
	return 0;
}

int log_entry_t::request_reset(void)
{
    if (file_open && num_entries < 1) return 0;
    request_reset_fl = true;
    return 0;
}


int log_entry_t::reset(void)
{
    int i;
    char path[100];
    struct stat st = { 0 };

    // if the logging is running, stop before starting a new log file
    if (logging_enabled) 
    {
        logging_enabled = false;
        if (file_open)
        {
            fclose(log_fd);
            file_open = false;
        }        
    }

    // first make sure the directory exists, make it if not
    if (stat(LOG_DIR, &st) == -1) {
        mkdir(LOG_DIR, 0755);
    }

    // search for existing log files to determine the next number in the series
    for (i = 1; i <= MAX_LOG_FILES + 1; i++) {
        memset(&path, 0, sizeof(path));
        sprintf(path, LOG_DIR "%d.csv", i);
        // if file exists, move onto the next index
        if (stat(path, &st) == 0) continue;
        else break;
    }
    // limit number of log files
    if (i == MAX_LOG_FILES + 1) {
        fprintf(stderr, "ERROR: log file limit exceeded\n");
        fprintf(stderr, "delete old log files before continuing\n");
        return -1;
    }
    // create and open new file for writing
    log_fd = fopen(path, "w+");
    if (log_fd == 0) {
        printf("ERROR: can't open log file for writing\n");
        return -1;
    }
    file_open = true;

    // write header
    if (unlikely(write_header()) == -1)
    {
        printf("ERROR in reset, failed to write header\n");
        return -1;
    }


    // start reset counters        
    num_entries = 0;
    num_entries_skipped = 0;

    logging_enabled = true;
    return 0;
}

void log_entry_t::construct_new_entry(void)
{
	loop_index 	    = fstate.get_loop_index();
    last_step_ns 	= fstate.get_last_step_ns();
    log_time_ns     = rc_nanos_since_boot();

    if (settings.log_sensors)
    {
        battery_entry.update(batt, settings.battery);
        bmp_entry.update(bmp);
        IMU0_entry.update(IMU0, settings.IMU0);
        IMU1_entry.update(IMU1, settings.IMU1);
    }

    if (settings.log_state) state_estimator_entry.update(&state_estimate);
    
    if (settings.mocap.enable) mocap_entry.update(mocap, settings.mocap);

    /*
    rev1 = state_estimate.rev[0];
    rev2 = state_estimate.rev[1];
    rev3 = state_estimate.rev[2];
    rev4 = state_estimate.rev[3];
    */

    gps_lon 		= gps_data.lla.lon;
    gps_lat 		= gps_data.lla.lat;
    gps_gpsAlt  	= gps_data.lla.alt;
    gps_ned_x 	    = gps_data.ned.x;
    gps_ned_y 	    = gps_data.ned.y;
    gps_ned_z 	    = gps_data.ned.z;
    gps_spd 		= gps_data.spd;
    gps_fix 		= (int)gps_data.fix;
    gps_sat 		= gps_data.sat;
    gps_headingNc   = gps_data.headingNc;
    gps_cog 		= gps_data.cog;
    gps_gpsVsi 	    = gps_data.gpsVsi;
    gps_hdop 		= gps_data.hdop;
    gps_vdop 		= gps_data.vdop;
    gps_year 		= gps_data.year;
    gps_month 	    = gps_data.month;
    gps_day 		= gps_data.day;
    gps_hour 		= gps_data.hour;
    gps_minute 	    = gps_data.minute;
    gps_second 	    = gps_data.second;
    gps_time_received_ns = gps_data.gps_data_received_ns;

    /* Filters */
    KF_altitude_entry.update(&KF_altitude);
    EKF1_entry.update(EKF1);
    EKF2_entry.update(EKF2);


    X_throttle 	    = setpoint.POS_throttle.x.value.get();//setpoint.X_throttle;
    Y_throttle 	    = setpoint.POS_throttle.y.value.get();//setpoint.Y_throttle;
    Z_throttle      = setpoint.POS_throttle.z.value.get();//setpoint.Z_throttle;
    X_throttle_ff   = setpoint.POS_throttle.x.FF.get();//setpoint.X_throttle FF;
    Y_throttle_ff   = setpoint.POS_throttle.y.FF.get();//setpoint.Y_throttle FF;
    Z_throttle_ff   = setpoint.POS_throttle.z.FF.get();//setpoint.Z_throttle FF;
    roll_throttle   = setpoint.ATT_throttle.x.value.get();//setpoint.roll_throttle;
    pitch_throttle  = setpoint.ATT_throttle.y.value.get();//setpoint.pitch_throttle;
    yaw_throttle 	= setpoint.ATT_throttle.z.value.get();//setpoint.yaw_throttle;
    roll_throttle_ff = setpoint.ATT_throttle.x.FF.get();//setpoint.roll_throttle;
    pitch_throttle_ff = setpoint.ATT_throttle.y.FF.get();//setpoint.pitch_throttle;
    yaw_throttle_ff = setpoint.ATT_throttle.z.FF.get();//setpoint.yaw_throttle;

    sp_roll_dot 	= setpoint.ATT_dot.x.value.get();
    sp_pitch_dot 	= setpoint.ATT_dot.y.value.get();
    sp_yaw_dot 	    = setpoint.ATT_dot.z.value.get();
    sp_roll_dot_ff  = setpoint.ATT_dot.x.FF.get();
    sp_pitch_dot_ff = setpoint.ATT_dot.y.FF.get();
    sp_yaw_dot_ff   = setpoint.ATT_dot.z.FF.get();
    sp_roll 		= setpoint.ATT.x.value.get();
    sp_pitch 		= setpoint.ATT.y.value.get();
    sp_yaw 		    = setpoint.ATT.z.value.get();
    sp_roll_ff 	    = setpoint.ATT.x.FF.get();
    sp_pitch_ff 	= setpoint.ATT.y.FF.get();
    sp_yaw_ff       = setpoint.ATT.z.FF.get();

    sp_X 			= setpoint.XY.x.value.get();
    sp_Y 			= setpoint.XY.y.value.get();
    sp_Z 			= setpoint.Z.value.get();
    sp_X_ff         = setpoint.XY.x.FF.get();
    sp_Y_ff         = setpoint.XY.y.FF.get();
    sp_Z_ff         = setpoint.Z.FF.get();
    sp_Xdot 		= setpoint.XY_dot.x.value.get();
    sp_Ydot 		= setpoint.XY_dot.y.value.get();
    sp_Zdot 		= setpoint.Z_dot.value.get();
    sp_Xdot_ff 	    = setpoint.XY_dot.x.FF.get();
    sp_Ydot_ff  	= setpoint.XY_dot.y.FF.get();
    sp_Zdot_ff 	    = setpoint.Z.value.get();
    sp_Xddot        = setpoint.XYZ_ddot.x.value.get();
    sp_Yddot 		= setpoint.XYZ_ddot.y.value.get();
    sp_Zddot 		= setpoint.XYZ_ddot.z.value.get();
    sp_Xddot_ff     = setpoint.XYZ_ddot.x.FF.get();
    sp_Yddot_ff     = setpoint.XYZ_ddot.y.FF.get();
    sp_Zddot_ff     = setpoint.XYZ_ddot.z.FF.get();

    u_roll 		    = fstate.get_u(VEC_ROLL);
    u_pitch 		= fstate.get_u(VEC_PITCH);
    u_yaw 		    = fstate.get_u(VEC_YAW);
    u_X 			= fstate.get_u(VEC_Y);
    u_Y 			= fstate.get_u(VEC_X);
    u_Z 			= fstate.get_u(VEC_Z);

    mot_1 		= fstate.get_m(0);
    mot_2 		= fstate.get_m(1);
    mot_3 		= fstate.get_m(2);
    mot_4 		= fstate.get_m(3);
    mot_5 		= fstate.get_m(4);
    mot_6 		= fstate.get_m(5);
    mot_7 		= fstate.get_m(6);
    mot_8 		= fstate.get_m(7);

    dsm_con 		= user_input.input_active;

    flight_mode 	= user_input.get_flight_mode();

    tIMU_END 		= benchmark_timers.tIMU_END;
    tSM 			= benchmark_timers.tSM;
    tMOCAP 		    = benchmark_timers.tMOCAP;
    tCOMMS          = benchmark_timers.tCOMMS;
    tGPS 			= benchmark_timers.tGPS;
    tPNI 			= benchmark_timers.tPNI;
    tNAV 			= benchmark_timers.tNAV;
    tGUI 			= benchmark_timers.tGUI;
    tCTR 			= benchmark_timers.tCTR;
    tLOG 			= benchmark_timers.tLOG;
    tNTP 			= benchmark_timers.tNTP;
	return;
}

int log_entry_t::add_new()
{
    if (unlikely(!initialized))
    {
        printf("\nERROR in add_new: not initialized");
        return -1;
    }

	if (!logging_enabled)
    {
        if (settings.log_only_while_armed)
        {
            return 0;
        }   
        else 
        {
            fprintf(stderr, "ERROR: trying to log entry while logger isn't running\n");
            return -1;
        }
    }
	
    if (num_entries_skipped > settings.log_every_n_entry)
    {
        construct_new_entry();
        write_log_entry();
        fflush(log_fd);
        num_entries++;
    }
    num_entries_skipped++;

    return 0;
}

void log_entry_t::data_available(void)
{
    new_data_available = true;
}


int log_entry_t::cleanup(void)
{
	// just return if not logging
    if (!logging_enabled && !file_open) return 0;

    if (unlikely(!initialized))
    {
        printf("WARNING: trying to cleanup log manager when not initialized\n");
        return -1;
    }
    if (thread.is_started() && thread.stop(LOG_MANAGER_TOUT) < 0)
    {
        printf("ERROR: failed to terminate thread\n");
        return -1;
    }

    logging_enabled = false;
    initialized = false;
    fclose(log_fd);

    return 0;
}
