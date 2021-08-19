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
 * Last Edit:  08/19/2020 (MM/DD/YYYY)
 *
 * Class to start, stop, and interact with the log manager.
 * 
 * NOTE:
 * fflush only writes data to buffer, but not disk. we need to force pflush deamon to write 
 * flush buffer and write it to disk more frequntly to avoid runnning out of memory and delays
 * caused by waiting for the large chunk of data to be written to disk. 
 * To do this, modify /etc/sysctl.confby adding the following two lines at the end of the file:
 * vm.dirty_expire_centisecs = 100
 * vm.dirty_writeback_centisecs = 100
 * 
 * this will force pdflush deamon thread to write data to disk every 100 centiseconds. 
 * To apply the changes imediately:
 * sudo sysctl -p
 * and to check current settings, type in the console:
 * sysctl vm.dirty_expire_centisecs
 * sysctl vm.dirty_writeback_centisecs
 */

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
#include <rc/pthread.h>
#include <rc/encoder.h>

#include "rc_pilot_defs.h"
#include "thread_defs.h"
#include "settings.h"
#include "setpoint_manager.hpp"
#include "feedback.hpp"
#include "state_estimator.h"
#include "signal.h"
#include "xbee_receive.h"
#include "tools.h"
#include "gps.h"
#include "benchmark.h"
#include "input_manager.hpp"

#include "log_manager.hpp"
// preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

 /*
 * Invoke defaut constructor for all the built in and exernal types in the class
 */
log_entry_t log_entry{};

int log_entry_t::write_header(void)
{
    if (unlikely(!initialized))
    {
        printf("\nERROR in write_header: not initialized");
        return -1;
    }

	// always print loop index
    fprintf(log_fd, "loop_index,last_step_ns,imu_time_ns,bmp_time_ns,log_time_ns");
	

	if (settings.log_sensors)
    {
        fprintf(log_fd,
            ",v_batt,alt_bmp_raw,bmp_temp,gyro_roll,gyro_pitch,gyro_yaw,accel_X,accel_Y,accel_Z,mag_X, "
            "mag_Y, mag_Z");
    }

	if (settings.log_state)
    {
        fprintf(log_fd, ",roll,pitch,yaw,cont_yaw,rollDot,pitchDot,yawDot,X,Y,Z,Xdot,Ydot,Zdot,Xdot_raw,Ydot_raw,Zdot_raw,Zddot");
    }
	
	if (settings.log_mocap)
    {
        fprintf(log_fd,
            ",mocap_time,mocap_timestamp_ns,mocap_x,mocap_y,mocap_z,mocap_qw,mocap_qx,mocap_qy,mocap_qz,"
            "mocap_roll,mocap_pitch,"
            "mocap_yaw");
    }
	
	if (settings.log_gps)
    {
        fprintf(log_fd,
            ",gps_lat,gps_lon,gps_gpsAlt,gps_ned_x,gps_ned_y,gps_ned_z,gps_spd,gps_fix,gps_sat,gps_"
            "headingNc,gps_cog,gps_gpsVsi,gps_hdop,gps_vdop,gps_year,gps_month,gps_day,gps_hour,"
            "gps_minute,gps_second,gps_time_received_ns");
    }
	
	if (settings.log_throttles)
    {
        fprintf(log_fd, ",X_thrt,Y_thrt,Z_thrt,roll_thrt,pitch_thrt,yaw_thrt");
    }
	
	if (settings.log_attitude_setpoint)
    {
        fprintf(log_fd, ",sp_roll_dot,sp_pitch_dot,sp_yaw_dot,sp_roll_dot_ff,sp_pitch_dot_ff,sp_yaw_dot_ff");
        fprintf(log_fd, ",sp_roll,sp_pitch,sp_yaw,sp_roll_ff,sp_pitch_ff");
    }
	
	if (settings.log_position_setpoint)
    {
        fprintf(log_fd, ",sp_X,sp_Y,sp_Z,sp_Xdot,sp_Ydot,sp_Zdot");
        fprintf(log_fd, ",sp_Xdot_ff,sp_Ydot_ff,sp_Zdot_ff,sp_Xddot,sp_Yddot,sp_Zddot");
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
        fprintf(log_fd, ",tIMU,tIMU_END,tSM,tXBEE,tGPS,tPNI,tNAV,tGUI,tCTR,tLOG,tNTP");
    }

    if (settings.log_encoders) {
        fprintf(log_fd, ", rev1, rev2, rev3, rev4");
    }

	fprintf(log_fd, "\n");
	return 0;
}


int log_entry_t::write_log_entry(void)
{
    if (unlikely(!initialized))
    {
        printf("\nERROR in write_log_entry: not initialized");
        return -1;
    }

	// always print loop index
    fprintf(log_fd, "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64, 
            loop_index, last_step_ns, imu_time_ns, imu_time_ns, log_time_ns);
							
	if (settings.log_sensors)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", v_batt,
            alt_bmp_raw, bmp_temp, gyro_roll, gyro_pitch, gyro_yaw, accel_X, accel_Y, accel_Z,
            mag_X, mag_Y, mag_Z);
    }

	if (settings.log_state)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", roll, pitch, yaw,
            yaw_cont, rollDot, pitchDot, yawDot, X, Y, Z, Xdot, Ydot, Zdot, Xdot_raw, Ydot_raw, Zdot_raw, Zddot);
    }

    if (settings.log_mocap)
    {
        fprintf(log_fd, ",%" PRIu32 ",%" PRIu64, mocap_time, mocap_timestamp_ns);
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", mocap_x, mocap_y,
            mocap_z, mocap_qw, mocap_qx, mocap_qy, mocap_qz, mocap_roll, mocap_pitch,
            mocap_yaw);
    }

    if (settings.log_gps)
    {
        fprintf(log_fd,
            ",%.8f,%.8f,%.3f,%.3f,%.3f,%.3f,%.3f,%i,%i,%.3f,%.3f,%.3f,%.3f,%.3f,%i,%i,%i,%i,%i,%i",
            gps_lat, gps_lon, gps_gpsAlt, gps_ned_x, gps_ned_y, gps_ned_z, gps_spd,
            gps_fix, gps_sat, gps_headingNc, gps_cog, gps_gpsVsi, gps_hdop, gps_vdop,
            gps_year, gps_month, gps_day, gps_hour, gps_minute, gps_second);
        fprintf(log_fd, ",%" PRIu64, gps_time_received_ns);
    }

    if (settings.log_throttles)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", X_throttle, Y_throttle, Z_throttle,
            roll_throttle, pitch_throttle, yaw_throttle);
    }

    if (settings.log_attitude_setpoint)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", sp_roll_dot,
            sp_pitch_dot, sp_yaw_dot, sp_roll_dot_ff, sp_pitch_dot_ff, sp_yaw_dot_ff, sp_roll, sp_pitch, sp_yaw,
            sp_roll_ff, sp_pitch_ff);
    }

    if (settings.log_position_setpoint)
    {
        fprintf(log_fd, ",%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F,%.4F", sp_X,
            sp_Y, sp_Z, sp_Xdot, sp_Ydot, sp_Zdot, sp_Xdot_ff, sp_Ydot_ff, sp_Zdot_ff,
            sp_Xddot, sp_Yddot, sp_Zddot);
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
            tIMU, tIMU_END, tSM, tXBEE, tGPS, tPNI, tNAV, tGUI, tCTR, tLOG, tNTP);
    }
    if (settings.log_encoders) {
        fprintf(log_fd, ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%" PRId64, \
            rev1, \
            rev2, \
            rev3, \
            rev4);
    }
	
	fprintf(log_fd, "\n");
	return 0;
}

int log_entry_t::init(void)
{
    if (unlikely(initialized))
    {
        printf("\nERROR in log_manager_init: already initialized");
        return -1;
    }
    if (unlikely((reset() == -1)))
    {
        printf("\nERROR in log_manager_init: failed to reset");
        initialized = false;
        logging_enabled = false;
        return -1;
    }


    // start thread
    logging_enabled = true;
    initialized = true;
    num_entries = 0;
    num_entries_skipped = 0;
	return 0;
}


int log_entry_t::reset(void)
{
    int i;
    char path[100];
    struct stat st = { 0 };

    // if the thread is running, stop before starting a new log file
    if (logging_enabled) {
        cleanup();
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

    initialized = true;
    // write header
    if (unlikely(write_header()) == -1)
    {
        printf("\nERROR in init, failed to write header");
        initialized = false;
        return -1;
    }


    // start thread
    logging_enabled = true;
    initialized = true;
    num_entries = 0;
    num_entries_skipped = 0;
    return 0;
}

void log_entry_t::construct_new_entry(void)
{
	loop_index 	    = fstate.get_loop_index();
    last_step_ns 	= fstate.get_last_step_ns();
    imu_time_ns 	= state_estimate.imu_time_ns;
    bmp_time_ns 	= state_estimate.bmp_time_ns;
    log_time_ns     = rc_nanos_since_boot();

    v_batt 		    = state_estimate.v_batt_lp;
    alt_bmp_raw 	= state_estimate.alt_bmp_raw;
    bmp_temp 		= state_estimate.bmp_temp;
    gyro_roll 	    = state_estimate.gyro[0];
    gyro_pitch 	    = state_estimate.gyro[1];
    gyro_yaw 		= state_estimate.gyro[2];
    accel_X 		= state_estimate.accel[0];
    accel_Y 		= state_estimate.accel[1];
    accel_Z 		= state_estimate.accel[2];
    mag_X 		    = state_estimate.mag[0];
    mag_Y 		    = state_estimate.mag[1];
    mag_Z 		    = state_estimate.mag[2];

    roll 			= state_estimate.roll;
    pitch 		    = state_estimate.pitch;
    yaw 			= state_estimate.yaw;
	yaw_cont 		= state_estimate.continuous_yaw;
    rollDot 		= state_estimate.roll_dot;
    pitchDot 		= state_estimate.pitch_dot;
    yawDot 		    = state_estimate.yaw_dot;

    X 			    = state_estimate.X;
    Y 			    = state_estimate.Y;
    Z 			    = state_estimate.Z;
    Xdot 			= state_estimate.X_dot;
    Ydot 			= state_estimate.Y_dot;
    Zdot 			= state_estimate.Z_dot;
    Xdot_raw        = state_estimate.X_dot_raw;
    Ydot_raw        = state_estimate.Y_dot_raw;
    Zdot_raw        = state_estimate.Z_dot_raw;
    Zddot 		    = state_estimate.Z_ddot;

    mocap_time 	    = state_estimate.mocap_time;
    mocap_timestamp_ns = state_estimate.mocap_timestamp_ns;
    mocap_x 		= state_estimate.pos_mocap[0];
    mocap_y 		= state_estimate.pos_mocap[1];
    mocap_z 		= state_estimate.pos_mocap[2];
    mocap_qw 		= state_estimate.quat_mocap[0];
    mocap_qx 		= state_estimate.quat_mocap[1];
    mocap_qy 		= state_estimate.quat_mocap[2];
    mocap_qz 		= state_estimate.quat_mocap[3];
    mocap_roll 	    = state_estimate.tb_mocap[0];
    mocap_pitch 	= state_estimate.tb_mocap[0];
    mocap_yaw 	    = state_estimate.tb_mocap[0];

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

    X_throttle 	    = setpoint.X_throttle;
    Y_throttle 	    = setpoint.Y_throttle;
    Z_throttle 	    = setpoint.Z_throttle;
    roll_throttle   = setpoint.roll_throttle;
    pitch_throttle  = setpoint.pitch_throttle;
    yaw_throttle 	= setpoint.yaw_throttle;

    sp_roll_dot 	= setpoint.roll_dot;
    sp_pitch_dot 	= setpoint.pitch_dot;
    sp_yaw_dot 	    = setpoint.yaw_dot;
    sp_roll_dot_ff  = setpoint.roll_dot_ff;
    sp_pitch_dot_ff = setpoint.pitch_dot_ff;
    sp_yaw_dot_ff   = setpoint.yaw_dot_ff;
    sp_roll 		= setpoint.roll;
    sp_pitch 		= setpoint.pitch;
    sp_yaw 		    = setpoint.yaw;
    sp_roll_ff 	    = setpoint.roll_ff;
    sp_pitch_ff 	= setpoint.pitch_ff;

    sp_X 			= setpoint.X;
    sp_Y 			= setpoint.Y;
    sp_Z 			= setpoint.Z;
    sp_Xdot 		= setpoint.X_dot;
    sp_Ydot 		= setpoint.Y_dot;
    sp_Zdot 		= setpoint.Z_dot;
    sp_Xdot_ff 	    = setpoint.X_dot_ff;
    sp_Ydot_ff  	= setpoint.Y_dot_ff;
    sp_Zdot_ff 	    = setpoint.Z_dot_ff;
    sp_Xddot 		= setpoint.X_ddot;
    sp_Yddot 		= setpoint.Y_ddot;
    sp_Zddot 		= setpoint.Z_ddot;

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

    flight_mode 	= user_input.flight_mode;

    tIMU 			= benchmark_timers.tIMU;
    tIMU_END 		= benchmark_timers.tIMU_END;
    tSM 			= benchmark_timers.tSM;
    tXBEE 		    = benchmark_timers.tXBEE;
    tGPS 			= benchmark_timers.tGPS;
    tPNI 			= benchmark_timers.tPNI;
    tNAV 			= benchmark_timers.tNAV;
    tGUI 			= benchmark_timers.tGUI;
    tCTR 			= benchmark_timers.tCTR;
    tLOG 			= benchmark_timers.tLOG;
    tNTP 			= benchmark_timers.tNTP;

    rev1            = state_estimate.rev[0];
    rev2            = state_estimate.rev[1];
    rev3            = state_estimate.rev[2];
    rev4            = state_estimate.rev[3];

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
        //try writing to disk:
        //syncfs(fileno(log_fd)); //don't do this, introduces significant delay
        num_entries++;
    }
    num_entries_skipped++;

    return 0;
}

int log_entry_t::cleanup(void)
{
	// just return if not logging
    if (!logging_enabled) return 0;

    if (unlikely(!initialized))
    {
        printf("\nWARNING: trying to cleanup when not initialized");
        return 0;
    }


    logging_enabled = false;
    initialized = false;
    fclose(log_fd);

    return 0;
}
