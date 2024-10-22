/*
 * mocap_gen.hpp
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
 * Last Edit:  09/19/2022 (MM/DD/YYYY)
 *
 * Summary :
 * Here is defined general class for operating motion capture system 
 */

#ifndef MOCAP_GEN_HPP
#define MOCAP_GEN_HPP
#include <rc/time.h>
#include <stdint.h> // for uint64_t
#include <json.h>

#include "coordinate_frames_gen.hpp"
#include "signal_filter_gen.hpp"

typedef struct mocap_settings_t
{
	bool enable;
	bool use_roll_pitch_rate;
	bool use_yaw_rate;
	bool use_roll_pitch;
	bool use_heading;
	bool enable_pos_filter;
	coordinate_frames_gen_t frame_type;// = NWU;
	signal_filter_gen_settings_t att_filter[3]; //filter settings for attitude
	signal_filter_gen_settings_t vel_filter[3]; //filter settings for velocity
	signal_filter_gen_settings_t pos_filter[3]; //filter settings for position
	bool enable_warnings;
	bool enable_logging;
	bool log_att;
	bool log_vel;
	bool log_raw;
}mocap_settings_t;


 /* General class for all mocap instances */
class mocap_gen_t
{
private:
	uint64_t time = 0;

	bool initialized = false;
	bool valid = false;
	int no_new_updates = 0; //counts the number of updates

	uint64_t time_att = rc_nanos_since_boot();

	bool att_updated = false; //has to be true before marching
	bool pos_updated = false; //has to be true before marching

	double att_quat_raw[4] = { 1.0 , 0.0, 0.0, 0.0 };
	double att_quat_NED[4] = { 1.0 , 0.0, 0.0, 0.0 };

	double att_tb_raw[3] = { 0.0 , 0.0, 0.0 };
	double att_tb_NED[3] = { 0.0 , 0.0, 0.0 };

	double att_rate_tb_raw_NED[3] = { 0.0 , 0.0, 0.0 };
	double att_rate_tb_filtered_NED[3] = { 0.0 , 0.0, 0.0 };

	int num_heading_spins = 0;
	double continuous_heading_NED = 0.0; ///<  keeps increasing/decreasing above +-2pi


	uint64_t time_pos = rc_nanos_since_boot();

	double pos_raw[3] = { 0.0 , 0.0, 0.0 };
	double pos_raw_NED[3] = { 0.0 , 0.0, 0.0 };
	double pos_filtered_NED[3] = { 0.0 , 0.0, 0.0 };

	double vel_raw_NED[3] = { 0.0 , 0.0, 0.0 };
	double vel_filtered_NED[3] = { 0.0 , 0.0, 0.0 };		

	mocap_settings_t settings; //mocap settings

	// mocap filters
	signal_filter3D_gen_t att_lp{};
	signal_filter3D_gen_t vel_lp{};
	signal_filter3D_gen_t pos_lp{};

public:
	

	/* Initialization */
	char init(void);
	char init(mocap_settings_t& new_mocap_settings);
	bool is_initialized(void);
	bool is_valid(void);

	/* Updating */
	char update_time(uint64_t new_time, uint8_t new_valid_fl);
	char update_att_from_quat(double new_mocap_quat_raw[4]);
	char update_pos_vel_from_pos(double new_mocap_pos_raw[3]);
	char march(void);

	/* Data retrieval */
	uint64_t get_time(void);
	uint64_t get_time_att(void);
	uint64_t get_time_pos(void);
	void get_quat_raw(double* buff);
	void get_quat(double* buff);
	void get_tb_raw(double* buff);
	void get_tb(double* buff);
	void get_tb_rate_raw(double* buff);
	void get_tb_rate(double* buff);
	double get_continuous_heading(void);

	void get_pos_raw(double* buff);
	void get_pos_raw_NED(double* buff);
	void get_pos(double* buff);
	void get_vel_raw(double* buff);
	void get_vel(double* buff);

	/* Reset and cleanup */
	char reset(void);
	void cleanup(void);
};

extern mocap_gen_t mocap;


/** @name Logging class for mocap
* Defines how logging should be done for this class
*/
class mocap_log_entry_t
{
private:
	uint64_t time;
	bool valid;

	uint64_t time_att;

	double att_quat_raw[4];
	double att_quat_NED[4];

	double att_tb_raw[3];
	double att_tb_NED[3];

	double att_rate_tb_raw_NED[3];
	double att_rate_tb_filtered_NED[3];

	double continuous_heading_NED; ///<  keeps increasing/decreasing above +-2pi


	uint64_t time_pos;

	double pos_raw[3];
	double pos_raw_NED[3];
	double pos_filtered_NED[3];

	double vel_raw_NED[3];
	double vel_filtered_NED[3];
	

	char print_vec(FILE* file, double* vec_in, int size);
	char print_header_vec(FILE* file, const char* prefix, const char* var_name, int size);
public:
	char update(mocap_gen_t& new_state, mocap_settings_t& log_settings);
	char print_header(FILE* file, const char* prefix, mocap_settings_t& log_settings);
	char print_entry(FILE* file, mocap_settings_t& log_settings);
};


int parse_mocap_gen_settings(json_object* in_json, const char* name, mocap_settings_t& sensor);

#endif // !MOCAP_GEN_HPP