/*
 * mocap_gen.cpp
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
 * Last Edit:  09/03/2022 (MM/DD/YYYY)
 *
 * Summary :
 * Here is defined general class for operating motion capture system
 */

#include "mocap_gen.hpp"
#include "tools.h"
#include "coordinate_frames_gen.hpp"
#include "settings_gen.hpp"

#ifndef GET_VARIABLE_NAME
#define GET_VARIABLE_NAME(Variable) (#Variable)
#endif // !GET_VARIABLE_NAME

mocap_gen_t mocap{};

 /* General class for all MOCAP instances */
char mocap_gen_t::init(mocap_settings_t& new_mocap_settings)
{
	if (unlikely(initialized))
	{
		fprintf(stderr, "ERROR in init: mocap already initialized.\n");
		return -1;
	}

	char tmp = att_lp.set(new_mocap_settings.att_filter);
	if (unlikely(vel_lp.set(new_mocap_settings.vel_filter) < 0 || tmp < 0))
	{
		fprintf(stderr, "ERROR in init: failed to initialize mocap filters\n");
		return -1;
	}

	settings = new_mocap_settings;
	initialized = true;
	valid = false;
	att_updated = false;
	pos_updated = false;
	return 0;
}
char mocap_gen_t::init(void)
{
	/* assume some default values */
	settings.frame_type = NWU;
	for (int i = 0; i < 3; i++) {
		settings.att_filter[i].dt = DT;
		settings.att_filter[i].type = Lowpass;
		settings.att_filter[i].enable_saturation = false;

		settings.vel_filter[i].dt = DT;
		settings.vel_filter[i].type = Lowpass;
		settings.vel_filter[i].enable_saturation = false;
	}
	settings.att_filter[0].tc = 6.0 * DT;
	settings.att_filter[1].tc = 6.0 * DT;
	settings.att_filter[2].tc = 10.0 * DT;

	settings.vel_filter[0].tc = 20.0 * DT;
	settings.vel_filter[1].tc = 20.0 * DT;
	settings.vel_filter[2].tc = 4.0 * DT;
	settings.enable_warnings = true;

	return init(settings);


}

bool mocap_gen_t::is_initialized(void)
{
	return initialized;
}
bool mocap_gen_t::is_valid(void)
{
	return valid;
}

char mocap_gen_t::update_time(uint64_t new_time, uint8_t new_valid_fl)
{
	if (new_valid_fl > 0 && new_time > time)
	{
		time = new_time;
		no_new_updates = 0;
		valid = true;
	}
	else
	{
		if (no_new_updates > 3)
		{
			valid = false;
			if (settings.enable_warnings) printf("WARNING: mocap lost visual\n");
		}
		no_new_updates++;
	}
	return 0;
}

char mocap_gen_t::update_att_from_quat(double new_mocap_quat_raw[4])
{
	if (att_updated)
	{
		printf("WARNING in update_att_from_quat: already updated attitude.\n");
		return 0; //skip if already updated
	}
	double dt_s = finddt_s(time_att); //calculate time since last update

	/* temporarily keep old values */
	double tmp_tb_NED[2];
	tmp_tb_NED[0] = att_tb_NED[0];
	tmp_tb_NED[1] = att_tb_NED[1];
	//tmp_tb_NED[2] = att_tb_NED[2]; let's use continuous heading

	for (int i = 0; i < 4; i++)
	{
		/* save new mocap data internally */
		att_quat_raw[i] = new_mocap_quat_raw[i];
	}

	/* transform from mocap frame to NED */
	att_quat_NED[0] = att_quat_raw[0];
	rotate2NED(settings.frame_type, &att_quat_NED[1], &att_quat_raw[1]);

	/* normalize quaternians just in case, but do not touch "raw" data */
	rc_normalize_quaternion_array(att_quat_NED);
	rc_normalize_quaternion_array(new_mocap_quat_raw); // just in case, since we will use these for roll pitch and yaw

	/* calculate roll pitch and yaw from quaternians */
	rc_quaternion_to_tb_array(new_mocap_quat_raw, att_tb_raw); //use normalized raw quaternians
	rc_quaternion_to_tb_array(att_quat_NED, att_tb_NED);

	/*	Heading
	continuous heading is more annoying since we have to detect spins
	also make sign negative since NED coordinates has Z point down
	*/
	double diff_mocap = att_tb_NED[2] + (num_heading_spins * TWO_PI) - continuous_heading_NED;
	// detect the crossover point at +-PI and update num yaw spins
	if (diff_mocap < -M_PI) num_heading_spins++;
	else if (diff_mocap > M_PI) num_heading_spins--;

	// finally the new value can be written
	continuous_heading_NED = att_tb_NED[2] + (num_heading_spins * TWO_PI);

	/* estimate roll, pitch and yaw rates by taking numerical derivative of the tb angles */
	att_rate_tb_raw_NED[0] = (att_tb_NED[0] - tmp_tb_NED[0]) / dt_s;
	att_rate_tb_raw_NED[1] = (att_tb_NED[1] - tmp_tb_NED[1]) / dt_s;
	att_rate_tb_raw_NED[2] = diff_mocap / dt_s; //use continuous heading to avoid singularity at a crossover point

	att_updated = true;
	time_att = rc_nanos_since_boot(); //update timer
	return 0;
}

char mocap_gen_t::update_pos_vel_from_pos(double new_mocap_pos_raw[3])
{
	if (pos_updated)
	{
		printf("WARNING in updated_pos_vel_from_pos: already updated attitude.\n");
		return 0; //skip if already updated
	}

	double dt_s = finddt_s(time_pos); //calculate time since last update

	double tmp_pos_NED[3];
	for (int i = 0; i < 3; i++)
	{
		/* temporarily keep old values */
		tmp_pos_NED[i] = pos_NED[i];

		/* save new mocap data internally */
		pos_raw[i] = new_mocap_pos_raw[i];
	}

	/* transform from mocap frame to NED */
	rotate2NED(settings.frame_type, pos_NED, pos_raw);

	/* estimate velocity by taking numerical derivative of position */
	for (int i = 0; i < 3; i++) vel_raw_NED[i] = (pos_NED[i] - tmp_pos_NED[i]) / dt_s;

	pos_updated = true;
	time_pos = rc_nanos_since_boot(); //update timer
	return 0;
}

char mocap_gen_t::march(void)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in march: mocap not initialized.\n");
		return -1;
	}
	if (att_updated)
	{
		att_lp.march(att_rate_tb_filtered_NED, att_rate_tb_raw_NED);
		att_updated = false;
	}
	if (pos_updated)
	{
		vel_lp.march(vel_filtered_NED, vel_raw_NED);
		pos_updated = false;
	}

	return 0;
}

char mocap_gen_t::reset(void)
{
	if (unlikely(!initialized))
	{
		fprintf(stderr, "ERROR in reset: mocap not initialized.\n");
		return -1;
	}
	char tmp = att_lp.reset();
	if (vel_lp.reset() < 0) tmp = -1;
	att_updated = false;
	pos_updated = false;
	return tmp;
}

void mocap_gen_t::cleanup(void)
{
	if (!initialized) return;
	att_lp.cleanup();
	vel_lp.cleanup();
	initialized = false;
	pos_updated = false;
	att_updated = false;
	return;
}


/* Data retrieval */
uint64_t mocap_gen_t::get_time(void)
{
	return time;
}
uint64_t mocap_gen_t::get_time_att(void)
{
	return time_att;
}
uint64_t mocap_gen_t::get_time_pos(void)
{
	return time_pos;
}
void mocap_gen_t::get_quat_raw(double* buff)
{
	for (int i = 0; i < 4; i++) buff[i] = att_quat_raw[i];
	return;
}
void mocap_gen_t::get_quat(double* buff)
{
	for (int i = 0; i < 4; i++) buff[i] = att_quat_NED[i];
	return;
}
void mocap_gen_t::get_tb_raw(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = att_tb_raw[i];
	return;
}
void mocap_gen_t::get_tb(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = att_tb_NED[i];
	return;
}
void mocap_gen_t::get_tb_rate_raw(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = att_rate_tb_raw_NED[i];
	return;
}
void mocap_gen_t::get_tb_rate(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = att_rate_tb_filtered_NED[i];
	return;
}
double mocap_gen_t::get_continuous_heading(void)
{
	return continuous_heading_NED;
}

void mocap_gen_t::get_pos_raw(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = pos_raw[i];
	return;
}
void mocap_gen_t::get_pos(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = pos_NED[i];
	return;
}

void mocap_gen_t::get_vel_raw(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = vel_raw_NED[i];
	return;
}
void mocap_gen_t::get_vel(double* buff)
{
	for (int i = 0; i < 3; i++) buff[i] = vel_filtered_NED[i];
	return;
}





/** @name Logging class for IMU-9DOF
* Defines how logging should be done for this class
*/
char mocap_log_entry_t::update(mocap_gen_t& new_state, mocap_settings_t& log_settings)
{
	if (!log_settings.enable_logging) return 0; //return if disabled

	time = new_state.get_time();
	valid = new_state.is_valid();

	if (log_settings.log_att)
	{
		time_att = new_state.get_time_att();

		if (log_settings.log_raw) new_state.get_quat_raw(att_quat_raw);
		new_state.get_quat(att_quat_NED);

		if (log_settings.log_raw) new_state.get_tb_raw(att_tb_raw);
		new_state.get_tb(att_tb_NED);

		if (log_settings.log_raw) new_state.get_tb_rate_raw(att_rate_tb_raw_NED);
		new_state.get_tb_rate(att_rate_tb_filtered_NED);

		continuous_heading_NED = new_state.get_continuous_heading();
	}

	time_pos = new_state.get_time_pos();

	if (log_settings.log_raw) new_state.get_pos_raw(pos_raw);
	new_state.get_pos(pos_NED);

	if (log_settings.log_vel)
	{
		if (log_settings.log_raw) new_state.get_vel_raw(vel_raw_NED);
		new_state.get_vel(vel_filtered_NED);
	}
	
	return 0;
}
char mocap_log_entry_t::print_vec(FILE* file, double* vec_in, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%.4F", vec_in[i]);
	}
	return 0;
}

char mocap_log_entry_t::print_header_vec(FILE* file, const char* prefix, const char* var_name, int size)
{
	for (int i = 0; i < size; i++)
	{
		fprintf(file, ",%s%s_%i", prefix, var_name, i);
	}
	return 0;
}

char mocap_log_entry_t::print_header(FILE* file, const char* prefix, mocap_settings_t& log_settings)
{
	if (!log_settings.enable_logging) return 0; //return if disabled

	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(time));	
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(valid));

	if (log_settings.log_att)
	{
		fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(time_att));

		if (log_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(att_quat_raw), 4);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(att_quat_NED), 4);

		if (log_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_raw), 3);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_NED), 3);

		if (log_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(att_rate_tb_raw_NED), 3);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(att_rate_tb_filtered_NED), 3);

		fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(continuous_heading_NED));
	}
	
	fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(time_pos));
	if (log_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(pos_raw), 3);
	print_header_vec(file, prefix, GET_VARIABLE_NAME(pos_NED), 3);

	if (log_settings.log_vel)
	{
		if (log_settings.log_raw) print_header_vec(file, prefix, GET_VARIABLE_NAME(vel_raw_NED), 3);
		print_header_vec(file, prefix, GET_VARIABLE_NAME(vel_filtered_NED), 3);
	}
	return 0;
}
char mocap_log_entry_t::print_entry(FILE* file, mocap_settings_t& log_settings)
{
	if (!log_settings.enable_logging) return 0; //return if disabled

	fprintf(file, ",%" PRIu64, time);	
	fprintf(file, ",%i", valid);

	if (log_settings.log_att)
	{
		fprintf(file, ",%" PRIu64, time_att);

		if (log_settings.log_raw) print_vec(file, att_quat_raw, 4);
		print_vec(file, att_quat_NED, 4);

		if (log_settings.log_raw) print_vec(file, att_tb_raw, 3);
		print_vec(file, att_tb_NED, 3);

		if (log_settings.log_raw) print_vec(file, att_rate_tb_raw_NED, 3);
		print_vec(file, att_rate_tb_filtered_NED, 3);

		fprintf(file, ",%.4F", continuous_heading_NED);
	}

	fprintf(file, ",%" PRIu64, time_pos);
	if (log_settings.log_raw) print_vec(file, pos_raw, 3);
	print_vec(file, pos_NED, 3);

	if (log_settings.log_vel)
	{
		if (log_settings.log_raw) print_vec(file, vel_raw_NED, 3);
		print_vec(file, vel_filtered_NED, 3);
	}
	return 0;
}



int parse_mocap_gen_settings(json_object* in_json, const char* name, mocap_settings_t& sensor)
{
	struct json_object* tmp_main_json = NULL;
	struct json_object* tmp = NULL;    // temp object

	//find filter entry
	if (json_object_object_get_ex(in_json, name, &tmp_main_json) == 0)
	{
		fprintf(stderr, "ERROR: can't find %s entry\n", name);
		return -1;
	}
	if (json_object_is_type(tmp_main_json, json_type_object) == 0)
	{
		fprintf(stderr, "ERROR: %s must be an object\n", name);
		return -1;
	}

	// Parse enabled flag:
	if (parse_bool(tmp_main_json, "enable", sensor.enable))
	{
		fprintf(stderr, "ERROR: failed to parse enable flag for %s\n", name);
		return -1;
	}
	if (!sensor.enable)
	{
		/* just in case, disable all flags */
		sensor.enable_logging = false;
		sensor.enable_warnings = false;
		sensor.use_roll_pitch_rate = false;
		sensor.use_yaw_rate = false;
		sensor.use_roll_pitch = false;
		sensor.use_heading = false;
		return 0; //quit parsing if disabled
	}

	if (parse_bool(tmp_main_json, "use_roll_pitch_rate", sensor.use_roll_pitch_rate))
	{
		fprintf(stderr, "ERROR: failed to parse use_roll_pitch_rate flag for %s\n", name);
		return -1;
	}
	if (parse_bool(tmp_main_json, "use_yaw_rate", sensor.use_yaw_rate))
	{
		fprintf(stderr, "ERROR: failed to parse use_yaw_rate flag for %s\n", name);
		return -1;
	}
	if (parse_bool(tmp_main_json, "use_roll_pitch", sensor.use_roll_pitch))
	{
		fprintf(stderr, "ERROR: failed to parse use_roll_pitch flag for %s\n", name);
		return -1;
	}
	if (parse_bool(tmp_main_json, "use_heading", sensor.use_heading))
	{
		fprintf(stderr, "ERROR: failed to parse use_heading flag for %s\n", name);
		return -1;
	}

	// Parse coordinate frame type:
	if (parse_coordinate_frame_gen_type(tmp_main_json, "frame_type", sensor.frame_type))
	{
		fprintf(stderr, "ERROR: failed to parse frame_type flag for %s\n", name);
		return -1;
	}

	// Parse filter
	char buf[12+3];
	for (int i = 0; i < 3; i++)
	{
		snprintf(buf, 15, "att_filter[%i]", i); // puts string into buffer
		if (parse_signal_filter_gen_settings(tmp_main_json, buf, sensor.att_filter[i]) < 0)
		{
			fprintf(stderr, "ERROR: failed to parse filter for %s\n", name);
			return -1;
		}
	}
	for (int i = 0; i < 3; i++)
	{
		snprintf(buf, 15, "vel_filter[%i]", i); // puts string into buffer
		if (parse_signal_filter_gen_settings(tmp_main_json, buf, sensor.vel_filter[i]) < 0)
		{
			fprintf(stderr, "ERROR: failed to parse filter for %s\n", name);
			return -1;
		}
	}
	

	// Parse other entries
	if (parse_bool(tmp_main_json, "enable_warnings", sensor.enable_warnings))
	{
		fprintf(stderr, "ERROR: failed to parse enable_warnings flag for %s\n", name);
		return -1;
	}
	if (parse_bool(tmp_main_json, "enable_logging", sensor.enable_logging))
	{
		fprintf(stderr, "ERROR: failed to parse enable_logging flag for %s\n", name);
		return -1;
	}

	if (!sensor.enable_logging) return 0; //done if logging is disabled
	if (parse_bool(tmp_main_json, "log_att", sensor.log_att))
	{
		fprintf(stderr, "ERROR: failed to parse log_att flag for %s\n", name);
		return -1;
	}
	if (parse_bool(tmp_main_json, "log_vel", sensor.log_vel))
	{
		fprintf(stderr, "ERROR: failed to parse log_vel flag for %s\n", name);
		return -1;
	}
	if (parse_bool(tmp_main_json, "log_raw", sensor.log_raw))
	{
		fprintf(stderr, "ERROR: failed to parse log_raw flag for %s\n", name);
		return -1;
	}

	return 0;
}