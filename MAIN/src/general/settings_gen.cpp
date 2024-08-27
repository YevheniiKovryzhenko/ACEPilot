/*
 * settings_gen.cpp
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
 * Last Edit:  09/17/2022 (MM/DD/YYYY)
 *
 * Defines some basic functions for loading settings from JSON files
 */

#include "settings_gen.hpp"

//#include <stdbool.h>
#include <stdio.h>
#include <string.h>	// FOR str_cmp()


int parse_bool(json_object* jobj_str, const char* name, bool& value)
{
	struct json_object* tmp = NULL; // temp object
	if (json_object_object_get_ex(jobj_str, name, &tmp) == 0) {
		fprintf(stderr, "ERROR can't find %s in settings file\n", name);
		return -1;
	}
	if (json_object_is_type(tmp, json_type_boolean) == 0) {
		fprintf(stderr, "ERROR %s should be a boolean\n", name);
		return -1;
	}
	value = json_object_get_boolean(tmp);
	return 0;
}

int parse_int(json_object* jobj_str, const char* name, int& value)
{
	struct json_object* tmp = NULL; // temp object
	if (json_object_object_get_ex(jobj_str, name, &tmp) == 0) {
		fprintf(stderr, "ERROR can't find %s in settings file\n", name);
		return -1;
	}
	if (json_object_is_type(tmp, json_type_int) == 0) {
		fprintf(stderr, "ERROR %s should be an integer\n", name);
		return -1;
	}
	value = json_object_get_int(tmp);
	return 0;
}

int parse_int_positive(json_object* jobj_str, const char* name, int& value)
{
	int tmp;
	if (parse_int(jobj_str, name, tmp))
	{
		fprintf(stderr, "ERROR: failed to parse %s\n", name);
		return -1;
	}
	if (tmp < 0)
	{
		fprintf(stderr, "ERROR: failed to parse %s, must be positive\n", name);
		return -1;
	}
	value = tmp;
	return 0;
}

int parse_double(json_object* jobj_str, const char* name, double& value)
{
	struct json_object* tmp = NULL; // temp object
	if (json_object_object_get_ex(jobj_str, name, &tmp) == 0) {
		fprintf(stderr, "ERROR can't find %s in settings file\n", name);
		return -1;
	}
	if (json_object_is_type(tmp, json_type_double) == 0) {
		if (json_object_is_type(tmp, json_type_int) == 0) //try parsing int as double
		{
			fprintf(stderr, "ERROR %s should be a double or an int\n", name);
			return -1;
		}
		else
		{
			value = (double)json_object_get_int(tmp);
		}
	}
	else
	{
		value = json_object_get_double(tmp);
	}	
	return 0;
}

int parse_double_min(json_object* jobj_str, const char* name, double& new_min, double min_min)
{
	double tmp1;
	if (parse_double(jobj_str, name, tmp1))
	{
		fprintf(stderr, "ERROR: failed to parse min\n");
		return -1;
	}
	if (tmp1 < min_min)
	{
		fprintf(stderr, "ERROR: min must be greater than %f\n", min_min);
		return -1;
	}
	new_min = new_min;
	return 0;
}

int parse_double_min_max(json_object* jobj_str, double& new_min, double& new_max)
{
	double tmp1, tmp2;
	if (parse_double(jobj_str, "min", tmp1))
	{
		fprintf(stderr, "ERROR: failed to parse min\n");
		return -1;
	}
	if (parse_double(jobj_str, "max", tmp2))
	{
		fprintf(stderr, "ERROR: failed to parse max\n");
		return -1;
	}
	if (tmp1 > tmp2)
	{
		fprintf(stderr, "ERROR: min must be less than max\n");
		return -1;
	}
	new_min = tmp1;
	new_max = tmp2;
	return 0;
}

int parse_double_positive(json_object* jobj_str, const char* name, double& value)
{
	double tmp;
	if (parse_double(jobj_str, name, tmp))
	{
		fprintf(stderr, "ERROR: failed to parse double\n");
		return -1;
	}
	if (tmp < 0.0)
	{
		fprintf(stderr, "ERROR: failed to parse double\n");
		return -1;
	}
	value = tmp;
	return 0;
}