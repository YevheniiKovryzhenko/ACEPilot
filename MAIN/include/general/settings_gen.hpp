/*
 * settings_gen.hpp
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
 * Last Edit:  09/02/2022 (MM/DD/YYYY)
 *
 * Defines some basic functions for loading settings from JSON files
 */

#ifndef SETTINGS_GEN_HPP
#define SETTINGS_GEN_HPP

#include <json.h>


int parse_bool(json_object* jobj_str, const char* name, bool& value);

int parse_int(json_object* jobj_str, const char* name, int& value);

int parse_int_positive(json_object* jobj_str, const char* name, int& value);

int parse_double(json_object* jobj_str, const char* name, double& value);

int parse_double_min(json_object* jobj_str, const char* name, double& new_min, double min_min);

int parse_double_min_max(json_object* jobj_str, double& new_min, double& new_max);

int parse_double_positive(json_object* jobj_str, const char* name, double& value);

#endif // !SETTINGS_GEN_HPP
