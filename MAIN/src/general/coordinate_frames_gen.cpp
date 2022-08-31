/*
 * coordinate_frames_gen.cpp
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
 * Last Edit:  08/31/2022 (MM/DD/YYYY)
 *
 * Summary :
 * Here are defined general coordinate frames and related functionality
 */

#include "coordinate_frames_gen.hpp"
#include <stdio.h>

char rotate2NED(coordinate_frame_t type, double* out, double in[3])
{
	switch (type)
	{
	case NED:
		out[0] = in[0];
		out[1] = in[1];
		out[2] = in[2];
		return 0;
	case NWU:
		out[0] = in[0];
		out[1] = -in[1];
		out[2] = -in[2];
		return 0;
	case NUE:
		out[0] = in[0];
		out[1] = in[2];
		out[2] = -in[1];
		return 0;
	case NDW:
		out[0] = in[0];
		out[1] = -in[2];
		out[2] = in[1];
		return 0;
	case ENU:
		out[0] = in[1];
		out[1] = in[0];
		out[2] = -in[2];
		return 0;
	case ESD:
		out[0] = -in[1];
		out[1] = in[0];
		out[2] = in[2];
		return 0;
	case EUS:
		out[0] = -in[2];
		out[1] = in[0];
		out[2] = -in[1];
		return 0;
	case EDN:
		out[0] = in[2];
		out[1] = in[0];
		out[2] = in[1];
		return 0;
	case UNW:
		out[0] = in[1];
		out[1] = -in[2];
		out[2] = -in[0];
		return 0;
	case USE:
		out[0] = -in[1];
		out[1] = in[2];
		out[2] = -in[0];
		return 0;
	case UEN:
		out[0] = in[2];
		out[1] = in[1];
		out[2] = -in[0];
		return 0;
	case UWS:
		out[0] = -in[2];
		out[1] = -in[1];
		out[2] = -in[0];
		return 0;
	case DNE:
		out[0] = in[1];
		out[1] = in[2];
		out[2] = in[0];
		return 0;
	default:
		fprintf(stderr, "ERROR in rotate2NED: transformation is not supported (wrong coordinate frame type)\n");
		return -1;
	}
	fprintf(stderr, "ERROR in rotate2NED: something went wrong, should not have reached this line...\n");
	return -1;
}