/*
 * coordinate_frames_gen.hpp
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
 * Last Edit:  08/29/2022 (MM/DD/YYYY)
 *
 * Summary :
 * Here are defined general coordinate frames and related functionality
 */

#ifndef COORDINATE_FRAMES_GEN_HPP
#define COORDINATE_FRAMES_GEN_HPP
#ifdef __cplusplus
extern "C"
{
#endif

 /*
		 Coordinate frame types

	 Assume that we should have:
	 * North
	 *	X
	 *	|
	 *  / \
	 *	|
	 *	|
	 *	Z ---->  Y East
	 *  Down
	 If any of the sensors are pointing in other directions,
	 You need to select the respective coordinate fram (w.r.to NED)
	 */
typedef enum coordinate_frame_t {
	/* Axis:	X		Y		Z */
	NED,	//North		East	Down
	NWU,	//North		West	Up
	NUE,	//North		Up		East
	NDW,	//North		Down	West
	ENU,	//East		North	Up
	ESD,	//East		South	Down
	EUS,	//East		Up		South
	EDN,	//East		Down	North
	UNW,	//Up		North	West
	USE,	//Up		South	East
	UEN,	//Up		East	North
	UWS,	//Up		West	South
	DNE		//Down		North	East
}coordinate_frame_t;

/* function to quickly converst input 3D data into NED coordiante frame */
char rotate2NED(coordinate_frame_t type, double* out, double in[3]);


#ifdef __cplusplus
}
#endif
#endif // !COORDINATE_FRAMES_GEN_HPP




