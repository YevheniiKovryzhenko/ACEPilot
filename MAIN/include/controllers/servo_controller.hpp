/*
 * servo_controller.hpp
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
 * Last Edit:  07/28/2020 (MM/DD/YYYY)
 */

#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>

#include <rc/math.h>
#include <rc/time.h>
#include <rc/math/filter.h>

#include "mix_servos.hpp"

class feedback_servo_controller_t
{
private:
	bool initialized;

	int mix_all_control(double(&u)[MAX_SERVO_INPUTS], double(&mot)[MAX_SERVOS]);

public:

	int init(void);

	int march(double(&u)[MAX_SERVO_INPUTS], double(&mot)[MAX_SERVOS]);

	int reset(void);

};
#endif // SERVO_CONTROLLER_HPP