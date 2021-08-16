/*
 * main.hpp
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
 * Last Edit:  08/16/2020 (MM/DD/YYYY)
 */

#ifndef MAIN_HPP
#define MAIN_HPP

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <inttypes.h> // for PRIu64
#include <string.h>
#include <stdio.h>
#include <fcntl.h>


#include <rc/pthread.h>
#include <rc/time.h> // for nanos
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rc/mpu.h>
#include <rc/dsm.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/led.h>
#include <rc/cpu.h>
#include <rc/encoder.h>
#include <signal.h>


#include "state_estimator.h"
#include "tools.h"
#include "gps.h"
#include "serial_com.h"
#include "AdafruitGPS_cmds.hpp"
#include "adafruit_servo_driver.hpp"
#include "servos.hpp"
#include "lwgps.h"
#include "lwrb.h"
#include "rc_pilot_defs.h"
#include "input_manager.hpp"
#include "settings.h"
#include "state_machine.hpp"
#include "log_manager.hpp"
#include "printf_manager.hpp"
#include "xbee_receive.h"
#include "setpoint_manager.hpp"
#include "benchmark.h"
#include "feedback.hpp"
#include "mix_servos.hpp"
#include "path.hpp"

#endif //MAIN_HPP