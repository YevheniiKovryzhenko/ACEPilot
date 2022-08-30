/*
 * printf_manager.hpp
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
 * Summary:
 * 
 *	Functions to start and stop the printf mnaager which is a
 *  separate thread printing data to the console for debugging.
 * 
 * Last Edit:  08/26/2022 (MM/DD/YYYY)
 *
 */


#ifndef PRINTF_MANAGER_H
#define PRINTF_MANAGER_H


/**
* @brief      Start the printf_manager thread which should be the only thing
*             printing to the screen besides error messages from other threads.
*
* @return     0 on success, -1 on failure
*/
int printf_init(void);


/**
* @brief      Waits for the printf manager thread to exit.
*
* @return     0 on clean exit, -1 on exit time out/force close
*/
int printf_cleanup(void);


/**
* @brief      Only used by printf_manager right now, but could be useful
* elsewhere.
*
* @param[in]  mode  The mode
*
* @return     0 on success or -1 on error
*/
int print_flight_mode(flight_mode_t mode);


#endif //PRINTF_MANAGER_H
