/*
 * gen_thread.hpp
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
 * Last Edit:  05/20/2020 (MM/DD/YYYY)
 *
 * Summary :
 *
 *
 */


#ifndef GEN_THREAD
#define GEN_THREAD

#include <rc/pthread.h>
enum thread_policy
{
	OTHER = 0,
	FIFO = 1, // realtime 
	RR = 2 // realtime 
};



class gen_thread_t
{
private:
	pthread_t thread; // thread
	bool initialized; // thread parameters are initialized
	bool started; // thread is running
	int priority; // thread priority 1-99
	thread_policy policy; // thread type/policy

public:
	/**
	 * @brief      Sets the parameters for the thread.
	 *
	 * @return     0 on success, -1 on failure
	 */
	int init(int PRIORITY, thread_policy POLICY);

	/**
	 * @brief      Starts the thread. Must be initialized.
	 *
	 * @return     0 on success, -1 on failure
	 */
	int start(void* (*func)(void*));

	/**
	 * @brief      Stops/Kills the thread.
	 *
	 * @return     0 on sucess and clean exit, -1 on exit timeout/force close.
	 */
	int stop(float TOUT);

	/**
	 * @brief      Returns true if thread is started.
	 */
	bool is_started(void);

};

#endif // !GEN_THREAD
