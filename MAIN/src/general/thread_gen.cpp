/*
 * thread_gen.cpp
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
 * Last Edit:  05/29/2020 (MM/DD/YYYY)
 *
 * Summary :
 * 
 *
 */
#include <rc/time.h>
#include <stdio.h>
#include "thread_gen.hpp"

int thread_gen_t::init(int PRIORITY, thread_policy POLICY)
{    
    if (PRIORITY > 99 || PRIORITY < 1)
    {
        printf("ERROR in init: PRIORITY must be between 1 and 99\n");
        return -1;
    }
    priority = PRIORITY;
    policy = POLICY;

    initialized = true;
    started = false;

    return 0;
}

int thread_gen_t::start(void* (*func)(void*))
{
    if (!initialized)
    {
        printf("ERROR in start: thread not initialized\n");
        return -1;
    }
    if (rc_pthread_create(&thread, func, NULL,
        policy, priority) == -1) {
        printf("ERROR in start: failed to start thread\n");
        return -1;
    }
    rc_usleep(50000);
    started = true;
    return 0;
}
int thread_gen_t::stop(float TOUT)
{
    int ret = 0;
    if (started) {
        // wait for the thread to exit
        ret = rc_pthread_timed_join(thread, NULL, TOUT);
        if (ret == 1) fprintf(stderr, "WARNING in stop: exit timeout\n");
        else if (ret == -1) fprintf(stderr, "ERROR in stop: failed to join thread\n");
    }
    started = false;
    return ret;
}

bool thread_gen_t::is_started(void)
{
    return started;
}
