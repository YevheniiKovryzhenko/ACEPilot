/*
 * trajectories_common.cpp
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
 * Contains all the automated trajectory guidance and related functionality.
 * This establishes framework of how to start/end automated guidance and safely
 * mannage setpoints.
 *
 */

#include <math.h>

#include <rc/time.h>

#include "rc_pilot_defs.hpp"
#include "setpoint_manager.hpp"
#include "settings.hpp"
#include "tools.h"
#include "input_manager.hpp"
#include "state_estimator.hpp"
#include "state_machine.hpp"
#include "path.hpp"

#include "setpoint_guidance.hpp"

 // preposessor macros
#ifndef unlikely
#define unlikely(x)	__builtin_expect (!!(x), 0)
#endif // !unlikely

#ifndef likely
#define likely(x)	__builtin_expect (!!(x), 1)
#endif // !likely

setpoint_guidance_t setpoint_guidance{};


void cubic_guide_t::set(double new_xi, double new_xf, double new_xdoti, double new_xdotf, float new_tt)
{
    xi = new_xi;
    xf = new_xf;
    xdoti = new_xdoti;
    xdotf = new_xdotf;
    tt = new_tt;
    started = false;
    completed = false;
    return;
}

void cubic_guide_t::restart(void)
{
    started     = false;
    completed   = false;
}

bool cubic_guide_t::march(double &current_pos)
{
    if (completed)
    {
        printf("\nWARNING in march: trying to march cubic guide when completed");
        return true;
    }
    if (!started)
    {
        time_ns = rc_nanos_since_boot();
        started = true;
    }


    if (finddt_s(time_ns) >= tt)
    {
        return true;
    }
    else
    {
        // use cubic function and time to find the current position:
        current_pos =
            cubicPol(xi, xf, xdoti, xdotf, tt, finddt_s(time_ns));
        return false;
    }
    return false;
}


void circ_guide_t::set(double new_T, double new_R, double new_tt, bool new_dir_yaw_cw,\
    double new_x_i, double new_y_i, double new_yaw_i)
{
    x_i = new_x_i;
    y_i = new_y_i;
    yaw_i = new_yaw_i;
    tt = new_tt;
    dir_yaw_cw = new_dir_yaw_cw;
    T = new_T;
    R = new_R;

    omega = 2.0 * M_PI / T; //angular velocity in a turn

    started = false;
    completed = false;
    return;
}

void circ_guide_t::restart(void)
{
    started = false;
    completed = false;
}

bool circ_guide_t::march(double& X, double& Y, double &Yaw)
{
    if (completed)
    {
        printf("\nWARNING in march: trying to march circ guide when completed");
        return true;
    }
    if (!started)
    {
        time_ns = rc_nanos_since_boot();
        started = true;
    }

    double tmp = finddt_s(time_ns);
    if (tmp >= tt)
    {
        return true;
    }
    else
    {
        X = R * sin(omega * tmp) + x_i;
        Y = R * cos(omega * tmp) + y_i;
        if (dir_yaw_cw)
        {
            Yaw = -omega * tmp + yaw_i;
        }
        else
        {
            Yaw = omega * tmp + yaw_i;
        }
        return false;
    }
    return false;
}

//--------------------------------Z-----------------------------------------//
/* 
* Altitude/vertical/Z setpoint management and guidance general functions 
* Intended to be independent of XY guidance. Each Z algorithm must have 
* a way to start and stop itself and will use the below format. Note,
* only one Z guidance algorithm should be running at once to avoid overwrites
* of commands. Moreover, each time Z marching should be properly finished 
* by setting st_Z = true.
*/
int setpoint_guidance_t::march_Z(void)
{
    
    if (unlikely(!Z_initialized))
    {
        printf("\nERROR in march_Z: not initialized");
        return -1;
    }

    // check if already flagged as complete
    if (st_Z)
    {
        reset_Z();
        return 0;
    }
    // run only once when Z guidance is activated
    if (!last_en_Z)
    {
        Z_time_ns = rc_nanos_since_boot();   // capture time of initialization
        Z_initial = setpoint.Z.value.get();
        last_en_Z = true;                    // flag that Z guidance is active now
    }

    //run Z guidance algorithms if enabled (only 1 at a time)
    if (en_land && !land_finished)
    {
        int tmp = march_land();
        if (unlikely(tmp == -1))
        {
            printf("\nERROR in march_Z: failed to march landing");
            return -1;
        }
        else if (unlikely(tmp == -2))
        {
            printf("\nWARNING: Failed to complete landing, check vertical speed and constraints");
            land_finished = true;
            en_land = false;
            return 0;
        }
    }
    else if (en_takeoff && !takeoff_finished)
    {
        if (unlikely(march_takeoff() == -1))
        {
            printf("\nERROR in march_Z: failed to march takeoff");
            return -1;
        }
        return 0;
    }

    setpoint.Z.value.increment(Z_dot * DT); //Z positive down (so Z_dot must be negative)
    
    return 0;
}

void setpoint_guidance_t::reset_Z(void)
{
    last_en_Z   = false;
    st_Z        = false;
    en_Z        = false;
    en_takeoff  = false;
    en_land     = false;
    Z_dot       = 0.0;  // stop moving the setpoint down
    Z_initial   = setpoint.Z.value.get();

    return;
}


int setpoint_guidance_t::init_Z(void)
{
    if (unlikely(Z_initialized))
    {
        printf("\nWARNING in init_Z: already initialized");
        return 0;
    }
    reset_Z();


    init_land();
    init_takeoff();

    Z_initialized = true;
    return 0;
}

bool setpoint_guidance_t::is_Z_initialized(void)
{
    return Z_initialized;
}

bool setpoint_guidance_t::is_Z_en(void)
{
    return en_Z;
}

bool setpoint_guidance_t::get_state_Z(void)
{
    return st_Z;
}



int setpoint_guidance_t::march_land(void)
{
    //-------------Auto-Land---------------//
    // Landing algorithm basic logic is the following:
    // 1. run only if activated externally
    // 2. check if the drone is already on the ground
    //	2.1 reset the start-up flag & finish landing sequence
    // 3. detect the time of initiation
    // 4. decrease the altitude using a set velocity after some delay
    // should work at any altitude and is entirely velocity dependant -
    // the altitude will be decreased at a constant rate until landing has been detected

    if (unlikely(!is_Z_initialized()))
    {
        printf("\nERROR in land: not initialized");
        return -1;
    }

    // wait a second, then start decreasing altitude
    // need to wait in case altitude hold was just activated and
    // altitude is not stable yet
    if (finddt_s(Z_time_ns) > land_start_delay_s)
    {
        Z_dot = V_land;         // start moving the setpoint down
    }
    else
    {
        if (finddt_s(Z_time_ns) < land_start_delay_s)
        {
            Z_dot = V_land * (finddt_s(Z_time_ns)) / land_start_delay_s;         // start moving the setpoint down
        }
        else
        {
            Z_dot = 0.0;            // stabilize altitude
        }        
    }

    //-----Landing Detection----//
    if (setpoint.Z.value.get() < (state_estimate.get_Z() - XYZ_MAX_ERROR))
    {
        printf("\n WARNING: Flying too far up! Exceeding XYZ_MAX_ERROR, can't keep up \n");
        reset_Z();                              //terminante any Z guidance
        land_finished = true;                   //flag landing as finished to prevent restart
        en_land = false;                        //disable landing
        return -2;                              //failed to land (velocity can be too high)
    }

    if (setpoint.Z.value.get() > (state_estimate.get_Z() + XYZ_MAX_ERROR))
    {
        st_Z = true;                            // landed - let the Z guidance algorithm finish on it's own
        land_finished = true;                   //flag landing as finished to prevent restart
        en_land = false;                        //disable landing
        printf("\n Detected Touchdown! \n");
        return 0;
    }
    return 0;
}

void setpoint_guidance_t::init_land(void)
{
    reset_land();
    land_finished   = false;
    en_land         = false;
    return;
}

void setpoint_guidance_t::reset_land(void)
{
    land_start_delay_s  = land_start_def_delay_s;
    V_land              = settings.V_max_land;
    return;
}

void setpoint_guidance_t::start_land(void) //intended to allow running at each itter.
{
    if (!en_Z && !en_land && !land_finished)
    {
        en_Z        = true; // allow Z guidance
        en_land     = true; // start landing
        setpoint.Z.value.set(state_estimate.get_Z()); //zero out the altitude error
        printf("\nStarting landing algorithm....");
    }//otherwise do nothing
    
    return;
}

/* 
* Use this function to restart landing in case of failure. Should be triggered using 
* radio button or one-time command signal.
*/
int setpoint_guidance_t::restart_land(void) //intended as a trigger (do not run at each itter.)
{
    if (en_Z)
    {
        printf("\n Error in restart_land: Z guidance is already tunning, must finish or stop the current algorithm");
        return -1;
    }
    reset_Z();
    land_finished = false;
    start_land();
    return 0;
}

void setpoint_guidance_t::set_V_land(double new_V_max)
{
    if (fabs(new_V_max) < settings.V_max_land) //don't let exceed the maximum
    {
        V_land = fabs(new_V_max); // vertical velocity is positive down
    }
    else
    {
        V_land = settings.V_max_land;
    }

    return;
}

void setpoint_guidance_t::set_land_start_delay(double new_delay_s)
{
    if (new_delay_s < 0.0)
    {
        land_start_delay_s = 0.0;
    }
    else
    {
        land_start_delay_s = new_delay_s;
    }
    return;
}
/**
* @brief		automated takeoff to a set elevation above the current altitude
*
* @return     -1 on error, otherwise 0
*/
int setpoint_guidance_t::march_takeoff(void)
{
    //-------------Auto-Take-off---------------//
    // Take-off algorithm basic logic is the following:
    // 1. run only if activated externally using "setpoint.en_Z_takeoff=1"
    // 2. check if the drone is has taken off to target alt.
    //	2.1 reset the start-up flag & finish take-off sequence
    // 3. detect the time of initiation
    // 4. move the setpoint.Z
    if (unlikely(!Z_initialized))
    {
        printf("\nERROR in march_takeoff: Z guidance is not initialized");
        return -1;
    }
    if (unlikely(!en_Z))
    {
        printf("\nERROR in march_takeoff: Z guidance not enabled");
        return -1;
    }
    if (unlikely(!en_takeoff))
    {
        printf("\nWARNING in march_takeoff: trying to march while not enabled");
        return 0;
    }

    setpoint.Z_throttle_0 = settings.hover_throttle;      //this is needed for altitude hold
    if (Z_cubic.march(*setpoint.Z.value.get_pt())) st_Z = true;
    return 0;
}

void setpoint_guidance_t::reset_takeoff(void)
{
    t_takeoff_s             = settings.t_takeoff;
    takeoff_height          = settings.height_takeoff;
    return;
}

void setpoint_guidance_t::init_takeoff(void)
{
    en_takeoff          = false;
    takeoff_finished    = false;
    reset_takeoff();
    return;
}

void setpoint_guidance_t::set_takeoff_height(double height)
{
    if (!en_takeoff)
    {
        takeoff_height = fabs(height);
        Z_cubic.set(Z_initial, Z_initial - takeoff_height, 0,
            0, t_takeoff_s);
    }
    return;
}

void setpoint_guidance_t::start_takeoff(void) //intended to allow running at each itter.
{
    if (!en_Z && !en_takeoff && !takeoff_finished)
    {
        en_Z        = true; // allow Z guidance
        en_takeoff  = true; // start takeoff
        setpoint.Z.value.set(state_estimate.get_Z()); //zero out the error

        Z_cubic.set(Z_initial, Z_initial - takeoff_height, 0,
            0, t_takeoff_s);
        printf("\nStarting takeoff algorithm....");
    }//otherwise do nothing

    return;
}

/*
* Use this function to restart takeoff in case of failure. Should be triggered using
* radio button or one-time command signal.
*/
int setpoint_guidance_t::restart_takeoff(void) //intended as a trigger (do not run at each itter.)
{
    if (en_Z)
    {
        printf("\n Error in restart_takeoff: Z guidance is already tunning, must finish or stop the current algorithm");
        return -1;
    }
    reset_Z();
    takeoff_finished = false;
    start_takeoff();
    return 0;
}


//----------------------------------XY------------------------------------------//
/*
* Horizontal position setpoint management and guidance general functions.
* Intended to be independent of Z guidance. Each XY algorithm must have
* a way to start and stop itself and will use the below format. Note,
* only one XY guidance algorithm should be running at once to avoid overwrites
* of commands. Moreover, each time XY marching should be properly finished
* by setting st_XY = true.
*/
int setpoint_guidance_t::march_XY(void)
{

    if (unlikely(!XY_initialized))
    {
        printf("\nERROR in march_XY: not initialized");
        return -1;
    }

    // check if already flagged as landed
    if (st_XY)
    {
        reset_XY();
        return 0;
    }
    // run only once when XY guidance is activated
    if (!last_en_XY)
    {
        XY_time_ns  = rc_nanos_since_boot();   // capture time of initialization
        X_initial   = setpoint.XY.x.value.get();
        Y_initial   = setpoint.XY.y.value.get();
        Yaw_initial = setpoint.ATT.z.value.get();
        last_en_XY  = true;                    // flag that Z guidance is active now
    }

    //run XY guidance algorithms if enabled (only 1 at a time)
    if (en_square)
    {
        if (unlikely(march_square() == -1))
        {
            printf("\nERROR in march_XY: failed to march square");
            return -1;
        }
        return 0;
    }
    else if (en_circ)
    {
        if (unlikely(march_circ() == -1))
        {
            printf("\nERROR in march_XY: failed to march circ");
            return -1;
        }
        return 0;
    }

    setpoint.XY.x.value.increment(X_dot * DT);
    setpoint.XY.y.value.increment(Y_dot * DT);
    if (en_yaw) setpoint.ATT.z.value.increment(Yaw_dot * DT);
    return 0;
}

void setpoint_guidance_t::reset_XY(void)
{
    last_en_XY  = false;
    st_XY       = false;
    en_XY       = false;
    en_yaw      = false;
    en_square   = false;
    en_circ     = false;
    X_dot       = 0.0;          // stop moving the setpoint
    Y_dot       = 0.0;          // stop moving the setpoint
    X_initial   = setpoint.XY.x.value.get();
    Y_initial   = setpoint.XY.y.value.get();
    Yaw_initial = setpoint.ATT.z.value.get();
    XY_waypt    = 0;

    XY_start_delay_s = settings.XY_start_delay_s;
    XY_waypt_delay_s = settings.XY_waypt_delay_s;

    return;
}


int setpoint_guidance_t::init_XY(void)
{
    if (unlikely(XY_initialized))
    {
        printf("\nWARNING in init_XY: already initialized");
        return 0;
    }
    reset_XY();

    init_square();
    init_circ();

    XY_initialized = true;
    return 0;
}

bool setpoint_guidance_t::get_state_XY(void)
{
    return st_XY;
}

bool setpoint_guidance_t::is_XY_initialized(void)
{
    return XY_initialized;
}

bool setpoint_guidance_t::is_XY_en(void)
{
    return en_XY;
}

int setpoint_guidance_t::march_square(void)
{
    /* -------------- AUTONOMOUS XY TRACKING-----------//
    This algorithm performs simple square trajectory using cubic polynomials.
    The basic logic is as the following:
    1. Assume AUTONOMOUS has just been activated in air 
    Assuming Hover state, execute the trajectory
        1.1 using initial time and position apply dv
        1.f stabilize to hover (might need to write a separate function for this)
    */
    if (unlikely(!XY_initialized))
    {
        printf("\nERROR in march_square: XY guidance not initialized");
        return -1;
    }
    if (unlikely(!en_XY))
    {
        printf("\nERROR in march_square: XY guidance not enabled");
        return -1;
    }
    if (unlikely(!en_square))
    {
        printf("\nWARNING in march_square: trying to do_square while not enabled");
        return 0;
    }
    bool tmp1, tmp2;

    // begin main sequence -- use waypoints for transition
    switch (XY_waypt)
    {
    case 0:
        if (finddt_s(XY_time_ns) > XY_start_delay_s) //assume we are at the center
        {
            X_cubic.set(X_initial, square_X_offset / 2.0 + X_initial, 0,
                0, square_X_time_s / 2.0);
            Y_cubic.set(Y_initial, square_Y_offset / 2.0 + Y_initial, 0,
                0, square_Y_time_s / 2.0);
            XY_waypt = 1;
            break;
        }
        else
        {
            //keep current position:
            setpoint.XY.x.value.set(X_initial);
            setpoint.XY.y.value.set(Y_initial);
            break;
        }
    case 1:
        // move to first corner
        tmp1 = Y_cubic.march(*setpoint.XY.y.value.get_pt()); // always march both
        tmp2 = X_cubic.march(*setpoint.XY.x.value.get_pt());
        if (tmp1 && tmp2)
        {
            XY_waypt = 2;
            XY_time_ns = rc_nanos_since_boot();
            break;
        }
        else break;
    case 2:
        if (finddt_s(XY_time_ns) > XY_waypt_delay_s)
        {
            X_cubic.set(square_X_offset / 2.0 + X_initial, -square_X_offset / 2.0 + X_initial, 0,
                0, square_X_time_s);
            XY_waypt = 3;
            break;
        }
        else
        {
            //keep current position:
            setpoint.XY.x.value.set(square_X_offset / 2.0 + X_initial);
            setpoint.XY.y.value.set(square_Y_offset / 2.0 + Y_initial);
            setpoint.ATT.z.value.set(Yaw_initial);
            break;
        }
    
    case 3:
        // move to corner 2:
        if (X_cubic.march(*setpoint.XY.x.value.get_pt()))
        {
            XY_waypt = 4;
            XY_time_ns = rc_nanos_since_boot();
            break;
        }
        else break;
    case 4:
        if (finddt_s(XY_time_ns) > XY_waypt_delay_s)
        {
            Y_cubic.set(square_Y_offset / 2.0 + Y_initial, -square_Y_offset / 2.0 + Y_initial, 0,
                0, square_Y_time_s);
            XY_waypt = 5;
            break;
        }
        else
        {
            //keep current position:
            setpoint.XY.x.value.set(-square_X_offset / 2.0 + X_initial);
            setpoint.XY.y.value.set(square_Y_offset / 2.0 + Y_initial);
            break;
        }
    case 5:
        // move to corner 3:
        if (Y_cubic.march(*setpoint.XY.y.value.get_pt()))
        {
            XY_waypt = 6;
            XY_time_ns = rc_nanos_since_boot();
            break;
        }
        else break;
    case 6:
        if (finddt_s(XY_time_ns) > XY_waypt_delay_s)
        {
            X_cubic.set(-square_X_offset / 2.0 + X_initial, square_X_offset / 2.0 + X_initial, 0,
                0, square_X_time_s);
            XY_waypt = 7;
            break;
        }
        else
        {
            //keep current position:
            setpoint.XY.x.value.set(-square_X_offset / 2.0 + X_initial);
            setpoint.XY.y.value.set(-square_Y_offset / 2.0 + Y_initial);
            break;
        }
    case 7:
        // move to corner 4:
        if (X_cubic.march(*setpoint.XY.x.value.get_pt()))
        {
            XY_waypt = 8;
            XY_time_ns = rc_nanos_since_boot();
            break;
        }
        else break;
    case 8:
        if (finddt_s(XY_time_ns) > XY_waypt_delay_s)
        {
            Y_cubic.set(-square_Y_offset / 2.0 + Y_initial, square_Y_offset / 2.0 + Y_initial, 0,
                0, square_Y_time_s);
            XY_waypt = 9;
            break;
        }
        else
        {
            //keep current position:
            setpoint.XY.x.value.set(square_X_offset / 2.0 + X_initial);
            setpoint.XY.y.value.set(-square_Y_offset / 2.0 + Y_initial);
            break;
        }
    case 9:
        // move back to corner 1:
        if (Y_cubic.march(*setpoint.XY.y.value.get_pt()))
        {
            XY_time_ns = rc_nanos_since_boot();
            XY_waypt = 10;
            break;
        }
        else break;
    case 10:
        if (finddt_s(XY_time_ns) > XY_start_delay_s)
        {
            Y_cubic.set(setpoint.XY.y.value.get(), Y_initial, 0,
                0, square_Y_time_s / 2.0);
            X_cubic.set(setpoint.XY.x.value.get(), X_initial, 0,
                0, square_X_time_s / 2.0);
            XY_waypt = 11;
            break;
        }
        else
        {
            //keep current position:
            // we don't know where we are (depends on tt)
            // assume we reached our setpoint and want to go back
            break;
        }
    case 11:
        // move back to the origin:
        tmp1 = Y_cubic.march(*setpoint.XY.y.value.get_pt());
        tmp2 = X_cubic.march(*setpoint.XY.x.value.get_pt());
        if (tmp1 && tmp2)
        {
            XY_waypt = 12;
            XY_time_ns = rc_nanos_since_boot();
            break;
        }
        else break;
    case 12:
        if (finddt_s(XY_time_ns) > XY_waypt_delay_s)
        {
            st_XY = true;
            square_finished = true;
            en_square = false;
            return 0;
        }
        else
        {
            //keep current position:
            setpoint.XY.x.value.set(X_initial);
            setpoint.XY.y.value.set(Y_initial);
            break;
        }
        //---------End of trajectory-------------//
    }
    return 0;
}

void setpoint_guidance_t::init_square(void)
{
    en_square = false;
    reset_square();
    return;
}

void setpoint_guidance_t::reset_square(void)
{
    square_X_offset = settings.square_X_offset;
    square_Y_offset = settings.square_Y_offset;
    square_X_time_s = settings.square_X_time_s;
    square_Y_time_s = settings.square_Y_time_s;
    return;
}

void setpoint_guidance_t::start_square(void) //intended to allow running at each itter.
{
    if (!en_XY && !en_square && !square_finished)
    {
        en_XY = true; // allow XY guidance
        en_square = true; // start square guidance
        setpoint.XY.x.value.set(state_estimate.get_X()); //zero out the error
        setpoint.XY.y.value.set(state_estimate.get_Y()); //zero out the error

        printf("\nStarting square trajectory algorithm....");
    }//otherwise do nothing
    
    return;
}

/*
* Use this function to restart takeoff in case of failure. Should be triggered using
* radio button or one-time command signal.
*/
int setpoint_guidance_t::restart_square(void) //intended as a trigger (do not run at each itter.)
{
    if (en_XY)
    {
        printf("\n Error in restart_square: XY guidance is already tunning, must finish or stop the current algorithm");
        return -1;
    }
    reset_XY();
    square_finished = false;
    start_square();
    return 0;
}

int setpoint_guidance_t::march_circ(void)
{
    if (unlikely(!XY_initialized))
    {
        printf("\nERROR in march_circ: XY guidance not initialized");
        return -1;
    }
    if (unlikely(!en_XY))
    {
        printf("\nERROR in march_circ: XY guidance not enabled");
        return -1;
    }
    if (unlikely(!en_circ))
    {
        printf("\nWARNING in march_circ: trying to do_square while not enabled");
        return 0;
    }


    switch (XY_waypt)
    {
    case 0:
        if (finddt_s(XY_time_ns) > XY_start_delay_s)
        {
            XY_waypt = 1;
            break;
        }
        else
        {
            //keep current position:
            setpoint.XY.x.value.set(X_initial);
            setpoint.XY.y.value.set(Y_initial);
            setpoint.ATT.z.value.set(Yaw_initial);
            break;
        }
    case 1:
        // move to corner edge of the circle (theta = 0):
        if (Y_cubic.march(*setpoint.XY.y.value.get_pt()))
        {
            XY_waypt = 2;
            XY_time_ns = rc_nanos_since_boot();
            break;
        }
        else break;
    case 2:
        if (finddt_s(XY_time_ns) > XY_waypt_delay_s)
        {
            XY_waypt = 3;
            break;
        }
        else
        {
            //keep current position:
            setpoint.XY.x.value.set(X_initial);
            setpoint.XY.y.value.set(Y_initial + turn_radius);
            setpoint.ATT.z.value.set(Yaw_initial);
            break;
        }
    case 3:
        // move arround the circle untill time is up
        if (circ_guide.march(*setpoint.XY.x.value.get_pt(), *setpoint.XY.y.value.get_pt(), *setpoint.ATT.z.value.get_pt()))
        {
            XY_waypt = 4;
            XY_time_ns = rc_nanos_since_boot();
            break;
        }
        else break;
    case 4:
        if (finddt_s(XY_time_ns) > XY_start_delay_s)
        {
            Y_cubic.set(setpoint.XY.y.value.get(), Y_initial, 0,
                0, TWO_PI * turn_radius * turn_radius / turn_period);
            X_cubic.set(setpoint.XY.x.value.get(), X_initial, 0,
                0, TWO_PI * turn_radius * turn_radius / turn_period);
            XY_waypt = 5;
            break;
        }
        else
        {
            //keep current position:
            // we don't know where we are (depends on tt)
            // assume we reached our setpoint and want to go back
            break;
        }
    case 5:
        // move back from edge of the circle to the center:
        if (Y_cubic.march(*setpoint.XY.y.value.get_pt()) && X_cubic.march(*setpoint.XY.x.value.get_pt()))
        {
            XY_waypt = 6;
            XY_time_ns = rc_nanos_since_boot();
            break;
        }
        else break;
    case 6:
        if (finddt_s(XY_time_ns) > XY_waypt_delay_s)
        {
            st_XY = true;
            circ_finished = true;
            en_circ = false;
            return 0;
        }
        else
        {
            //keep current position:
            setpoint.XY.x.value.set(X_initial);
            setpoint.XY.y.value.set(Y_initial);
            break;
        }
        //---------End of trajectory-------------//
    }
    return 0;
}

void setpoint_guidance_t::init_circ(void)
{
    en_circ = false;
    reset_circ();
    return;
}

void setpoint_guidance_t::reset_circ(void)
{
    turn_radius = settings.turn_radius;
    turn_period = settings.turn_period;
    turn_time_s = settings.turn_time_s;
    turn_dir    = settings.turn_dir_yaw_cw;
    return;
}

void setpoint_guidance_t::start_circ(void) //intended to allow running at each itter.
{
    if (!en_XY && !en_circ && !circ_finished)
    {
        en_XY = true; // allow XY guidance
        en_circ = true; // start circ guidance
        setpoint.XY.x.value.set(state_estimate.get_X()); //zero out the error
        setpoint.XY.y.value.set(state_estimate.get_Y()); //zero out the error
        setpoint.ATT.z.value.set(state_estimate.get_continuous_heading()); //zero out the error

        circ_guide.set(turn_period, turn_radius, turn_time_s, turn_dir, \
            X_initial, Y_initial, Yaw_initial);
        Y_cubic.set(Y_initial, turn_radius + Y_initial, 0,
            0, TWO_PI * turn_radius * turn_radius / turn_period);

        printf("\nStarting circular trajectory algorithm....");
    }//otherwise do nothing

    return;
}

/*
* Use this function to restart takeoff in case of failure. Should be triggered using
* radio button or one-time command signal.
*/
int setpoint_guidance_t::restart_circ(void) //intended as a trigger (do not run at each itter.)
{
    if (en_XY)
    {
        printf("\n Error in restart_circ: XY guidance is already tunning, must finish or stop the current algorithm");
        return -1;
    }
    reset_XY();
    circ_finished = false;
    start_circ();
    return 0;
}

int setpoint_guidance_t::init(void)
{
    if (unlikely(initialized))
    {
        printf("\nERROR in init: already initialized");
        return -1;
    }

    if (unlikely(init_Z() == -1))
    {
        printf("\nERROR in init: failed to intialize Z guidance");
        return -1;
    }

    if (unlikely(init_XY() == -1))
    {
        printf("\nERROR in init: failed to intialize XY guidance");
        return -1;
    }

    if (unlikely(path.init() == -1))
    {
        printf("\nERROR in init: failed to intialize path guidance");
        return -1;
    }

    reset();

    initialized = true;

    return 0;
}


int setpoint_guidance_t::march(void)
{
    if (unlikely(!initialized))
    {
        printf("\nERROR in march: not initialized");
        return -1;
    }

    if (user_input.get_flight_mode() != AUTO_FFFAFA)
    {
        if (en_Z) reset_Z();
        if (en_XY) reset_XY();
        if (path.is_en()) path.stop();
    }

    if (path.is_en())
    {
        if (en_Z) reset_Z();
        if (en_XY) reset_XY();
        if (unlikely(path.update_setpoint_from_waypoint_NH(setpoint, waypoint_state_machine) == -1))
        {
            printf("\nERROR in march: failed to update setpoint from waypoint");
            return -1;
        }

    }

    if (en_Z && setpoint.Z.value.is_en())
    {
        if (unlikely(march_Z() == -1))
        {
            printf("\nERROR in march: failed to march Z guidance");
            return -1;
        }
    }
    
    if (en_XY && setpoint.XY.is_en())
    {
        if (unlikely(march_XY() == -1))
        {
            printf("\nERROR in march: failed to march XY guidance");
            return -1;
        }
    }

    return 0;
}


void setpoint_guidance_t::reset(void)
{
    reset_Z();
    reset_land();
    reset_takeoff();

    reset_XY();
    reset_square();
    reset_circ();
    return;
}