/**
 * @file trajectories_common.c
 */
#include "trajectories_common.h"

double cubicPol(double xi, double xf, double xdoti, double xdotf, float tt_s, double dt)
{
    double dX = (2 * xi - 2 * xf + tt_s * (xdotf + xdoti)) / (pow(tt_s, 3)) * pow(dt, 3) -
                (3 * xi - 3 * xf + tt_s * (xdotf + 2 * xdoti)) / (pow(tt_s, 2)) * pow(dt, 2) +
                xdoti * dt + xi;
    /* Cubic BVP:
    initial position and velocity is 0
    final position is xf and velocity is 0
    tt_s is total time in seconds
    */
    // double dX = (-2*xf/(tt_s*tt_s*tt_s)) * dt*dt*dt + 3*xf/(tt_s*tt_s) * dt*dt;
    return dX;
}

void __circ_traj(double R, double T, double dt)
{
    double omega = 2.0 * M_PI / T; //angular velocity in a turn
    setpoint.dX_CIRC    = R * sin(omega * dt);
    setpoint.dY_CIRC    = R * cos(omega * dt);
    setpoint.dYaw_CIRC  = -omega * dt;
    return;
}

int AUTO_LAND(void)
{
    //-------------Auto-Land---------------//
    // Landing algorithm basic logic is the following:
    // 1. run only if activated externally using "setpoint.en_Z_land=1"
    // 2. check if the drone is already on the ground
    //	2.1 reset the start-up flag & finish landing sequence
    // 3. detect the time of initiation
    // 4. decrease the altitude using a set velocity after some delay
    // should work at any altitude and is entirely velocity dependant -
    // the altitude will be decreased at a constant rate until landing has been detected
    while (setpoint.en_Z_land)
    {
        // check if already flagged as landed
        if (setpoint.st_land)
        {
            setpoint.last_en_land   = 0;                   // reset status flag for the next time the landing is initiated
            setpoint.Z_dot          = 0;      // stop moving the setpoint down
            setpoint.en_Z_land      = 0;  // finish landing algorithm
            break;
        }
        // run only once when landing is initiated
        if (setpoint.last_en_land == 0)
        {
            setpoint.land_time_ns = rc_nanos_since_boot();  // capture time of initialization
            setpoint.last_en_land = 1;  // flag that landing algorithm is active now
        }
        else
        {
            // wait a second, then start decreasing altitude
            // need to wait in case altitude hold was just activated and
            // altitude is not stable yet
            if (finddt_s(setpoint.land_time_ns) > 1)
            {
                setpoint.Z_dot = setpoint.V_max_land;  // start moving the setpoint down
            }
            else
            {
                setpoint.Z_dot = 0.0;  // stabilize altitude
            }
        }
        //-----Landing Detection----//
        if (setpoint.Z < (state_estimate.Z - XYZ_MAX_ERROR))
        {
            // setpoint.st_land = 1; //landed - let the landing algorithm finish on it's own
            printf("\n WARNING: Flying up! \n");
            // break;
        }

        if (setpoint.Z > (state_estimate.Z + XYZ_MAX_ERROR))
        {
            setpoint.st_land = 1;  // landed - let the landing algorithm finish on it's own
            printf("\n Detected Touchdown! \n");
            break;
        }
        else
        {
            setpoint.st_land = 0;  // not landed yet
            setpoint.Z += setpoint.Z_dot * DT;
            break;
        }
        break;
    }
    return setpoint.st_land;
}

int AUTO_TAKEOFF(void)
{
    //-------------Auto-Take-off---------------//
    // Take-off algorithm basic logic is the following:
    // 1. run only if activated externally using "setpoint.en_Z_takeoff=1"
    // 2. check if the drone is has taken off to target alt.
    //	2.1 reset the start-up flag & finish take-off sequence
    // 3. detect the time of initiation
    // 4. move the setpoint.Z
    while (setpoint.en_Z_takeoff)
    {
        // check if flagged as taken-off
        if (setpoint.st_takeoff)
        {
            printf("\nFinished Takeoff\n");
            setpoint.last_en_takeoff    = 0;  // reset status flag for the next time the take-off is initiated
            setpoint.en_Z_takeoff       = 0;  // finish take-off algorithm
            break;
        }
        // run only once when take-off is initiated
        if (setpoint.last_en_takeoff == 0)
        {
            //printf("\nSetting up Takeoff\n");
            setpoint.takeoff_time_ns    = rc_nanos_since_boot();  // capture time of initialization
            setpoint.last_en_takeoff    = 1;                      // flag that take-off is active now
            setpoint.st_takeoff         = 0;                           // reset take-off success flag
            // capture altitude of initialization
            setpoint.Z_init_takeoff     = state_estimate.Z;
            setpoint.Z_throttle_0       = 0.40;
        }
        if (finddt_s(setpoint.takeoff_time_ns) >= setpoint.t_takeoff)
        {
            //printf("\nCompleted takeoff! Time: %f / %f\n", finddt_s(setpoint.takeoff_time_ns), setpoint.t_takeoff);
            setpoint.st_takeoff = 1;  // take-off altitude has been reached - disable take-off and
                                      // let alt-hold do its job
            break;
        }
        else
        {
            //printf("\nTakingoff!\n");
            setpoint.st_takeoff = 0;
            // use cubic function:
            setpoint.Z =
                cubicPol(setpoint.Z_init_takeoff, setpoint.dZ_takeoff + setpoint.Z_init_takeoff, 0,
                    0, setpoint.t_takeoff, finddt_s(setpoint.takeoff_time_ns));
            break;
        }
        break;
    }
    return setpoint.st_takeoff;
}

int XY_SQUARE(double dX, double dY, double t_X, double t_Y)
{
    /* -------------- AUTONOMOUS XY TRACKING-----------//
    This algorithm performs simple square trajectory using cubic polynomials.
    The basic logic is as the following:
    1. Assume AUTONOMOUS has just been activated in air 
    Assuming Hover state, execute the trajectory
        1.1 using initial time and position apply dv
        1.f stabilize to hover (might need to write a separate funtion for this)
    */
    static double temp_X, temp_Y;
    static uint64_t temp_time;
    static int wp, initialized;

    if (setpoint.en_XY_SQUARE)
    {
        // check if flagged as completed
        if (setpoint.st_XY_SQUARE)
        {
            initialized = 0;  // reset status flag for the next time the algorithm is initiated
            setpoint.en_XY_SQUARE   = 0;  // finish algorithm
            return setpoint.st_XY_SQUARE;
        }
        // initialization
        if (initialized == 0)
        {
            temp_time               = rc_nanos_since_boot();              // capture time of initialization
            initialized             = 1;                                     // flag that AUTO is active now
            setpoint.st_XY_SQUARE   = 0;                      // reset success flag
            wp                      = 1;                      // reset waypoint flag

            // capture current position and time (corner 1)
            temp_X = setpoint.X;
            temp_Y = setpoint.Y;
        }

        // begin main sequence -- use waypoints for transition
        switch (wp)
        {
            case 1:
                // move to corner 2:
                if (finddt_s(temp_time) < t_X)
                {
                    setpoint.X = cubicPol(
                        temp_X, dX + temp_X, 0, 0, t_X, finddt_s(temp_time));
                    setpoint.Y_dot = 0;
                    setpoint.st_XY_SQUARE = 0;
                    break;
                }
                else
                {
                    setpoint.X_dot = 0;
                    setpoint.Y_dot = 0;
                    wp = 2;
                    temp_time = rc_nanos_since_boot();
                    setpoint.st_XY_SQUARE = 0;
                    break;
                }
            case 2:
                // move to corner 3:
                if (finddt_s(temp_time) < t_Y)
                {
                    setpoint.X_dot = 0;
                    setpoint.Y = cubicPol(
                        temp_Y, dY + temp_Y, 0, 0, t_Y, finddt_s(temp_time));
                    setpoint.st_XY_SQUARE = 0;
                }
                else
                {
                    setpoint.X_dot = 0;
                    setpoint.Y_dot = 0;
                    wp = 3;
                    temp_time = rc_nanos_since_boot();
                    setpoint.st_XY_SQUARE = 0;
                }
                break;
            case 3:
                // move to corner 4:
                if (finddt_s(temp_time) < t_X)
                {
                    setpoint.X = cubicPol(
                        temp_X + dX, temp_X, 0, 0, t_X, finddt_s(temp_time));
                    setpoint.Y_dot = 0;
                    setpoint.st_XY_SQUARE = 0;
                }
                else
                {
                    setpoint.X_dot = 0;
                    setpoint.Y_dot = 0;
                    setpoint.st_XY_SQUARE = 0;
                    wp = 4;
                    temp_time = rc_nanos_since_boot();
                }
                break;
            case 4:
                // move back to corner 1:
                if (finddt_s(temp_time) < t_Y)
                {
                    setpoint.X_dot = 0;
                    setpoint.Y = cubicPol(
                        temp_Y + dY, temp_Y, 0, 0, t_Y, finddt_s(temp_time));
                    setpoint.st_XY_SQUARE = 0;
                    break;
                }
                else
                {
                    setpoint.X_dot = 0;
                    setpoint.Y_dot = 0;
                    setpoint.st_XY_SQUARE = 1;
                }
                break;
            //---------End of trajectory-------------//
        }
    }
    return setpoint.st_XY_SQUARE;
}

int XY_CIRC(double R, double T)
{
    static double temp_X, temp_Y, temp_yaw;
    static uint64_t temp_time;
    static int initialized;

    while (setpoint.en_XY_CIRC)
    {
        // check if flagged as complete
        if (setpoint.st_XY_CIRC)
        {
            // reset status flag for the next time it is initiated
            initialized = 0;
            // finish the algorithm
            setpoint.en_XY_CIRC = 0;
            break;
        }
        // run only once when initiated
        if (initialized == 0)
        {
            temp_time = rc_nanos_since_boot();                 // capture time of initialization
            initialized = 1;                                   // flag active now
            setpoint.st_XY_CIRC = 0;                           // reset success flag

            // capture state of initialization:
            temp_X = setpoint.X;
            temp_Y = setpoint.Y;
            temp_yaw = setpoint.yaw;
        }
        if (finddt_s(temp_time) >= setpoint.T_CIRC)
        {
            setpoint.st_XY_CIRC = 1;  // assume we are at our target, disable algorithm now
            break;
        }
        else
        {
            setpoint.st_XY_CIRC = 0;
            // update increments:
            __circ_traj(R, T, finddt_s(temp_time));
            // update setpoints:
            setpoint.X = temp_X + setpoint.dX_CIRC;
            setpoint.Y = temp_Y + setpoint.dY_CIRC - setpoint.R_CIRC;
            setpoint.yaw = temp_yaw + setpoint.dYaw_CIRC;

            // printf("\n X=%f, Y=%f and Yaw=%f\n", setpoint.X, setpoint.Y, setpoint.yaw);
            break;
        }
        break;
    }
    return setpoint.st_XY_CIRC;
}

void AUTO_LIFTOFF_HOWER_TEST(void)
{
    /* -------------- AUTONOMOUS XY TRACKING-----------//
    This algorithm performs simple take-off to a set altitude,
    performs simple square trajectory and lands.
    The basic logic is as the following:
    1. Assume AUTONOMOUS has just been activated on the ground
        1.1 initialization
        1.2 perform automated ascent
        1.3 stabilize to hover
        1.4 log final time and position
    2. Stabilize to hover (might need to write a separate funtion for this)
    3. Land
    */
    static int initialized;
    if (setpoint.en_AUTO_LIFTOFF_HOWER_TEST)
    {
        // check if flagged as completed
        if (setpoint.st_AUTO)
        {
            initialized = 0;  // reset status flag for the next time the algorithm is initiated
            setpoint.en_AUTO_LIFTOFF_HOWER_TEST = 0;  // finish algorithm
            return;
        }
        // initialization
        if (initialized == 0)
        {
            setpoint.AUTO_time_ns = rc_nanos_since_boot();  // capture time of initialization
            initialized = 1;                                   // flag that AUTO is active now
            setpoint.st_AUTO    = 0;                           // reset success flag
            setpoint.AUTO_wp    = 0;                           // reset waypoint flag

            // set constants -- move these out later
            // take-off:
            setpoint.dZ_takeoff = -settings.height_takeoff;  // lift-off height above initial point
            setpoint.t_takeoff  = settings.t_takeoff;
            // landing:
            setpoint.V_max_land = settings.V_max_land;  // for testing
            // trajectory:
            setpoint.t_AUTO = 10.0;
        }
        if (finddt_s(setpoint.AUTO_time_ns) > 0.3 && setpoint.AUTO_wp < 1)
        {
            setpoint.AUTO_wp = 1;
        }
        // begin main sequence -- use waypoints for transition
        switch (setpoint.AUTO_wp)
        {
            case 1:
                setpoint.en_Z_takeoff = 1;  // start take-off algorithm
                setpoint.st_takeoff = 0;
                setpoint.AUTO_wp = 2;
                break;

            case 2:
                if (AUTO_TAKEOFF())
                {
                    setpoint.AUTO_wp = 3;
                }
                else
                {
                    break;
                }
                // end lift-off
            //-----------------------//
            case 3:
                setpoint.AUTO_time_ns = rc_nanos_since_boot();  // capture time
                setpoint.AUTO_wp = 4;
            case 4:
                // stabilize
                setpoint.X_dot = 0;
                setpoint.Y_dot = 0;

                if (finddt_s(setpoint.AUTO_time_ns) >= setpoint.t_AUTO)
                {
                    setpoint.AUTO_time_ns = rc_nanos_since_boot();  // capture time
                    setpoint.X_dot = 0;
                    setpoint.Y_dot = 0;
                    setpoint.AUTO_wp = 6;
                }
                break;
            //---------End of trajectory-------------//
            case 6:
                if (finddt_s(setpoint.AUTO_time_ns) > 1)
                {
                    setpoint.en_Z_land = 1;  // start landing algorithm
                    setpoint.AUTO_wp = 7;
                }
                break;

            case 7:
                if (AUTO_LAND())
                {
                    setpoint.AUTO_wp = 12;
                    setpoint.st_AUTO = 1;  // done
                    user_input.requested_arm_mode = DISARMED;
                    // should be on the ground at this point
                    return;
                }
                return;
        }
    }
    return;
}

void AUTO_XY_SQUARE_TEST(void)
{
    /* -------------- AUTONOMOUS XY TRACKING-----------//
    This algorithm performs simple take-off to a set altitude,
    performs simple square trajectory and lands.
    The basic logic is as the following:
    1. Assume AUTONOMOUS has just been activated on the ground
        1.1 initialization
        1.2 perform automated ascent
        1.3 stabilize to hover
        1.4 log final time and position
    2. Assuming Hover state, execute the trajectory
        2.1 using initial time and position apply dv
        2.f stabilize to hover (might need to write a separate funtion for this)
    3. 	Land
    */
    static double t_X;
    static double t_Y;
    static int initialized;

    if (setpoint.en_XY_SQUARE_TEST)
    {
        // check if flagged as completed
        if (setpoint.st_AUTO)
        {
            initialized = 0;  // reset status flag for the next time the algorithm is initiated
            setpoint.en_XY_SQUARE_TEST = 0;  // finish algorithm
            return;
        }
        // initialization
        if (initialized == 0)
        {
            setpoint.AUTO_time_ns = rc_nanos_since_boot();  // capture time of initialization
            initialized = 1;                                // flag that AUTO is active now
            setpoint.st_AUTO = 0;                           // reset success flag
            setpoint.AUTO_wp = 0;                           // reset waypoint flag

            // set constants -- move these out later
            // take-off:
            setpoint.dZ_takeoff = -settings.height_takeoff;  // lift-off height above initial point
            setpoint.t_takeoff  = settings.t_takeoff;
            // landing:
            setpoint.V_max_land = settings.V_max_land;
            // trajectory:
            setpoint.dX_AUTO = 1.5;
            setpoint.dY_AUTO = 3.0;
            setpoint.t_AUTO = 30.0;

            t_X = setpoint.t_AUTO / (2 * (setpoint.dY_AUTO / setpoint.dX_AUTO + 1));
            t_Y = setpoint.t_AUTO / (2 * (setpoint.dX_AUTO / setpoint.dY_AUTO + 1));
        }
        if (finddt_s(setpoint.AUTO_time_ns) > 0.3 && setpoint.AUTO_wp < 1)
        {
            setpoint.AUTO_wp = 1;
        }
        // begin main sequence -- use waypoints for transition
        switch (setpoint.AUTO_wp)
        {
            case 1:
                setpoint.en_Z_takeoff = 1;  // start take-off algorithm
                setpoint.st_takeoff = 0;
                setpoint.AUTO_wp = 2;
                break;

            case 2:
                if (AUTO_TAKEOFF())
                {
                    setpoint.AUTO_wp = 3;
                }
                else
                {
                    break;
                }
                // end lift-off
            //-----------------------//
            case 3:
                setpoint.AUTO_time_ns = rc_nanos_since_boot();  // capture time
                setpoint.AUTO_wp = 4;
            case 4:
                // stabilize
                setpoint.X_dot = 0;
                setpoint.Y_dot = 0;
                if (finddt_s(setpoint.AUTO_time_ns) >= 5)
                {
                    setpoint.AUTO_wp = 5;
                    setpoint.en_XY_SQUARE = 1;
                }
                else
                {
                    break;
                }
            case 5:
                if (XY_SQUARE(setpoint.dX_AUTO, setpoint.dY_AUTO, t_X, t_Y))
                {
                    setpoint.X_dot = 0;
                    setpoint.Y_dot = 0;
                    setpoint.AUTO_wp = 9;
                    setpoint.AUTO_time_ns = rc_nanos_since_boot();
                }
                break;
            //---------End of trajectory-------------//
            case 9:
                if (finddt_s(setpoint.AUTO_time_ns) > 1)
                {
                    setpoint.en_Z_land = 1;  // start landing algorithm
                    setpoint.AUTO_wp = 10;
                }
                break;

            case 10:
                if (AUTO_LAND())
                {
                    setpoint.AUTO_wp = 12;
                    setpoint.st_AUTO = 1;  // done
                    user_input.requested_arm_mode = DISARMED;
                    // should be on the ground at this point
                    return;
                }
                return;
        }
    }
    return;
}

void AUTO_XY_CIRC_TEST(void)
{
    /* -------------- AUTONOMOUS XY TRACKING-----------//
    This algorithm performs simple take-off to a set altitude,
    performs simple square trajectory and lands.
    The basic logic is as the following:
    1. Assume AUTONOMOUS has just been activated on the ground
        1.1 initialization
        1.2 perform automated ascent
        1.3 stabilize to hover
        1.4 log final time and position
    2. Do circular trajectory
    3. Stabilize to hover
    4. Land
    */
    static int initialized;
    if (setpoint.en_XY_CIRC_TEST)
    {
        // check if flagged as completed
        if (setpoint.st_AUTO)
        {
            initialized = 0;  // reset status flag for the next time the algorithm is initiated
            setpoint.en_XY_CIRC_TEST = 0;  // finish algorithm
            return;
        }
        // initialization
        if (initialized == 0)
        {
            setpoint.AUTO_time_ns = rc_nanos_since_boot();  // capture time of initialization
            initialized = 1;                                // flag that AUTO is active now
            setpoint.st_AUTO = 0;                           // reset success flag
            setpoint.AUTO_wp = 0;                           // reset waypoint flag

            // set constants -- move these out later
            // take-off:
            setpoint.dZ_takeoff = -settings.height_takeoff;  // lift-off height above initial point
            setpoint.t_takeoff = settings.t_takeoff;
            // landing:
            setpoint.V_max_land = settings.V_max_land;  // for testing
            // trajectory:
            setpoint.dX_AUTO = 0.0;
            setpoint.dY_AUTO = 0.0;
            setpoint.t_AUTO = 3.0;

            // circular:
            setpoint.R_CIRC = 1.0;
            setpoint.T_CIRC = 20.0;
        }
        if (finddt_s(setpoint.AUTO_time_ns) > 0.3 && setpoint.AUTO_wp < 1)
        {
            setpoint.AUTO_wp = 1;
        }
        // begin main sequence -- use waypoints for transition
        switch (setpoint.AUTO_wp)
        {
            case 1:
                setpoint.en_Z_takeoff = 1;  // start take-off algorithm
                setpoint.st_takeoff = 0;
                setpoint.AUTO_wp = 2;
                break;

            case 2:
                if (AUTO_TAKEOFF())
                {
                    setpoint.AUTO_wp = 3;
                }
                else
                {
                    break;
                }
                // end lift-off
            //-----------------------//
            case 3:
                setpoint.AUTO_time_ns = rc_nanos_since_boot();  // capture time
                setpoint.AUTO_wp = 4;
            case 4:
                // stabilize
                setpoint.X_dot = 0;
                setpoint.Y_dot = 0;

                if (finddt_s(setpoint.AUTO_time_ns) >= setpoint.t_AUTO)
                {
                    setpoint.AUTO_time_ns = rc_nanos_since_boot();  // capture time
                    setpoint.X_dot = 0;
                    setpoint.Y_dot = 0;
                    setpoint.AUTO_wp = 5;
                    setpoint.en_XY_CIRC = 1;
                }
                break;
            case 5:
                if (XY_CIRC(setpoint.R_CIRC,setpoint.T_CIRC))
                {
                    setpoint.en_XY_CIRC = 0;
                    setpoint.AUTO_time_ns = rc_nanos_since_boot();  // capture time
                    setpoint.X_dot = 0;
                    setpoint.Y_dot = 0;
                    setpoint.AUTO_wp = 6;
                }
                break;
            //---------End of trajectory-------------//
            case 6:
                if (finddt_s(setpoint.AUTO_time_ns) > 1)
                {
                    setpoint.en_Z_land = 1;  // start landing algorithm
                    setpoint.AUTO_wp = 7;
                }
                break;

            case 7:
                if (AUTO_LAND())
                {
                    setpoint.AUTO_wp = 12;
                    setpoint.st_AUTO = 1;  // done
                    user_input.requested_arm_mode = DISARMED;
                    // should be on the ground at this point
                    return;
                }
                return;
        }
    }
    return;
}