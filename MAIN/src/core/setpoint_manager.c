/**
* @file setpoint_manager.c
*
*
**/
#include "setpoint_manager.h"


setpoint_t setpoint; // extern variable in setpoint_manager.h

/**
 * Function only used locally
 */
/***********************************/

/**
 * @brief   Logic for starting to follow path, reset time and waypoint counter
 */
static void __start_waypoint_counter();
static void __stop_waypoint_counter();
static void __reset_waypoint_counter();

/***********************************/

void __update_yaw(void)
{
	// if throttle stick is down all the way, probably landed, so
	// keep the yaw setpoint at current yaw so it takes off straight
	if(user_input.thr_stick < 0.1){
		setpoint.yaw = state_estimate.continuous_yaw;
		setpoint.yaw_dot = 0.0;
		return;
	}
	// otherwise, scale yaw_rate by max yaw rate in rad/s
	// and move yaw setpoint
	setpoint.yaw_dot = user_input.yaw_stick * MAX_YAW_RATE;
	setpoint.yaw += setpoint.yaw_dot*DT;
	return;
}

void __update_Z(void)
{
	//----Manual/Radio/Direct control----//
	// always run this - need to have direct pilot control just in case
	// make sure setpoint doesn't go too far below current altitude since we
	// can't sink into the ground
	//-----Safety ----//
	
	//printf("\n Throttle value is %f, zero throttle is %f and setpoint.z is %f \n",user_input.thr_stick,setpoint.Z_throttle_0,setpoint.Z_dot);
	/* Controller saturation - need to move out as a separate function and always run
	if(setpoint.Z > (state_estimate.Z + XYZ_MAX_ERROR)){
		setpoint.Z = state_estimate.Z + XYZ_MAX_ERROR;
		setpoint.Z_dot = 0.0;
		return;
	}
	else if(setpoint.Z < (state_estimate.Z - XYZ_MAX_ERROR)){
		setpoint.Z = state_estimate.Z - XYZ_MAX_ERROR;
		setpoint.Z_dot = 0.0;
		return;
		}
	else{
		setpoint.Z += setpoint.Z_dot*DT;
		setpoint.Z_dot = 0;
	}
	*/
	
	if	(user_input.thr_stick > setpoint.Z_throttle_0+0.1){   	
		setpoint.Z_dot = (user_input.thr_stick - setpoint.Z_throttle_0) * settings.max_Z_velocity;
	}
	else if (user_input.thr_stick < setpoint.Z_throttle_0-0.1){
		setpoint.Z_dot = (user_input.thr_stick - setpoint.Z_throttle_0) * settings.max_Z_velocity;
	}
	else{
		setpoint.Z_dot = 0;
	}
	setpoint.Z -= setpoint.Z_dot*DT; //neagtive since Z positive is defined to be down
	
	return;
}

void __update_XY_pos(void)
{
	// X in the body frame (forward flight)
	// make sure setpoint doesn't go too far from state in case touching something
	if(setpoint.X > (state_estimate.X + XYZ_MAX_ERROR)){
		setpoint.X = state_estimate.X + XYZ_MAX_ERROR;
		setpoint.X_dot = 0.0;
	}
	else if(setpoint.X < (state_estimate.X - XYZ_MAX_ERROR)){
		setpoint.X = state_estimate.X - XYZ_MAX_ERROR;
		setpoint.X_dot = 0.0;
		return;
	}
	else{
		//apply velocity command 
		setpoint.X += setpoint.X_dot*DT;
	}
	
	// Y in the body frame (lateral translation)
	// make sure setpoint doesn't go too far from state in case touching something
	
	if(setpoint.Y > (state_estimate.Y + XYZ_MAX_ERROR)){
		setpoint.Y = state_estimate.Y + XYZ_MAX_ERROR;
		setpoint.Y_dot = 0.0;
		return;
	}
	else if(setpoint.Y < (state_estimate.Y - XYZ_MAX_ERROR)){
		setpoint.Y = state_estimate.Y - XYZ_MAX_ERROR;
		setpoint.Y_dot = 0.0;
		return;
	}
	else{
		//apply velocity command 
		setpoint.Y += setpoint.Y_dot*DT; //Y is defined positive to the left
	}
	

	return;
}




int setpoint_manager_init(void)
{
	if(setpoint.initialized){
		fprintf(stderr, "ERROR in setpoint_manager_init, already initialized\n");
		return -1;
	}
	memset(&setpoint,0,sizeof(setpoint_t));
	setpoint.initialized = 1;
	return 0;
}

int set_new_path(const char* file_name)
{
    if (path_load_from_file(file_name) == -1)
    {
        fprintf(stderr, "ERROR: could not load new path file\n");
        return -1;
    }

    __reset_waypoint_counter();
    return -1;
}

static void __start_waypoint_counter()
{
    // If this is the first time in autonomous mode and armed, save the current time
    if (setpoint.auto_armed_set == 0)
    {
        // If the system is armed and autonomous mode is set, record time in
        // time_auto_set
        setpoint.auto_armed_set = 1;
        setpoint.time_auto_set = rc_nanos_since_boot();

		path.waypoints_init.x		= setpoint.X;
        path.waypoints_init.y		= setpoint.Y;
        path.waypoints_init.z		= setpoint.Z;
        path.waypoints_init.roll	= setpoint.roll;
        path.waypoints_init.pitch	= setpoint.pitch;
        if (fabs(setpoint.yaw - state_estimate.continuous_yaw) >= M_PI/18.0)
        {
            setpoint.yaw = state_estimate.continuous_yaw;
            printf("\nWARNING: High yaw error, overwriting setpoint");
		}
		
        path.waypoints_init.yaw = setpoint.yaw;
    }

    setpoint.waypoint_time = finddt_s(setpoint.time_auto_set);
}

static void __stop_waypoint_counter()
{
    // If the system is disarmed or out of auto reset the auto_armed_set flag
    // and change the current waytpoint to zero
    setpoint.auto_armed_set = 0;
    setpoint.cur_waypoint_num = 0;
}

static void __reset_waypoint_counter()
{
    __stop_waypoint_counter();
    __start_waypoint_counter();
}

void setpoint_update_setpoint_from_waypoint()
{
    __start_waypoint_counter();
    // Break out of function if the current waypoint is the last point in the path
    //printf("\n setpoint.cur_waypoint_num = %" PRId64 "", setpoint.cur_waypoint_num);
    if (setpoint.cur_waypoint_num == path.len)
    {
        return;
    }

    // Parse waypoint flag
    //printf("\n path.waypoints[setpoint.cur_waypoint_num].flag = %d",
    //    path.waypoints[setpoint.cur_waypoint_num].flag);
    switch ((int)path.waypoints[setpoint.cur_waypoint_num].flag)
    {
        case TIME_TRANSITION_FLAG:
            // Check if there are additional waypoints and advnace control
            // to the next waytoint if it is time to do so.  If there are no additional waypoints,
            // keep controlling to the previous point
            if (setpoint.cur_waypoint_num < (path.len - 1) &&
                setpoint.waypoint_time >= path.waypoints[setpoint.cur_waypoint_num + 1].t)
            {
                ++setpoint.cur_waypoint_num;

				// Set the desired x, y, and z if allowed
                if (setpoint.en_waypoint_update)
                {
					setpoint.X		= path.waypoints_init.x		+ path.waypoints[setpoint.cur_waypoint_num].x;
					setpoint.Y		= path.waypoints_init.y		+ path.waypoints[setpoint.cur_waypoint_num].y;
					setpoint.Z		= path.waypoints_init.z		+ path.waypoints[setpoint.cur_waypoint_num].z;
					setpoint.roll	= path.waypoints_init.roll	+ path.waypoints[setpoint.cur_waypoint_num].roll;
					setpoint.pitch	= path.waypoints_init.pitch + path.waypoints[setpoint.cur_waypoint_num].pitch;
					setpoint.yaw	= path.waypoints_init.yaw	+ path.waypoints[setpoint.cur_waypoint_num].yaw;
				}
                
            }
            if (setpoint.cur_waypoint_num >= (path.len - 1))
            {
                //printf("\nDisable wp");
                setpoint.en_waypoint_update = 0;
			}
            break;
        case POS_TRANSITION_FLAG:
            // TODO: determine position error and compare to convergence tolerance
            //       (? who sets/determines/stores convergence tolerance ?)
            assert(0);
            break;
        default:
            fprintf(stderr, "ERROR: unrecognized waypoint flag\n");
    }
    //printf("\n setpoint.waypoint_time = %f time = %f",
    //    setpoint.waypoint_time, path.waypoints[setpoint.cur_waypoint_num + 1].t);
}


int setpoint_manager_update(void)
{
	if(setpoint.initialized==0){
		fprintf(stderr, "ERROR in setpoint_manager_update, not initialized yet\n");
		return -1;
	}

	if(user_input.initialized==0){
		fprintf(stderr, "ERROR in setpoint_manager_update, input_manager not initialized yet\n");
		return -1;
	}

	// if PAUSED or UNINITIALIZED, do nothing
	if(rc_get_state()!=RUNNING) return 0;

	// shutdown feedback on kill switch
	if(user_input.requested_arm_mode == DISARMED){
		if(fstate.arm_state!=DISARMED) feedback_disarm();
		return 0;
	}

	// finally, switch between flight modes and adjust setpoint properly
	switch(user_input.flight_mode){


	case TEST_BENCH_4DOF:
		// configure which controllers are enabled
		setpoint.en_6dof		= 0;
		setpoint.en_rpy_ctrl	= 0;
		setpoint.en_Z_ctrl		= 0;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.roll_throttle	=  user_input.roll_stick;
		setpoint.pitch_throttle	=  user_input.pitch_stick;
		setpoint.yaw_throttle	=  user_input.yaw_stick;
		setpoint.Z_throttle		= -user_input.thr_stick;
		// TODO add these two throttle modes as options to settings, I use a radio
		// with self-centering throttle so having 0 in the middle is safest
		// setpoint.Z_throttle = -(user_input.thr_stick+1.0)/2.0;
		break;

	case TEST_BENCH_6DOF:
		setpoint.en_6dof		= 1;
		setpoint.en_rpy_ctrl	= 0;
		setpoint.en_Z_ctrl		= 0;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.X_throttle		= -user_input.pitch_stick;
		setpoint.Y_throttle		=  user_input.roll_stick;
		setpoint.roll_throttle	=  0.0;
		setpoint.pitch_throttle	=  0.0;
		setpoint.yaw_throttle	=  user_input.yaw_stick;
		setpoint.Z_throttle		= -user_input.thr_stick;
		break;

	case DIRECT_THROTTLE_4DOF:
		setpoint.en_6dof		= 0;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl		= 0;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.roll			=  user_input.roll_stick;
		setpoint.pitch			=  user_input.pitch_stick;
		setpoint.Z_throttle		= -user_input.thr_stick;
		__update_yaw();
		break;

	case DIRECT_THROTTLE_6DOF:
		setpoint.en_6dof		= 1;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl		= 0;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.X_throttle		= -user_input.pitch_stick;
		setpoint.Y_throttle		=  user_input.roll_stick;
		setpoint.Z_throttle		= -user_input.thr_stick;
		__update_yaw();
		break;

	case AUTONOMOUS:
		setpoint.en_6dof		= 0;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl		= 1;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 1;
		
		//Test functions:
		setpoint.en_AUTO_LIFTOFF_HOWER_TEST = 0;
        setpoint.en_XY_CIRC_TEST			= 0;
        setpoint.en_XY_SQUARE_TEST			= 0;

		if (setpoint.en_AUTO_LIFTOFF_HOWER_TEST) AUTO_LIFTOFF_HOWER_TEST();	//run automated position guide (only liftoff, hover and landing)
        if (setpoint.en_XY_CIRC_TEST) AUTO_XY_CIRC_TEST();  // run automated position guide (circular)
        if (setpoint.en_XY_SQUARE_TEST) AUTO_XY_SQUARE_TEST();  // run automated position guide (square)
        
        setpoint_update_setpoint_from_waypoint();
        //printf("\nsp.x = %f and st.x = %f, sp.y = %f and st.y = %f, sp.z = %f, st.z = %f sp.yaw=%f and st.yaw = %f\n",
        //    setpoint.X, state_estimate.X, setpoint.Y, state_estimate.Y, setpoint.Z,
        //    state_estimate.Z, setpoint.yaw, state_estimate.yaw);

		break;	
	case ALT_HOLD_4DOF:
		setpoint.en_6dof		= 0;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl		= 1;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.roll			= user_input.roll_stick;
		setpoint.pitch			= user_input.pitch_stick;
		__update_Z();
		__update_yaw();
		break;

	case ALT_HOLD_6DOF:
		setpoint.en_6dof		= 1;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl		= 1;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 0;

		setpoint.roll			= 0.0;
		setpoint.pitch			= 0.0;
		setpoint.X_throttle		= -user_input.pitch_stick;
		setpoint.Y_throttle		=  user_input.roll_stick;
		__update_Z();
		__update_yaw();
		break;

	case VELOCITY_CONTROL_4DOF:
		setpoint.en_6dof		= 0;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl		= 1;
		setpoint.en_XY_vel_ctrl	= 1;
		setpoint.en_XY_pos_ctrl	= 0;
		
		setpoint.X_dot 			= -user_input.pitch_stick * settings.max_XY_velocity;
		setpoint.Y_dot	 		=  user_input.roll_stick  * settings.max_XY_velocity;
		
		//check validity of the velocity command, construct virtual setpoint
		__update_Z();
		__update_yaw();
		break;

	case VELOCITY_CONTROL_6DOF:
		setpoint.en_6dof		= 1;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl		= 1;
		setpoint.en_XY_vel_ctrl	= 1;
		setpoint.en_XY_pos_ctrl	= 0;
		
		setpoint.X_dot 			= -user_input.pitch_stick * settings.max_XY_velocity;
		setpoint.Y_dot 			=  user_input.roll_stick  * settings.max_XY_velocity;
		__update_Z();
		__update_yaw();
		break;

	case POSITION_CONTROL_4DOF:
		setpoint.en_6dof		= 0;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl		= 1;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 1;

		setpoint.X_dot = -user_input.pitch_stick * settings.max_XY_velocity;
        setpoint.Y_dot = user_input.roll_stick * settings.max_XY_velocity;
		
		//check validity of the velocity command, construct virtual setpoint
		__update_XY_pos();
		__update_Z();
		__update_yaw();
		
		//printf("\nM2  setpoint.X=%f setpoint.Y=%f setpoint.Z=%f with altitude of %f \n",setpoint.X,setpoint.Y,setpoint.Z,state_estimate.Z);
		break;

	case POSITION_CONTROL_6DOF:
		setpoint.en_6dof		= 1;
		setpoint.en_rpy_ctrl	= 1;
		setpoint.en_Z_ctrl		= 1;
		setpoint.en_XY_vel_ctrl	= 0;
		setpoint.en_XY_pos_ctrl	= 1;

		setpoint.X_dot 			= -user_input.pitch_stick * settings.max_XY_velocity;
		setpoint.Y_dot 			=  user_input.roll_stick  * settings.max_XY_velocity;
		__update_XY_pos();
		__update_Z();
		__update_yaw();
		break;

	default: // should never get here
		fprintf(stderr,"ERROR in setpoint_manager thread, unknown flight mode\n");
		break;

	} // end switch(user_input.flight_mode)

	// arm feedback when requested
	if(user_input.requested_arm_mode == ARMED){
		if(fstate.arm_state==DISARMED) feedback_arm();
	}


	return 0;
}


int setpoint_manager_cleanup(void)
{
	setpoint.initialized=0;
	return 0;
}
