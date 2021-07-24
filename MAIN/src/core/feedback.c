/**
 * @file feedback.c
 *
 */

#include <feedback.h>


#define TWO_PI (M_PI*2.0)

feedback_state_t fstate; // extern variable in feedback.h

// keep original controller gains for scaling later
static double D_roll_gain_orig, D_pitch_gain_orig, D_yaw_gain_orig, D_Z_gain_orig;

// filters
static rc_filter_t D_roll	= RC_FILTER_INITIALIZER;
static rc_filter_t D_pitch	= RC_FILTER_INITIALIZER;
static rc_filter_t D_yaw	= RC_FILTER_INITIALIZER;
static rc_filter_t D_Z		= RC_FILTER_INITIALIZER;
static rc_filter_t D_Xdot_4	= RC_FILTER_INITIALIZER;
static rc_filter_t D_Xdot_6	= RC_FILTER_INITIALIZER;
static rc_filter_t D_X_4	= RC_FILTER_INITIALIZER;
static rc_filter_t D_X_6	= RC_FILTER_INITIALIZER;
static rc_filter_t D_Ydot_4	= RC_FILTER_INITIALIZER;
static rc_filter_t D_Ydot_6	= RC_FILTER_INITIALIZER;
static rc_filter_t D_Y_4	= RC_FILTER_INITIALIZER;
static rc_filter_t D_Y_6	= RC_FILTER_INITIALIZER;


static int __send_motor_stop_pulse(void)
{
	int i;
	if(settings.num_rotors>8){
		printf("ERROR: set_motors_to_idle: too many rotors\n");
		return -1;
	}
	for(i=0;i<settings.num_rotors;i++){
		fstate.m[i] = -0.1;
		rc_servo_send_esc_pulse_normalized(i+1,-0.1);
	}
	return 0;
}

static void __rpy_init(void)
{
	// get controllers from settings

	rc_filter_duplicate(&D_roll,	settings.roll_controller);
	rc_filter_duplicate(&D_pitch,	settings.pitch_controller);
	rc_filter_duplicate(&D_yaw,	    settings.yaw_controller);

	#ifdef DEBUG
	printf("ROLL CONTROLLER:\n");
	rc_filter_print(D_roll);
	printf("PITCH CONTROLLER:\n");
	rc_filter_print(D_pitch);
	printf("YAW CONTROLLER:\n");
	rc_filter_print(D_yaw);
	#endif

	// save original gains as we will scale these by battery voltage later
	D_roll_gain_orig = D_roll.gain;
	D_pitch_gain_orig = D_pitch.gain;
	D_yaw_gain_orig = D_yaw.gain;
	
	//save original TF numerator and denominator
	

	// enable saturation. these limits will be changed late but we need to
	// enable now so that soft start can also be enabled
	rc_filter_enable_saturation(&D_roll,	-MAX_ROLL_COMPONENT, MAX_ROLL_COMPONENT);
	rc_filter_enable_saturation(&D_pitch,	-MAX_PITCH_COMPONENT, MAX_PITCH_COMPONENT);
	rc_filter_enable_saturation(&D_yaw,	-MAX_YAW_COMPONENT, MAX_YAW_COMPONENT);
	// enable soft start
	rc_filter_enable_soft_start(&D_roll, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_pitch, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_yaw, SOFT_START_SECONDS);
}


int feedback_disarm(void)
{
	fstate.arm_state = DISARMED;
	// set LEDs
	rc_led_set(RC_LED_RED,1);
	rc_led_set(RC_LED_GREEN,0);
	return 0;
}

int feedback_arm(void)
{
	//printf("\n Arming!\n");
	if(fstate.arm_state==ARMED){
		printf("WARNING: trying to arm when controller is already armed\n");
		return -1;
	}
	// start a new log file every time controller is armed, this may take some
	// time so do it before touching anything else
	if(settings.enable_logging) log_manager_init();
	// get the current time
	fstate.arm_time_ns = rc_nanos_since_boot();
	// reset the index
	fstate.loop_index = 0;
	// when swapping from direct throttle to altitude control, the altitude
	// controller needs to know the last throttle input for smooth transition
	// TODO: Reinitialize altitude bias
	//static int last_en_alt_ctrl = 0; //make sure altitude control will go through initialization
	//last_usr_thr = MIN_Z_COMPONENT;
	// yaw estimator can be zero'd too
	// TODO: Reinitialize yaw estimate
	//num_yaw_spins = 0;
	//last_yaw = -mpu_data.fused_TaitBryan[TB_YAW_Z]; // minus because NED coordinates
	// zero out all filters
	rc_filter_reset(&D_roll);
	rc_filter_reset(&D_pitch);
	rc_filter_reset(&D_yaw);
	rc_filter_reset(&D_Z);
	
	rc_filter_reset(&D_Xdot_4);
	rc_filter_reset(&D_Xdot_6);
	rc_filter_reset(&D_X_4);
	rc_filter_reset(&D_X_6);
	rc_filter_reset(&D_Ydot_4);
	rc_filter_reset(&D_Ydot_6);
	rc_filter_reset(&D_Y_4);
	rc_filter_reset(&D_Y_6);

	// prefill filters with current error
	rc_filter_prefill_inputs(&D_roll, -state_estimate.roll);
	rc_filter_prefill_inputs(&D_pitch, -state_estimate.pitch);
	// set LEDs
	rc_led_set(RC_LED_RED,0);
	rc_led_set(RC_LED_GREEN,1);
	// last thing is to flag as armed
	fstate.arm_state = ARMED;
	return 0;
}



int feedback_init(void)
{

	__rpy_init();

	rc_filter_duplicate(&D_Z,		settings.altitude_controller);
	rc_filter_duplicate(&D_Xdot_4,	settings.horiz_vel_ctrl_4dof);
	rc_filter_duplicate(&D_Xdot_6,	settings.horiz_vel_ctrl_6dof);
	rc_filter_duplicate(&D_X_4,		settings.horiz_Xpos_ctrl_4dof);
	rc_filter_duplicate(&D_X_6,		settings.horiz_pos_ctrl_6dof);
	
	rc_filter_duplicate(&D_Ydot_4,	settings.horiz_vel_ctrl_4dof);
	rc_filter_duplicate(&D_Ydot_6,	settings.horiz_vel_ctrl_6dof);
	rc_filter_duplicate(&D_Y_4,		settings.horiz_Ypos_ctrl_4dof);
	rc_filter_duplicate(&D_Y_6,		settings.horiz_pos_ctrl_6dof);


	#ifdef DEBUG
	printf("ALTITUDE CONTROLLER:\n");
	rc_filter_print(D_Z);
	#endif

	D_Z_gain_orig = D_Z.gain;

	rc_filter_enable_saturation(&D_Z, -1.0, 1.0);
	rc_filter_enable_soft_start(&D_Z, SOFT_START_SECONDS);
	rc_filter_enable_saturation(&D_X_4, -1.0, 1.0);
	rc_filter_enable_soft_start(&D_X_4, SOFT_START_SECONDS);
	rc_filter_enable_saturation(&D_Y_4, -1.0, 1.0);
	rc_filter_enable_soft_start(&D_Y_4, SOFT_START_SECONDS);

	// make sure everything is disarmed them start the ISR
	feedback_disarm();

	fstate.initialized=1;

	return 0;
}

int feedback_march(void)
{
	int i;
	double tmp, min, max;
	double u[6], mot[8];
	
	/*Update gains using xbee. Use xbeeMsg.GainCH for swithing
	between channels, assume 0 is the default mode of operation
	with the original gains.
	*/
	
	if(xbeeMsg.GainCH == 1){
		D_roll.num.d[0] = xbeeMsg.GainN0;
		D_roll.num.d[1] = xbeeMsg.GainN1;
		D_roll.num.d[2] = xbeeMsg.GainN2;
		D_roll.den.d[0] = xbeeMsg.GainD0;
		D_roll.den.d[1] = xbeeMsg.GainD1;
		D_roll.den.d[2] = xbeeMsg.GainD2;
		
		}
	else if(xbeeMsg.GainCH == 2){
		D_pitch.num.d[0] = xbeeMsg.GainN0;
		D_pitch.num.d[1] = xbeeMsg.GainN1;
		D_pitch.num.d[2] = xbeeMsg.GainN2;
		D_pitch.den.d[0] = xbeeMsg.GainD0;
		D_pitch.den.d[1] = xbeeMsg.GainD1;
		D_pitch.den.d[2] = xbeeMsg.GainD2;
		
		}
	else if(xbeeMsg.GainCH == 3){
		D_yaw.num.d[0] = xbeeMsg.GainN0;
		D_yaw.num.d[1] = xbeeMsg.GainN1;
		D_yaw.num.d[2] = xbeeMsg.GainN2;
		D_yaw.den.d[0] = xbeeMsg.GainD0;
		D_yaw.den.d[1] = xbeeMsg.GainD1;
		D_yaw.den.d[2] = xbeeMsg.GainD2;
		
		}
	else if(xbeeMsg.GainCH == 4){
		D_Z.num.d[0] = xbeeMsg.GainN0;
		D_Z.num.d[1] = xbeeMsg.GainN1;
		D_Z.num.d[2] = xbeeMsg.GainN2;
		D_Z.den.d[0] = xbeeMsg.GainD0;
		D_Z.den.d[1] = xbeeMsg.GainD1;
		D_Z.den.d[2] = xbeeMsg.GainD2;
		
	}
	else if(xbeeMsg.GainCH == 5){
		D_X_4.num.d[0] = xbeeMsg.GainN0;
		D_X_4.num.d[1] = xbeeMsg.GainN1;
		D_X_4.num.d[2] = xbeeMsg.GainN2;
		D_X_4.den.d[0] = xbeeMsg.GainD0;
		D_X_4.den.d[1] = xbeeMsg.GainD1;
		D_X_4.den.d[2] = xbeeMsg.GainD2;
		
	}
	else if(xbeeMsg.GainCH == 6){
		D_Y_4.num.d[0] = xbeeMsg.GainN0;
		D_Y_4.num.d[1] = xbeeMsg.GainN1;
		D_Y_4.num.d[2] = xbeeMsg.GainN2;
		D_Y_4.den.d[0] = xbeeMsg.GainD0;
		D_Y_4.den.d[1] = xbeeMsg.GainD1;
		D_Y_4.den.d[2] = xbeeMsg.GainD2;
		
	}
	else{
		//use original pid gains defined in the settings file:	
	}
	

	// it  won't redfune here the second time the feedback_march is called
	static int last_en_Z_ctrl = 0;
	static int last_en_XYZ_ctrl =0;
	//last_en_Z_ctrl = 0;

	// Disarm if rc_state is somehow paused without disarming the controller.
	// This shouldn't happen if other threads are working properly.
	if(rc_get_state()!=RUNNING && fstate.arm_state==ARMED){
		feedback_disarm();
		printf("\n rc_state is somehow paused \n");
	}

	// check for a tipover
	if(fabs(state_estimate.roll)>TIP_ANGLE || fabs(state_estimate.pitch)>TIP_ANGLE){
		feedback_disarm();
		printf("\n TIPOVER DETECTED \n");
	}

	//For ESC wakeup:
	static uint64_t time_boot_wakeup;
	static int wakeup_flag = 0;
	if (wakeup_flag != 1) time_boot_wakeup = rc_nanos_since_boot();

	// if not running or not armed, keep the motors in an idle state
	if (rc_get_state() != RUNNING || fstate.arm_state == DISARMED) {
		if (wakeup_flag != 1)
		{
			printf("\n WARNING: Waking up the ESCs....");
			while (finddt_s(time_boot_wakeup) < 3.0)
			{
				__send_motor_stop_pulse();
			}
			wakeup_flag = 1;
			printf("\n Done! Wake up time was: %f ", finddt_s(time_boot_wakeup));
		}
		else
		{
			__send_motor_stop_pulse();
		}
		return 0;
	}
	
	/* Make sure Altitude controller flag is set to off if was not used or if
	switching between flight modes in flight: Has to make last_en_Z_ctrl = 0
	if not using altitude control - we want to make sure controller is being
	reset every time flight mode is switched to altitude control.
	*/
	if(setpoint.en_Z_ctrl==0) last_en_Z_ctrl = 0;
	if(setpoint.en_XY_pos_ctrl==0) last_en_XYZ_ctrl = 0;
	// We are about to start marching the individual SISO controllers forward.
	// Start by zeroing out the motors signals then add from there.
	for(i=0;i<8;i++) mot[i] = 0.0;
		for(i=0;i<6;i++) u[i] = 0.0;
	/***************************************************************************
	* Position Controller
	***************************************************************************/
	// this needs work...
			if(setpoint.en_XY_pos_ctrl){
				if(last_en_XYZ_ctrl == 0)
				{
					setpoint.X = state_estimate.X;
					setpoint.Y = state_estimate.Y;

					rc_filter_reset(&D_X_4);   //B reset the filter and reads from json
					rc_filter_reset(&D_Y_4);
						
					rc_filter_prefill_outputs(&D_X_4, 0);
					rc_filter_prefill_outputs(&D_Y_4, 0);
					last_en_XYZ_ctrl = 1;
				}
				
				////////////// PID for horizontal positon control

                double X_error = setpoint.X - state_estimate.X;
                double Y_error = setpoint.Y - state_estimate.Y;
                double Z_error = setpoint.Z - state_estimate.Z;

				double state_estimate_roll	= state_estimate.roll;
                double state_estimate_pitch = state_estimate.pitch;
                double state_estimate_yaw	= state_estimate.continuous_yaw;

                double x_error = X_error * cos(state_estimate_yaw) * cos(state_estimate_pitch) -
                                 Z_error * sin(state_estimate_pitch) +
                                 Y_error * cos(state_estimate_pitch) * sin(state_estimate_yaw);

                double y_error = Y_error * (cos(state_estimate_yaw) * cos(state_estimate_roll) +
                                               sin(state_estimate_yaw) * sin(state_estimate_roll) *
                                                   sin(state_estimate_pitch)) -
                                 X_error * (cos(state_estimate_roll) * sin(state_estimate_yaw) -
                                               cos(state_estimate_yaw) * sin(state_estimate_roll) *
                                                   sin(state_estimate_pitch)) +
                                 Z_error * cos(state_estimate_pitch) * sin(state_estimate_roll);

				
				setpoint.pitch = rc_filter_march(&D_X_4, -x_error);
                setpoint.roll = rc_filter_march(&D_Y_4, y_error);
				
				last_en_XYZ_ctrl = 1;
			}
	/***************************************************************************
	* Throttle/Altitude Controller
	*
	* If transitioning from direct throttle to altitude control, prefill the
	* filter with current throttle input to make smooth transition. This is also
	* true if taking off for the first time in altitude mode as arm_controller
	* sets up last_en_Z_ctrl and last_usr_thr every time controller arms
	***************************************************************************/
	// run altitude controller if enabled
			if(setpoint.en_Z_ctrl)
			{
				//B only first cycle (the first time step after altitude controll is on)
				if(last_en_Z_ctrl == 0)
				{
					//take stick position as nominal hover thrust but leave enough room for manual adjustements for extreme cases
					if (settings.enable_xbee){
						if (user_input.thr_stick > 0.80){
							setpoint.Z_throttle_0 = 0.80; //don't let hover thrust be too high (but account for heavy drones with T/W < 1.7)
						}
						else if (user_input.thr_stick < 0.15){
							setpoint.Z_throttle_0 = 0.15; //don't let hover thrust be too low if starting from the ground
						}
						else {
							setpoint.Z_throttle_0 = user_input.thr_stick; //detect last user input
						}
					}
					setpoint.Z = state_estimate.Z; // set altitude setpoint to current altitude
					
					rc_filter_reset(&D_Z);   // reset the filter (works well so far, not need to prefill outputs)

					last_en_Z_ctrl = 1;
				}

				D_Z.gain = D_Z_gain_orig * settings.v_nominal/state_estimate.v_batt_lp;	// updating the gains based on battery voltage

				// This needs to be tested! currently altitude control is done in the inertial frame, but needs to be in the body 
				tmp = -setpoint.Z_throttle_0 + rc_filter_march(&D_Z, setpoint.Z - state_estimate.Z);  // altitude is positive but +Z is down
				//tmp = -setpoint.Z_throttle_0 + rc_filter_march(&D_Z, z_error);  // altitude is positive but +Z is down
				rc_saturate_double(&tmp, MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
				
				u[VEC_Z] = tmp / (cos(state_estimate.roll)*cos(state_estimate.pitch));
                //u[VEC_Z] = tmp;
				rc_saturate_double(&u[VEC_Z], MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
				
				mix_add_input(u[VEC_Z], VEC_Z, mot);
				last_en_Z_ctrl = 1;
			}
	// else use direct throttle
	else
	{
		// compensate for tilt
		tmp = setpoint.Z_throttle / (cos(state_estimate.roll)*cos(state_estimate.pitch));
		//printf("throttle: %f\n",tmp);
		rc_saturate_double(&tmp, MIN_THRUST_COMPONENT, MAX_THRUST_COMPONENT);
		u[VEC_Z] = tmp;
		mix_add_input(u[VEC_Z], VEC_Z, mot);
	}

	/***************************************************************************
	* Roll Pitch Yaw controllers, only run if enabled
	***************************************************************************/
	if(setpoint.en_rpy_ctrl){
		// Roll
		mix_check_saturation(VEC_ROLL, mot, &min, &max);
		if(max>MAX_ROLL_COMPONENT)  max =  MAX_ROLL_COMPONENT;
		if(min<-MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
		rc_filter_enable_saturation(&D_roll, min, max);
		D_roll.gain = D_roll_gain_orig * settings.v_nominal/state_estimate.v_batt_lp;
		 // B: rc_filter_march is PID controller
		u[VEC_ROLL] = rc_filter_march(&D_roll, setpoint.roll - state_estimate.roll);
		mix_add_input(u[VEC_ROLL], VEC_ROLL, mot);
	
		// pitch
		mix_check_saturation(VEC_PITCH, mot, &min, &max);
		if(max>MAX_PITCH_COMPONENT)  max =  MAX_PITCH_COMPONENT;
		if(min<-MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
		rc_filter_enable_saturation(&D_pitch, min, max);
		D_pitch.gain = D_pitch_gain_orig * settings.v_nominal/state_estimate.v_batt_lp;
		u[VEC_PITCH] = rc_filter_march(&D_pitch, setpoint.pitch - state_estimate.pitch);
		mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);

		// Yaw
		// if throttle stick is down (waiting to take off) keep yaw setpoint at
		// current heading, otherwide update by yaw rate
		mix_check_saturation(VEC_YAW, mot, &min, &max);
		if(max>MAX_YAW_COMPONENT)  max =  MAX_YAW_COMPONENT;
		if(min<-MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
		rc_filter_enable_saturation(&D_yaw, min, max);
		D_yaw.gain = D_yaw_gain_orig * settings.v_nominal/state_estimate.v_batt_lp;
		u[VEC_YAW] = rc_filter_march(&D_yaw, setpoint.yaw - state_estimate.continuous_yaw);
		mix_add_input(u[VEC_YAW], VEC_YAW, mot);
	}
	// otherwise direct throttle to roll pitch yaw
	else{
		// roll
		mix_check_saturation(VEC_ROLL, mot, &min, &max);
		if(max>MAX_ROLL_COMPONENT)  max =  MAX_ROLL_COMPONENT;
		if(min<-MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
		u[VEC_ROLL] = setpoint.roll_throttle;
		rc_saturate_double(&u[VEC_ROLL], min, max);
		mix_add_input(u[VEC_ROLL], VEC_ROLL, mot);

		// pitch
		mix_check_saturation(VEC_PITCH, mot, &min, &max);
		if(max>MAX_PITCH_COMPONENT)  max =  MAX_PITCH_COMPONENT;
		if(min<-MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
		u[VEC_PITCH] = setpoint.pitch_throttle;
		rc_saturate_double(&u[VEC_PITCH], min, max);
		mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);

		// YAW
		mix_check_saturation(VEC_YAW, mot, &min, &max);
		if(max>MAX_YAW_COMPONENT)  max =  MAX_YAW_COMPONENT;
		if(min<-MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
		u[VEC_YAW] = setpoint.yaw_throttle;
		rc_saturate_double(&u[VEC_YAW], min, max);
		mix_add_input(u[VEC_YAW], VEC_YAW, mot);
	}

	// for 6dof systems, add X and Y
	if(setpoint.en_6dof){
		// X
		mix_check_saturation(VEC_X, mot, &min, &max);
		if(max>MAX_X_COMPONENT)  max =  MAX_X_COMPONENT;
		if(min<-MAX_X_COMPONENT) min = -MAX_X_COMPONENT;
		u[VEC_X] = setpoint.X_throttle;
		rc_saturate_double(&u[VEC_X], min, max);
		mix_add_input(u[VEC_X], VEC_X, mot);

		// Y
		mix_check_saturation(VEC_Y, mot, &min, &max);
		if(max>MAX_Y_COMPONENT)  max =  MAX_Y_COMPONENT;
		if(min<-MAX_Y_COMPONENT) min = -MAX_Y_COMPONENT;
		u[VEC_Y] = setpoint.Y_throttle;
		rc_saturate_double(&u[VEC_Y], min, max);
		mix_add_input(u[VEC_Y], VEC_Y, mot);
	}

	/***************************************************************************
	* Send ESC motor signals immediately at the end of the control loop
	***************************************************************************/
	for(i=0;i<settings.num_rotors;i++){
		rc_saturate_double(&mot[i], 0.0, 1.0);
		fstate.m[i] = map_motor_signal(mot[i]);

		// final saturation just to take care of possible rounding errors
		// this should not change the values and is probably excessive
		rc_saturate_double(&fstate.m[i], 0.0, 1.0);

		// finally send pulses!
		rc_servo_send_esc_pulse_normalized(i+1,fstate.m[i]);
	}

	/***************************************************************************
	* Final cleanup, timing, and indexing
	***************************************************************************/
	// Load control inputs into cstate for viewing by outside threads
	for(i=0;i<6;i++) fstate.u[i]=u[i];
	// keep track of loops since arming
		fstate.loop_index++;
	// log us since arming, mostly for the log
	fstate.last_step_ns = rc_nanos_since_boot();

	return 0;
}


int feedback_cleanup(void)
{
	__send_motor_stop_pulse();
	return 0;
}
