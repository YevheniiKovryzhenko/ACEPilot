/**
 * <flight_mode.hpp>
 */

#ifndef FLIGHT_MODE_HPP
#define FLIGHT_MODE_HPP

/**
 * This is how the user interacts with the setpoint manager.
 */
typedef enum flight_mode_t{
	/**
	* test_bench mode does no feedback at all, it takes the raw user inputs
	* and directly outputs to the motors. This could technically fly but
	* would not be easy! Designed for confirming mixing matrix and motors
	* are working. maps Z,Roll,Pitch,Yaw
	*/
	TEST_BENCH_4DOF,

	/**
	* test_bench mode does no feedback at all, it takes the raw user inputs
	* and directly outputs to the motors. This could technically fly but
	* would not be easy! Designed for confirming mixing matrix and motors
	* are working. maps X,Y,Z,Yaw
	*/
	TEST_BENCH_6DOF,

	/**
	* Radio sticks are directly controling 6 servo channels, each individually. 
	* Can be used for testing servo channels, don't ever try to fly with this mode
	*/
	TEST_6xSERVOS_DIRECT,

	/**
	* user inputs translate directly to the throttle, roll, pitch, & yaw
	* setpoints. No altitude feedback control.Roll and pitch are left at 0
	*/
	DIRECT_THROTTLE_6DOF,

	/**
	* Attitude sticks command attitude rate (roll, pitch, yaw)
	* Throttle stick directly controls throttle
	* Can be used for fpv drone racing and also attitude rate tuning in the pvc rig
	*/
	ACRO_Axxxxx,

	/**
	* Attitude sticks command attitude rate (roll, pitch, yaw)
	* Throttle stick directly controls throttle. Feedforward path enabled.
	* Can be used for fpv drone racing and also attitude rate tuning in the pvc rig
	*/
	ACRO_Fxxxxx,

	/**
	* user inputs translate directly to the throttle, roll, pitch, & yaw
	* setpoints. No altitude feedback control. On 6DOF platforms the X & Y
	* thrust directions are left at 0. No attitude rate control
	*/
	MANUAL_xAxxxx,

	/**
	* user inputs translate directly to the throttle, roll, pitch, & yaw
	* setpoints. No altitude feedback control. On 6DOF platforms the X & Y
	* thrust directions are left at 0. Attitude rate control enabled
	*/
	MANUAL_AAxxxx,

	/**
	* user inputs translate directly to the throttle, roll, pitch, & yaw
	* setpoints. No altitude feedback control. On 6DOF platforms the X & Y
	* thrust directions are left at 0. No attitude rate control
	* Feedforward path enabled for attitude only.
	*/
	MANUAL_xFxxxx,

	/**
	* user inputs translate directly to the throttle, roll, pitch, & yaw
	* setpoints. No altitude feedback control. On 6DOF platforms the X & Y
	* thrust directions are left at 0. Attitude rate control enabled.
	* Feedforward path enabled for attitude rate only.
	*/
	MANUAL_FAxxxx,

	/**
	* user inputs translate directly to the throttle, roll, pitch, & yaw
	* setpoints. No altitude feedback control. On 6DOF platforms the X & Y
	* thrust directions are left at 0. Attitude rate control enabled.
	* Feedforward path enabled for both attitude and attitude rate.
	*/
	MANUAL_FFxxxx,

	/**
	* like MANUAL_Axxxxx for roll/pitch/yaw rates but feedback is performed to
	* hold vertival velocity setpoint which is them moved up and down steadily based
	* on user input. No vertical position control.
	*/
	ALT_HOLD_AxAxxx,

	/**
	* like MANUAL_Fxxxxx for roll/pitch/yaw rates but feedback is performed to
	* hold vertival velocity setpoint which is them moved up and down steadily based
	* on user input. No vertical position control.
	*/
	ALT_HOLD_FxAxxx,

	/**
	* like MANUAL_Fxxxxx for roll/pitch/yaw rates but feedback is performed to
	* hold vertival velocity setpoint which is them moved up and down steadily based
	* on user input. No vertical position control. Feedforward path enabled for velocity.
	*/
	ALT_HOLD_FxFxxx,

	/**
	* like MANUAL_Axxxxx for roll/pitch/yaw rates but feedback is performed to
	* hold vertival position setpoint which is them moved up and down steadily based
	* on user input. Vertical velocity control is enabled.
	*/
	ALT_HOLD_AxAAxx,

	/**
	* like MANUAL_Fxxxxx for roll/pitch/yaw rates but feedback is performed to
	* hold vertival position setpoint which is them moved up and down steadily based
	* on user input. Vertical velocity control is enabled.
	*/
	ALT_HOLD_FxAAxx,

	/**
	* like MANUAL_Fxxxxx for roll/pitch/yaw rates but feedback is performed to
	* hold vertival position setpoint which is them moved up and down steadily based
	* on user input. Vertical velocity control is enabled. Feedforward path enabled
	* for velocity only.
	*/
	ALT_HOLD_FxFAxx,

	/**
	* like MANUAL_Fxxxxx for roll/pitch/yaw rates but feedback is performed to
	* hold vertival position setpoint which is them moved up and down steadily based
	* on user input. Vertical velocity control is enabled. Feedforward path enabled.
	*/
	ALT_HOLD_FxFFxx,

	/**
	* like MANUAL_xAxxxx for roll/pitch/yaw but feedback is performed to
	* hold altitude setpoint which is them moved up and down steadily based
	* on user input. No vertical velocity control.
	*/
	ALT_HOLD_xAxAxx,

	/**
	* like MANUAL_xFxxxx for roll/pitch/yaw but feedback is performed to
	* hold altitude setpoint which is them moved up and down steadily based
	* on user input. No vertical velocity control.
	*/
	ALT_HOLD_xFxAxx,

	/**
	* like MANUAL_xFxxxx for roll/pitch/yaw but feedback is performed to
	* hold altitude setpoint which is them moved up and down steadily based
	* on user input. No vertical velocity control. 
	* Feedforward path enabled for vertical position controler.
	*/
	ALT_HOLD_xFxFxx,

	/**
	* like MANUAL_AAxxxx for roll/pitch/yaw but feedback is performed to
	* hold altitude setpoint which is them moved up and down steadily based
	* on user input. No vertical velocity control.
	*/
	ALT_HOLD_AAxAxx,

	/**
	* like MANUAL_FAxxxx for roll/pitch/yaw but feedback is performed to
	* hold altitude setpoint which is them moved up and down steadily based
	* on user input. No vertical velocity control.
	*/
	ALT_HOLD_FAxAxx,

	/**
	* like MANUAL_FFxxxx for roll/pitch/yaw but feedback is performed to
	* hold altitude setpoint which is them moved up and down steadily based
	* on user input. No vertical velocity control.
	*/
	ALT_HOLD_FFxAxx,

	/**
	* like MANUAL_FFxxxx for roll/pitch/yaw but feedback is performed to
	* hold altitude setpoint which is them moved up and down steadily based
	* on user input. No vertical velocity control.
	* Feedforward path enabled for vertical position controler.
	*/
	ALT_HOLD_FFxFxx,

	/**
	* like MANUAL_AAxxxx for roll/pitch/yaw but feedback is performed to
	* hold vertical velocity setpoint which is them moved up and down steadily based
	* on user input. 
	*/
	ALT_HOLD_AAAxxx,

	/**
	* like MANUAL_FAxxxx for roll/pitch/yaw but feedback is performed to
	* hold vertical velocity setpoint which is them moved up and down steadily based
	* on user input.
	*/
	ALT_HOLD_FAAxxx,

	/**
	* like MANUAL_FFxxxx for roll/pitch/yaw but feedback is performed to
	* hold vertical velocity setpoint which is them moved up and down steadily based
	* on user input.
	*/
	ALT_HOLD_FFAxxx,

	/**
	* like MANUAL_FF for roll/pitch/yaw but feedback is performed to
	* hold vertical velocity setpoint which is them moved up and down steadily based
	* on user input. Feedforward path enabled for vertical velocity controler. 
	*/
	ALT_HOLD_FFFxxx,

	/**
	* like MANUAL_AA for roll/pitch/yaw but feedback is performed to
	* hold altitude setpoint which is them moved up and down steadily based
	* on user input. Attitude rate control and velocity control.
	*/
	ALT_HOLD_AAAAxx,

	/**
	* like MANUAL_FA for roll/pitch/yaw but feedback is performed to
	* hold altitude setpoint which is them moved up and down steadily based
	* on user input. Attitude rate control and velocity control.
	*/
	ALT_HOLD_FAAAxx,

	/**
	* like MANUAL_FF for roll/pitch/yaw but feedback is performed to
	* hold altitude setpoint which is them moved up and down steadily based
	* on user input. Attitude rate control and velocity control.
	*/
	ALT_HOLD_FFAAxx,

	/**
	* like MANUAL_FF for roll/pitch/yaw but feedback is performed to
	* hold altitude setpoint which is them moved up and down steadily based
	* on user input. Attitude rate control and velocity control. Feedforward
	* path enabled for velocity controler. 
	*/
	ALT_HOLD_FFFAxx,

	/**	
	* like MANUAL_FF for roll/pitch/yaw but feedback is performed to
	* hold altitude setpoint which is them moved up and down steadily based
	* on user input. Attitude rate control and velocity control. Feedforward
	* path enabled for velocity and altitude controler. 
	*/
	ALT_HOLD_FFFFxx,

	/**
	* Like ALT_HOLD_AAAAxx. Enabled horisonal velocity and
	* position controllers.
	* Roll and pitch sticks are converted into position setpoints.
	*/
	POS_CTRL_AAAAAA,

	/**
	* Like ALT_HOLD_FFFAxx. Enabled horisonal velocity controllers.
	* Roll and pitch sticks are converted into velocity setpoints.
	* No horisontal position control.
	*/
	POS_CTRL_FFFAAx,

	/**
	* Like ALT_HOLD_FFFAxx. Enabled horisonal velocity controllers.
	* Roll and pitch sticks are converted into velocity setpoints.
	* No horisontal position control. Feed forward path enabled for velocity.
	*/
	POS_CTRL_FFFAFx,

	/**
	* Like ALT_HOLD_FFFFxx. Enabled horisonal velocity controllers.
	* Roll and pitch sticks are converted into velocity setpoints.
	* No horisontal position control.
	*/
	POS_CTRL_FFFFAx,

	/**
	* Like ALT_HOLD_FFFAxx. Enabled horisonal velocity and 
	* position controllers.
	* Roll and pitch sticks are converted into position setpoints.
	*/
	POS_CTRL_FFFAAA,

	/**
	* Like ALT_HOLD_FFFAxx. Enabled horisonal velocity and
	* position controllers.
	* Roll and pitch sticks are converted into position setpoints.
	* Feedforward path enabled for traslational velocity controllers.
	*/
	POS_CTRL_FFFAFA,

	/**
	* Like ALT_HOLD_FFFAxx. Enabled horisonal velocity and
	* position controllers.
	* Roll and pitch sticks are converted into position setpoints.
	* Feedforward path enabled for traslational position and velocity controllers.
	*/
	POS_CTRL_FFFAFF,

	/**
	* Like ALT_HOLD_FFFFxx. Enabled horisonal velocity and
	* position controllers.
	* Roll and pitch sticks are converted into position setpoints.
	* Feedforward path enabled for traslational position and velocity controllers.
	*/
	POS_CTRL_FFFFFF,
	
	/**
	* Commands 0 roll, 0 pitch, and descends at maximum user-defined speed
	* Useful as an emergency mode if MOCAP drops out for too long
	*/
	EMERGENCY_LAND,
	
	/**
	* Fully autoomous position control mode. Just like POS_CTRL_FFFAFA
	* No external user input 
	*/
	AUTO_FFFAFA,
	/*
	* Zeppelin mixed servo/esc control with thrust vectoring
	*/
	ZEPPELIN
} flight_mode_t;


/** This function is basically a list of the enums in 'flight_mode_t'
 * that require Motion Capture (MOCAP) to function properly.
 */
bool mode_needs_mocap(flight_mode_t mode);



#endif