/*
 * servos.cpp
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
 * Last Edit:  07/30/2020 (MM/DD/YYYY)
 */

#include "tools.h"
#include "servos.hpp"
#include "mix_servos.hpp"
#include "rc_pilot_defs.h"
#ifdef RC_PILOT_DEFS_H ////only need these if used with rc_pilot
#include "settings.h"
#endif // RC_PILOT_DEFS_H ////only need these if used with rc_pilot


 // preposessor macros
#define unlikely(x)	__builtin_expect (!!(x), 0)
#define likely(x)	__builtin_expect (!!(x), 1)

 /*!
  *  @brief  Class that stores state and functions for interacting with PCA9685
  * PWM chip
  */
  /*
  * Invoke defaut constructor for all the built in and exernal types in the class
  */
servo_state_t sstate{};

 /*
 * This function should be used anytime
 * the servos need to be returned to
 * their nominal (safe) positions
 */
int servo_state_t::set_nom_pulse(void)
{
    for (int i = 0; i < MAX_SERVOS; i++) {
        m_us[i] = nom_us[i]; //have to set to calibrated nominal values
    }

    return 0;
}

/*
 * This function should be used anytime
 * all the servos need to be set to
 * their min positions
 */
int servo_state_t::set_min_pulse(void)
{
    for (int i = 0; i < MAX_SERVOS; i++)
    {
        m_us[i] = min_us[i];  // have to set to calibrated min values
    }

    return 0;
}

/*
 * This function should be used anytime
 * all the servos need to be set to
 * their max positions
 */
int servo_state_t::set_max_pulse(void)
{
    for (int i = 0; i < MAX_SERVOS; i++)
    {
        m_us[i] = max_us[i];  // have to set to calibrated min values
    }

    return 0;
}



/*
* This function coverts commanded signal
* in [0 1] range into pulse width in
* miroseconds for motor i
*/
int servo_state_t::cmnd2us(int i)
{
    // sanity check
    if (unlikely(m[i] > 1.0 || m[i] < 0.0))
    {
        printf("\nERROR: in CMD2PWM, command signal must be between 0.0 & 1.0");
        return -1;
    }

    // return quickly for boundary conditions
    if (m[i] == 0.0)
    {
        m_us[i] = min_us[i];
        return 0;
    }
    if (m[i] == 1.0)
    {
        m_us[i] = max_us[i];
        return 0;
    }

    // Map [0 1] signal to servo pulse width
    m_us[i] = m[i] * (max_us[i] - min_us[i]) + min_us[i];
    return 0;
}

/*
* This is how the user should initialize
* servos. Initializes both the servo driver
* and i2c driver
*/
/* Returns 0 on success or -1 on failure */
int servo_state_t::init(int driver_bus_id)
{
    return (init(driver_bus_id, DEF_I2C_ADDRESS));
}

int servo_state_t::set_servo_calibration(void)
{
    for (int i = 0; i < 8; i+=2)
    {
        if (unlikely(set_min_us(i, SERVO_MIN_US) == -1))
        {
            printf("\nERROR in set_servo_calibration: failed to set new min for servo %d", i);
            return -1;
        }
        if (unlikely(set_max_us(i, SERVO_MIN_US + SERVO_DEG_US_INC * 150) == -1))
        {
            printf("\nERROR in set_servo_calibration: failed to set new max for servo %d", i);
            return -1;
        }
        if (unlikely(set_nom_us(i, SERVO_MIN_US + SERVO_DEG_US_INC * 20) == -1))
        {
            printf("\nERROR in set_servo_calibration: failed to set new nom for servo %d", i);
            return -1;
        }

        if (unlikely(set_min_us(i + 1, SERVO_MAX_US) == -1))
        {
            printf("\nERROR in set_servo_calibration: failed to set new min for servo %d", i + 1);
            return -1;
        }
        if (unlikely(set_max_us(i + 1, SERVO_MAX_US - SERVO_DEG_US_INC * 150) == -1))
        {
            printf("\nERROR in set_servo_calibration: failed to set new max for servo %d", i + 1);
            return -1;
        }
        if (unlikely(set_nom_us(i + 1, SERVO_MAX_US - SERVO_DEG_US_INC * 20) == -1))
        {
            printf("\nERROR in set_servo_calibration: failed to set new nom for servo %d", i + 1);
            return -1;
        }
    }
    
    printf("\nCalibration for servos is set to:");
    printf("\nservo      min         nom         max");
    for (int i = 0; i < MAX_SERVOS; i++)
    {
        printf("\n%d    %f     %f   %f", i, min_us[i], nom_us[i], max_us[i]);
    }
    
    return 0;
}

/*
* This is how the user should initialize
* servos. Initializes both the servo driver
* and i2c driver
*/
/* Returns 0 on success or -1 on failure */
int servo_state_t::init(int driver_bus_id, uint8_t devAddr)
{
    if (unlikely(initialized))
    {
        printf("\nERROR in init: servos already initialized");
        return -1;
    }

#ifdef RC_PILOT_DEFS_H //only need arming/disarming if used with rc_pilot
    arm_state = DISARMED;
#endif
    arm_time_ns = 0;

    // initialize PRU //(if used on BBB rails)
    //if (rc_servo_init()) return -1;

    printf("\nInitializing servos....");
    if (unlikely(i2c_driver.open(driver_bus_id, devAddr) == -1))
    {
        printf("\nERROR: Failed to initialize servos.");
        return -1;
    }

    if (unlikely(servo_driver.init(i2c_driver) == -1)) //initialize servo driver
    {
        printf("\nERROR: Failed to initialize servos.");
        return -1;
    }

    servo_driver.setOscillatorFrequency(25E6);

    if (unlikely(servo_driver.setPWMFreq(50) == -1))
    {
        printf("\nERROR: Failed to initialize servos.");
        return -1;
    }

    usleep(10E3);

    for (int i = 0; i < MAX_SERVOS; i++) {
        m[i] = 0; //zero everything out just in case

        if (unlikely(set_min_us(i, SERVO_MIN_US)))
        {
            printf("\nERROR: failed to initialize servos");
            return -1;
        }

        if (unlikely(set_nom_us(i, SERVO_NOM_US)))
        {
            printf("\nERROR: failed to initialize servos");
            return -1;
        }

        if (unlikely(set_max_us(i, SERVO_MAX_US)))
        {
            printf("\nERROR: failed to initialize servos");
            return -1;
        }
    }

    if (unlikely(set_servo_calibration() == -1))
    {
        printf("\nERROR: failed to initialize servos");
        return -1;
    }

    set_nom_pulse();

#ifdef RC_PILOT_DEFS_H //only need arming/disarming if used with rc_pilot
    if (unlikely(controller.init() == -1))
    {
        printf("\nERROR in init: failed to initialize servo controller");
        return -1;
    }

    if (unlikely(controller.reset() == -1))
    {
        printf("\nERROR in init: failed to reset servo controller");
        return -1;
    }

#endif
    printf("\nSuccess, servos initialized");
    initialized = true;

    //send servo signals using Pulse Width in microseconds
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (unlikely(servo_driver.writeMicroseconds(i, m_us[i]) == -1))
        {
            printf("\nERROR in march: failed to write signal in us");
            return -1;
        }
    }

    return 0;
}

bool servo_state_t::is_initialized(void)
{
    return initialized;
}


#ifdef RC_PILOT_DEFS_H //only need arming/disarming if used with rc_pilot
int servo_state_t::arm(void)
{
    if (unlikely(arm_state == ARMED)) {
        printf("WARNING: trying to arm when servos are already armed\n");
        return 0;
    }
    if (unlikely(!initialized))
    {
        printf("ERROR in arm: servos have not been initialized \n");
        return -1;
    }
    if (unlikely(controller.reset() == -1))
    {
        printf("\nERROR in arm: failed to reset servo controller");
        return -1;
    }

    controller.reset();
    // need to set each of the servos to their nominal positions:
    set_nom_pulse();

    arm_time_ns = rc_nanos_since_boot();
    arm_state = ARMED; //set servos to armed and powered
    printf("\n WARNING: Waking up the servos....");
    do // at least once, but maybe more when specified
    {
        // need to set each of the servos to their nominal positions:
        return_to_nominal();
    } while (finddt_s(arm_time_ns) < settings.servos_arm_time_s);
    printf("\nDone! Servo wake up time was %f", finddt_s(arm_time_ns));

    //enable power: //(if used of BBB servo rails)
    //rc_servo_power_rail_en(1);

    arm_state = ARMED; //set servos to armed and powered
    return 0;
}

bool servo_state_t::is_armed(void)
{
    return arm_state;
}

int servo_state_t::return_to_nominal(void)
{
    if (unlikely(!initialized))
    {
        printf("ERROR in return_to_nominal: Servos have not been initialized \n");
        return -1;
    }

    set_nom_pulse(); //do this every time to ensure nominal position

    if (arm_state == DISARMED) {
        //no need to proceed if already disarmed (no power to servo rail)
        return 0;
    }

    //send servo signals using Pulse Width in microseconds
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (unlikely(servo_driver.writeMicroseconds(i, m_us[i]) == -1))
        {
            printf("\nERROR in march: failed to write signal in us");
            return -1;
        }
    }
    return 0;
}

int servo_state_t::disarm(void)
{
    // need to set each of the servos to their nominal positions:
    set_nom_pulse(); //won't work, need extra time before power is killed

    if (arm_state == DISARMED)
    {
        //no need to proceed if already disarmed (no power to servo rail)
        printf("WARNING: trying to disarm when servos are already disarmed\n");
        return 0;
    }
    //send servo signals using Pulse Width in microseconds
    for (int i = 0; i < MAX_SERVOS; i++) {
        if (unlikely(servo_driver.writeMicroseconds(i, m_us[i]) == -1))
        {
            printf("\nERROR in march: failed to write signal in us");
            return -1;
        }
    }

    //power-off servo rail: //if running on BBB rail
    //rc_servo_power_rail_en(0);

    arm_state = DISARMED;
    return 0;
}
#endif //RC_PILOT_DEFS_H


/*
* This is how the user should change the servo
* command. Accepts servo command signal in
* between 0 and 1 range for servo i.
*/
/* Returns 0 on success or -1 on failure */
int servo_state_t::set_cmnd_signal(int i, double signal)
{
    if (unlikely(i > MAX_SERVOS || i < 0))
    {
        printf("\nERROR in set_cmnd_signal: trying to control motor %d, \
            motor number has to be between 0 and %d", i, MAX_SERVOS);
        return -1;
    }

    if (unlikely(signal > 1.0 || signal < 0.0))
    {
        printf("\nERROR in set_cmnd_signal: trying to set control signal as %f, \
            must be between 0.0 and 1.0", signal);
        return -1;
    }

    m[i] = signal;
    return 0;
}


/*
* This is a simple saturation function which
* limits the signal between 0 and 1.
*/
/* Returns 0 on success or -1 on failure */
int servo_state_t::cmnd_signal_saturate(int i, double signal)
{
    if (signal < 1.0 && signal > 0.0)
    {
        m[i] = signal;
        return 0;
    }
    else if (signal == 0.0)
    {
        m[i] = signal;
        return 0;
    }
    else if (signal == 1.0)
    {
        m[i] = signal;
        return 0;
    }
    else if (signal > 1.0)
    {
        m[i] = 1.0;
        return 0;
    }
    else if (signal < 0.0)
    {
        m[i] = 0.0;
        return 0;
    }
    else
    {
        printf("\nERROR in cmnd_signal_saturate: unknown signal command.");
        return -1;
    }
}


#ifdef RC_PILOT_DEFS_H //only need arming/disarming if used with rc_pilot
int servo_state_t::march_controller(double(&u)[MAX_SERVO_INPUTS], double(&mot)[MAX_SERVOS])
{
    return controller.march(u, mot);
}
#endif // RC_PILOT_DEFS_H //only need arming/disarming if used with rc_pilot


/*
* This is how the user should march (apply)
* servo command. Sends m[i] coverted into PWM
* motor signal to motor i.
*/
/* Returns 0 on success or -1 on failure */
int servo_state_t::march(int i, double signal)
{
    
    if (unlikely(!initialized))
    {
        printf("\nERROR in march: trying to march servos when not initialized");
        return -1;
    }

#ifdef RC_PILOT_DEFS_H //only need arming/disarming if used with rc_pilot
    if (unlikely(arm_state == DISARMED)) {
        printf("WARNING: trying to march servos when servos disarmed\n");
        return 0;
    }
#endif // RC_PILOT_DEFS_H //only need arming/disarming if used with rc_pilot

    
    // need to do mapping between [0 1] and servo signal in us
    if (unlikely(cmnd_signal_saturate(i, signal) == -1))
    {
        printf("\nERROR in march: failed to set command signal");
        return -1;
    }
    

    // need to do mapping between [0 1] and servo signal in us
    if (unlikely(cmnd2us(i) == -1))
    {
        printf("\nERROR in march: failed to convert signal to us");
        return -1;
    }

    //send servo signals using Pulse Width in microseconds
    //if (rc_servo_send_pulse_us(i + 1, sstate.m_us[i]) == -1) return -1;
    if (unlikely(servo_driver.writeMicroseconds(i, m_us[i]) == -1))
    {
        printf("\nERROR in march: failed to write signal in us");
        return -1;
    }

    return 0;
}


/*
* This is how the user should preset minimum
* servo pulse width in microseconds for motor i.
*/
/* Returns 0 on success or -1 on failure */
int servo_state_t::set_min_us(int i, double signal_us)
{
    if (unlikely(i > MAX_SERVOS || i < 0))
    {
        printf("\nERROR in set_min_us: trying to control motor %d, \
            motor number has to be between 0 and %d", i, MAX_SERVOS);
        return -1;
    }

    if (unlikely(signal_us > SERVO_MAX_US || signal_us < SERVO_MIN_US))
    {
        printf("\nERROR in set_min_us: trying to set minimum pulse width as %fus, \
            must be between %fus and %fus", signal_us, SERVO_MIN_US, SERVO_MAX_US);
        return -1;
    }

    min_us[i] = signal_us;
    return 0;
}


/*
* This is how the user should preset nominal
* servo pulse width in microseconds for motor i.
*/
/* Returns 0 on success or -1 on failure */
int servo_state_t::set_nom_us(int i, double signal_us)
{
    if (unlikely(i > MAX_SERVOS || i < 0))
    {
        printf("\nERROR in set_nom_us: trying to control motor %d, \
            motor number has to be between 0 and %d", i, MAX_SERVOS);
        return -1;
    }

    if (unlikely(signal_us > SERVO_MAX_US || signal_us < SERVO_MIN_US))
    {
        printf("\nERROR in set_nom_us: trying to set nominal pulse width as %fus, \
            must be between %fus and %fus", signal_us, SERVO_MIN_US, SERVO_MAX_US);
        return -1;
    }

    nom_us[i] = signal_us;
    return 0;
}


/*
* This is how the user should preset maximum
* servo pulse width in microseconds for motor i.
*/
/* Returns 0 on success or -1 on failure */
int servo_state_t::set_max_us(int i, double signal_us)
{
    if (unlikely(i > MAX_SERVOS || i < 0))
    {
        printf("\nERROR in set_max_us: trying to control motor %d, \
            motor number has to be between 0 and %d", i, MAX_SERVOS);
        return -1;
    }

    if (unlikely(signal_us > SERVO_MAX_US || signal_us < SERVO_MIN_US))
    {
        printf("\nERROR in set_max_us: trying to set maximum pulse width as %fus, \
            must be between %fus and %fus", signal_us, SERVO_MIN_US, SERVO_MAX_US);
        return -1;
    }

    max_us[i] = signal_us;
    return 0;
}

/*
* This is testing function for checking the primary
* functunality of servo driver. Sweeps servos from 0
* to 1 for all channels.
*/
/* Returns 0 on success or -1 on failure */
int servo_state_t::test_min_max(void)
{
    printf("\nRunning servo test...");
    if (unlikely(!initialized))
    {
        printf("\nERROR in march: trying to march servos when not initialized");
        return -1;
    }
    for (int motor_number = 0; motor_number < MAX_SERVOS; motor_number++)
    {
        printf("\nTesting motor number: %d", motor_number);

        for (double signal = 0.0; signal < 1.0; signal += 0.1)
        {
            if (march(motor_number, signal) == -1)
            {
                printf("\nERROR: failed to march servos");
                return -1;
            }
        }
        usleep(0.5E6);

        for (double signal = 1.0; signal > 0.0; signal -= 0.1)
        {
            if (march(motor_number, signal) == -1)
            {
                printf("\nERROR: failed to march servos");
                return -1;
            }
        }
        usleep(0.1E6);
    }
    printf("\nMin/Max servo test completed...");
    return 0;
}



/*
* This is how the user should terminate the servo
* driver and i2c driver. Run this on exit.
*/
/* Returns 0 on success or -1 on failure */
int servo_state_t::cleanup(void)
{
    printf("\nCleaning up servos...");
    if (unlikely(initialized))
    {
        if (unlikely(servo_driver.reset() == -1))
        {
            printf("\nWARNING: Failed to reset servo driver");
        }

        if (unlikely(i2c_driver.close()))
        {
            printf("\nERROR: Failed to cleanup servos");
            return -1;
        }
    }
    else
    {
        printf("\nWARNING: servos were not initialized, can't cleanup");
    }
    initialized = false;
    return 0;
}