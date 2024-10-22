﻿/*
 * main.cpp
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
 * Last Edit:  09/17/2022 (MM/DD/YYYY)
 */

#include "main.hpp"

using namespace std;
struct stat buffer;
//check if running on BBB:
static const bool RUNNING_ON_BBB = (stat("/sys/devices/platform/leds/leds/", &buffer) == 0 && S_ISDIR(buffer.st_mode)); 


/**
 *  @brief      Standard exit for initialization failures
 */
#define FAIL(str)                       \
    fprintf(stderr, str);               \
    if (RUNNING_ON_BBB) rc_led_set(RC_LED_GREEN, 0);                        \
    if (RUNNING_ON_BBB) rc_led_blink(RC_LED_RED, 8.0, 2.0);                 \
    printf("\ncleaning up");            \
    rc_mpu_power_off();                 \
    if (fstate.is_initialized()) fstate.cleanup();                          \
    if (user_input.is_initialized()) user_input.input_manager_cleanup();    \
    if (setpoint.is_initialized()) setpoint.cleanup();                      \
    printf_cleanup();                   \
    log_entry.cleanup();                \
    rc_encoder_cleanup();               \
    if (sstate.is_initialized()) sstate.cleanup();                          \
    return -1;

/**
 *  @brief      Prints main function commond line ussage (arguments)
 */
void print_usage()
{
    printf("\n");
    printf(" Options\n");
    printf(" -s {settings file} Specify settings file to use\n");
    printf(" -h                 Print this help message\n");
    printf("\n");
    printf("Some example settings files are included with the\n");
    printf("source code. You must specify the location of one of these\n");
    printf("files or ideally the location of your own settings file.\n");
    printf("\n");


}

/**
 * temporary check for dsm calibration until I add this to librobotcontrol
 */
int __rc_dsm_is_calibrated()
{
    if (!access("/var/lib/robotcontrol/dsm.cal", F_OK)) return 1;
    else return 0;
}

/**
* If the user holds the pause button for 2 seconds, set state to exiting which
* triggers the rest of the program to exit cleanly.
*/

void on_pause_press()
{
    int i = 0;
    const int quit_check_us = 100000;
    const int samples = 2000000 / quit_check_us;

    // toggle betewen paused and running modes
    if (rc_get_state() == RUNNING) {
        rc_set_state(PAUSED);
        printf("PAUSED\n");
    }
    else if (rc_get_state() == PAUSED) {
        rc_set_state(RUNNING);
        printf("RUNNING\n");
    }
    fflush(stdout);

    // now keep checking to see if the button is still held down
    for (i = 0; i < samples; i++) {
        rc_usleep(quit_check_us);
        if (rc_button_get_state(RC_BTN_PIN_PAUSE) == RC_BTN_STATE_RELEASED) {
            return;
        }
    }
    printf("long press detected, shutting down\n");
    rc_set_state(EXITING);
    return;
}

/**
 * @brief      Interrupt service routine for IMU0
 *
 * This is called every time the Invensense IMU0 has new data
 *
 * __imu_isr should be running at 200 Hz
 */

static void __imu_isr(void)
{
    //update comms:
    comms_manager.update(); 

    //Navigation
    state_estimate.march();
    benchmark_timers.tNAV = rc_nanos_since_boot(); //using this for velocity estimation

    //Guidance
    setpoint.update();
    if (settings.log_benchmark) benchmark_timers.tGUI = rc_nanos_since_boot();

    //Control
    fstate.march();
    if (settings.log_benchmark) benchmark_timers.tCTR = rc_nanos_since_boot();

    //Save data to log file
    if (settings.enable_logging)
    {
        if (settings.log_only_while_armed)
        {
            if (fstate.get_arm_state() == ARMED)
            {
                log_entry.data_available();
            }
        }
        else
        {
            log_entry.data_available();
        }
    }   
    

    //Currently, this only reads from the BMP pressure sensor
    state_estimate.march_jobs_after_feedback();
    if (settings.log_benchmark) benchmark_timers.tIMU_END = rc_nanos_since_boot();    
}


/**
 * Initialize the IMU0, start all the threads, and wait until something triggers
 * a shut down by setting the RC state to EXITING.
 *
 * @return     0 on success, -1 on failure
 */
int main(int argc, char** argv) 
{
    
    int c;
    char* settings_file_path = NULL;

    // parse arguments
    opterr = 0;
    while ((c = getopt(argc, argv, "s:h")) != -1) {
        switch (c) {
            // settings file option
        case 's':
            settings_file_path = optarg;
            printf("User specified settings file:\n%s\n", settings_file_path);
            break;

            // help mode
        case 'h':
            print_usage();
            return 0;

        default:
            printf("Invalid Argument\n");
            print_usage();
            return -1;
        }
    }

    // settings file option is mandatory
    if (settings_file_path == NULL) {
        print_usage();
        return -1;
    }

    // first things first, load settings which may be used during startup
    if (settings_load_from_file(settings_file_path) < 0) {
        fprintf(stderr, "ERROR: failed to load settings\n");
        char tmp[256];
        getcwd(tmp, 256);
        cout << "Current working directory: " << tmp << endl;
        return -1;
    }
    printf("Loaded settings: %s\n", settings.name);
    

    // before touching hardware, make sure another instance isn't running
    // return value -3 means a root process is running and we need more
    // privileges to stop it.
    if (rc_kill_existing_process(2.0) == -3) return -1;

    // start with both LEDs off
    if (RUNNING_ON_BBB)
    {
        if (rc_led_set(RC_LED_GREEN, 0) == -1)
        {
            fprintf(stderr, "ERROR in main(), failed to set RC_LED_GREEN\n");
            return -1;
        }
        if (rc_led_set(RC_LED_RED, 0) == -1)
        {
            fprintf(stderr, "ERROR in main() failed to set RC_LED_RED\n");
            return -1;
        }

        // make sure IMU0 is calibrated
        if (!rc_mpu_is_gyro_calibrated()) {
            FAIL("ERROR, must calibrate gyroscope with rc_calibrate_gyro first\n")
        }
        if (!rc_mpu_is_accel_calibrated()) {
            FAIL("ERROR, must calibrate accelerometer with rc_calibrate_accel first\n")
        }
        if (settings.IMU0.compass.enable && !rc_mpu_is_gyro_calibrated()) {
            FAIL("ERROR, must calibrate magnetometer with rc_calibrate_mag first\n")
        }
        if (settings.enable_dsm && !__rc_dsm_is_calibrated()) {
            FAIL("ERROR, must calibrate DSM with rc_calibrate_dsm first\n")
        }

        // turn cpu freq to max for most consistent performance and lowest
        // latency servicing the IMU0's interrupt service routine
        // this also serves as an initial check for root access which is needed
        // by the PRU later. PRU root acces might get resolved in the future.
        if (rc_cpu_set_governor(RC_GOV_PERFORMANCE) < 0) {
            FAIL("WARNING, can't set CPU governor, need to run as root\n")
        }
    }
    // do initialization not involving threads
    printf("initializing thrust map\n");
    if (thrust_map_init(settings.thrust_map) < 0) {
        FAIL("ERROR: failed to initialize thrust map\n")
    }
    printf("\ninitializing mixing matrix\n");
    if (mix_init(settings.layout) < 0) {
        FAIL("ERROR: failed to initialize mixing matrix\n")
    }
    printf("initializing servo mixing matrix\n");
    if (servo_mix.init(settings.servo_layout) < 0) {
        FAIL("ERROR: failed to initialize mixing matrix for servos\n")
    }
    printf("initializing setpoint_manager\n");
    if (setpoint.init() < 0) {
        FAIL("ERROR: failed to initialize setpoint_manager\n")
    }

    // initialize cape hardware, this prints an error itself if unsuccessful
    if (RUNNING_ON_BBB)
    {
        printf("initializing servos\n");
        if (rc_servo_init() == -1) {
            FAIL("ERROR: failed to initialize servos, probably need to run as root\n")
        }

        if (settings.enable_servos)
        {
            if (sstate.init(settings.servo_i2c_driver_id) == -1)
            {
                printf("ERROR: failed to initialize servos\n");
                return -1;
            }
        }        

        printf("initializing adc\n");
        if (rc_adc_init() == -1) {
            FAIL("ERROR: failed to initialize ADC\n")
        }

        // start signal handler so threads can exit cleanly
        printf("initializing signal handler\n");
        if (rc_enable_signal_handler() < 0) {
            FAIL("ERROR: failed to complete rc_enable_signal_handler\n")
        }

        // start threads
        if (settings.enable_dsm)
        {
            printf("initializing DSM and input_manager\n");
            if (user_input.input_manager_init() < 0) {
                FAIL("ERROR: failed to initialize input_manager\n")
            }
        }        

        // initialize buttons and Assign functions to be called when button
        // events occur
        if (rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
            RC_BTN_DEBOUNCE_DEFAULT_US)) {
            FAIL("ERROR: failed to init buttons\n")
        }
        rc_button_set_callbacks(RC_BTN_PIN_PAUSE, on_pause_press, NULL);
    }

    // initialize log_manager if enabled in settings
    if (settings.enable_logging)
    {
        printf("initializing log manager\n");
        if (log_entry.init() < 0)
        {
            FAIL("ERROR: failed to initialize log manager\n")
        }
        if (!settings.log_only_while_armed)
        {
            if (log_entry.start() < 0)
            {
                FAIL("ERROR: failed to start log manager\n")
            }
        }
    }

    printf("initializing comms manager\n");
    if (comms_manager.init() < 0)
    {
        FAIL("ERROR: failed to initialize comms manager\n")
    }    

    if (RUNNING_ON_BBB)
    {
        // start barometer, must do before starting state estimator
        printf("initializing Barometer\n");
        if (rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16)) {
            FAIL("ERROR: failed to initialize barometer\n")
        }

        if (settings.enable_encoders) {
            //initializiation on counter
            printf("initializing revolution counter\n");
            if (rc_encoder_init() < 0) {
                FAIL("ERROR: failed to initialize encoder\n")
            }
        }

        // set up state estimator
        printf("initializing state_estimator\n");
        if (state_estimate.init() < 0) {
            FAIL("ERROR: failed to init state_estimator\n")
        }
    }

    // Initialize waypoint state machine
    printf("initializing waypoint state machine\n");
    if (waypoint_state_machine.init() < 0)
    {
        FAIL("ERROR: failed to init waypoint state machine\n");
    }

    if (RUNNING_ON_BBB)
    {
        // set up feedback controller
        printf("initializing feedback controller\n");
        if (fstate.init() < 0) {
            FAIL("ERROR: failed to init feedback controller\n")
        }

        // start the IMU0
        rc_mpu_config_t mpu_conf = rc_mpu_default_config();
        mpu_conf.i2c_bus = I2C_BUS;
        mpu_conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
        mpu_conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
        mpu_conf.dmp_sample_rate = FEEDBACK_HZ;
        mpu_conf.dmp_fetch_accel_gyro = 1;
        //mpu_conf.orient = ORIENTATION_Z_UP;
        mpu_conf.dmp_interrupt_sched_policy = SCHED_FIFO;
        mpu_conf.dmp_interrupt_priority = IMU_PRIORITY;

        // optionally enbale magnetometer
        mpu_conf.enable_magnetometer = settings.IMU0.compass.enable;

        // now set up the IMU0 for dmp interrupt operation
        printf("initializing MPU\n");
        if (rc_mpu_initialize_dmp(&mpu_data, mpu_conf)) {
            fprintf(stderr, "ERROR: failed to start MPU DMP\n");
            return -1;
        }        

        // final setup
        if (rc_make_pid_file() != 0) {
            FAIL("ERROR: failed to make a PID file\n")
        }

        // make sure everything is disarmed them start the ISR
        fstate.disarm();
        printf("waiting for dmp to settle...\n");
        fflush(stdout);
        rc_usleep(3000000);
        if (rc_mpu_set_dmp_callback(__imu_isr) != 0) {
            FAIL("ERROR: failed to set dmp callback function\n")
        }

        
    }
    // start printf_thread if running from a terminal
    // if it was started as a background process then don't bother	

    if (isatty(fileno(stdout))) {
        printf("initializing printf manager\n");
        if (printf_init() < 0) {
            FAIL("ERROR: failed to initialize printf_manager\n")
        }
    }

    // set state to running and chill until something exits the program
    rc_set_state(RUNNING);
    while (rc_get_state() != EXITING) {
        usleep(50000);
    }

    // some of these, like printf_manager and log_manager, have cleanup
    // functions that can be called even if not being used. So just call all
    // cleanup functions here.
    printf("Cleaning up\n");
    rc_mpu_power_off();
    fstate.cleanup();
    if(settings.enable_dsm) user_input.input_manager_cleanup();
    setpoint.cleanup();
    printf_cleanup();
    if (settings.enable_logging) log_entry.cleanup();
    rc_encoder_cleanup();
    path.cleanup();
    waypoint_state_machine.clean_up();
    if (sstate.is_initialized()) sstate.cleanup();
    comms_manager.cleanup();
    if (state_estimate.is_initialized()) state_estimate.cleanup();


    if (RUNNING_ON_BBB)
    {
        // turn off red LED and blink green to say shut down was safe
        rc_led_set(RC_LED_RED, 0);
        rc_led_blink(RC_LED_GREEN, 8.0, 2.0);
    }

    rc_remove_pid_file();   // remove pid file LAST
    return 0;
}