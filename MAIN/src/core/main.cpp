/*
 * main.cpp
 *
 * Copyright Yevhenii Kovryzhenko, Department of Aerospace Engineering, Auburn University.
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
 */

#include "main.hpp"

using namespace std;
struct stat buffer;
//check if running on BBB:
bool RUNNING_ON_BBB = (stat("/sys/devices/platform/leds/leds/", &buffer) == 0 && S_ISDIR(buffer.st_mode)); 


//uint64_t t_init     = rc_nanos_since_boot(); //log start time

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
 * @brief      Interrupt service routine for IMU
 *
 * This is called every time the Invensense IMU has new data
 *
 * __imu_isr should be running at 100 Hz (YK)
 */

static void __imu_isr(void)
{
    static uint64_t temp_time;
    static double temp_dt_s;

    int i = 0;
    //printf("imu interupt...\n");
    setpoint_manager_update();
    state_estimator_march();
    if (settings.enable_xbee) {
        XBEE_getData();
    }
    // Update the state machine
    if (user_input.flight_mode == AUTONOMOUS)
    {
        sm_transition(&waypoint_state_machine, (sm_alphabet)xbeeMsg.sm_event);
    }

    //printf("\n state = %d", xbeeMsg.sm_event);
    feedback_march();

    // we are not using encoders 
    if (settings.enable_encoders) {
        for (i = 1; i < 5; i++) {
            state_estimate.rev[i - 1] = rc_encoder_read(i);
        };
    }
    if (settings.enable_logging) log_manager_add_new();
    state_estimator_jobs_after_feedback();

    if (settings.delay_warnings_en)
    {
        temp_dt_s = (rc_nanos_since_boot() - temp_time) / (1e9); //calculate time since last successful reading
        if (1 / temp_dt_s < 90) printf("\nWARNING, Low update frequency of __imu_isr %f (Hz)\n", 1 / temp_dt_s); //check the update frequency
        temp_time = rc_nanos_since_boot();
    }
}


/**
 * Initialize the IMU, start all the threads, and wait until something triggers
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
            printf("\nInvalid Argument \n");
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
            fprintf(stderr, "\nERROR in main(), failed to set RC_LED_GREEN");
            return -1;
        }
        if (rc_led_set(RC_LED_RED, 0) == -1)
        {
            fprintf(stderr, "\nERROR in main() failed to set RC_LED_RED");
            return -1;
        }

        // make sure IMU is calibrated
        if (!rc_mpu_is_gyro_calibrated()) {
            FAIL("ERROR, must calibrate gyroscope with rc_calibrate_gyro first\n")
        }
        if (!rc_mpu_is_accel_calibrated()) {
            FAIL("ERROR, must calibrate accelerometer with rc_calibrate_accel first\n")
        }
        if (settings.enable_magnetometer && !rc_mpu_is_gyro_calibrated()) {
            FAIL("ERROR, must calibrate magnetometer with rc_calibrate_mag first\n")
        }
        if (!__rc_dsm_is_calibrated()) {
            FAIL("ERROR, must calibrate DSM with rc_calibrate_dsm first\n")
        }

        // turn cpu freq to max for most consistent performance and lowest
        // latency servicing the IMU's interrupt service routine
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
    printf("initializing mixing matrix\n");
    if (mix_init(settings.layout) < 0) {
        FAIL("ERROR: failed to initialize mixing matrix\n")
    }
    printf("initializing setpoint_manager\n");
    if (setpoint_manager_init() < 0) {
        FAIL("ERROR: failed to initialize setpoint_manager\n")
    }

    // initialize cape hardware, this prints an error itself if unsuccessful
    if (RUNNING_ON_BBB)
    {
        printf("initializing servos\n");
        if (rc_servo_init() == -1) {
            FAIL("ERROR: failed to initialize servos, probably need to run as root\n")
        }

        if (ss.init(1, 0x40) == -1)
        {
            printf("\nERROR: failed to initialize servos");
            return -1;
        }

        printf("initializing adc\n");
        if (rc_adc_init() == -1) {
            FAIL("ERROR: failed to initialize ADC")
        }

        // start signal handler so threads can exit cleanly
        printf("initializing signal handler\n");
        if (rc_enable_signal_handler() < 0) {
            FAIL("ERROR: failed to complete rc_enable_signal_handler\n")
        }

        // start threads
        printf("initializing DSM and input_manager\n");
        if (input_manager_init() < 0) {
            FAIL("ERROR: failed to initialize input_manager\n")
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
    if (settings.enable_logging) {
        printf("initializing log manager\n");
        if (log_manager_init() < 0) {
            FAIL("ERROR: failed to initialize log manager\n")
        }
    }

    if (RUNNING_ON_BBB)
    {
        // start barometer, must do before starting state estimator
        printf("initializing Barometer\n");
        if (rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16)) {
            FAIL("ERROR: failed to initialize barometer\n")
        }

        if (settings.enable_encoders) {
            //B: initializiation on counter
            printf("initializing revolution counter\n");
            if (rc_encoder_init() < 0) {
                FAIL("ERROR: failed to initialize encoder\n")
            }
        }

        // set up state estimator
        printf("initializing state_estimator\n");
        if (state_estimator_init() < 0) {
            FAIL("ERROR: failed to init state_estimator")
        }
    }

    // set up XBEE serial link
    if (settings.enable_xbee) {
        printf("initializing xbee serial link.\n");
        if (XBEE_init(settings.serial_port_1, settings.serial_baud_1) < 0)
        {
            FAIL("ERROR: failed to init xbee serial link")
        }
    }

    // Initialize waypoint state machine
    printf("initializing waypoint state machine\n");
    if (sm_init(&waypoint_state_machine) < 0)
    {
        FAIL("ERROR: failed to init waypoint state machine");
    }

    if (RUNNING_ON_BBB)
    {
        // set up feedback controller
        printf("initializing feedback controller\n");
        if (feedback_init() < 0) {
            FAIL("ERROR: failed to init feedback controller")
        }

    }
    
    if (RUNNING_ON_BBB)
    {
        // start the IMU
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
        mpu_conf.enable_magnetometer = settings.enable_magnetometer;

        // now set up the imu for dmp interrupt operation
        printf("initializing MPU\n");
        if (rc_mpu_initialize_dmp(&mpu_data, mpu_conf)) {
            fprintf(stderr, "ERROR: failed to start MPU DMP\n");
            return -1;
        }

        if (ss.test_min_max() == -1)
        {
            printf("\nERROR: failed to run servo test");
            return -1;
        }

        // final setup
        if (rc_make_pid_file() != 0) {
            FAIL("ERROR: failed to make a PID file\n")
        }

        // make sure everything is disarmed them start the ISR
        feedback_disarm();
        printf("waiting for dmp to settle...\n");
        fflush(stdout);
        rc_usleep(3000000);
        if (rc_mpu_set_dmp_callback(__imu_isr) != 0) {
            FAIL("ERROR: failed to set dmp callback function\n")
        }

        // start printf_thread if running from a terminal
        // if it was started as a background process then don't bother	

        if (isatty(fileno(stdout))) {
            printf("initializing printf manager\n");
            if (printf_init() < 0) {
                FAIL("ERROR: failed to initialize printf_manager\n")
            }
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
    printf("cleaning up\n");
    rc_mpu_power_off();
    feedback_cleanup();
    input_manager_cleanup();
    setpoint_manager_cleanup();
    printf_cleanup();
    log_manager_cleanup();
    rc_encoder_cleanup();
    ss.cleanup();


    if (RUNNING_ON_BBB)
    {
        // turn off red LED and blink green to say shut down was safe
        rc_led_set(RC_LED_RED, 0);
        rc_led_blink(RC_LED_GREEN, 8.0, 2.0);
    }

    /* Init GPS */
    /*
    printf("\nInitializing gps serial link");
    int portID = gps_init("/dev/ttyS11", 57600);
    if (RUNNING_ON_BBB == 1)
    {
        if (portID == -1) FAIL("\nTerminating...");
    }
    else
    {
        if (portID == -1) return -1;
    }

    //GPS.init_GPS(portID);
    //GPS.send_all();
    */
    /*
    if (RUNNING_ON_BBB)
    {
        if (ss.init(1, 0x40) == -1)
        {
            printf("\nERROR: failed to initialize servos");
            return -1;
        }
    }
    
    if (RUNNING_ON_BBB)
    {
        if (ss.test_min_max() == -1)
        {
            printf("\nERROR: failed to run servo test");
            return -1;
        }
    }

    
    while (finddt_s(t_init) <= 20.0)
    {
        //gps_getData();
        
    }
    */
    /*
    if (RUNNING_ON_BBB)
    {
        ss.cleanup();
    }
    */
    //gps_cleanup();
    return 0;
}