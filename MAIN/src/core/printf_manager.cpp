/*
 * printf_manager.cpp
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
 * Last Edit:  06/01/2022 (MM/DD/YYYY)
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>
#include <rc/encoder.h>
#include <signal.h>

#include "rc_pilot_defs.h"
#include "flight_mode.h"
#include "input_manager.hpp"
#include "setpoint_manager.hpp"
#include "feedback.hpp"
#include "state_estimator.h"
#include "thread_defs.h"
#include "settings.h"
#include "comms_tmp_data_packet.h"
#include "gps.h"

#include "printf_manager.hpp"

static pthread_t printf_manager_thread;
static int initialized = 0;

const char* const colours[] = {KYEL, KCYN, KGRN, KMAG};
const int num_colours = 4; // length of above array
int current_colour = 0;

/**
 * @brief      { function_description }
 *
 * @return     string with ascii colour code
 */
static const char* __next_colour()
{
	// if reached the end of the colour list, loop around
	if(current_colour>=(num_colours-1)){
		current_colour=0;
		return colours[num_colours-1];
	}
	// else increment counter and return
	current_colour++;
	return colours[current_colour-1];
}

static void __reset_colour()
{
	current_colour = 0;
}



static int __print_header()
{
	int i;

	printf("\n");
	__reset_colour();
	if(settings.printf_arm)
	{
		printf("  arm   |");
	}
	if (settings.printf_tracking)
	{
		printf("%s Sats | Tracking |", __next_colour(), gps_data.sat, GS_RX.trackingValid);
	}
	if(settings.printf_altitude)
	{
		printf("%s alt(m) |altdot|", __next_colour());
	}
	if (settings.printf_battery)
	{
		printf("%s batt_V|", __next_colour());
	}
	if(settings.printf_rpy)
	{
		printf("%s roll|pitch| yaw | yaw_c |", __next_colour());
	}
	if(settings.printf_sticks)
	{
		printf("%s  kill  | thr |roll |pitch| yaw |", __next_colour());
	}
	if (settings.printf_setpoint)
	{
		if (settings.printf_setpoint_xy)
		{
			printf("%s sp_x | sp_y |", __next_colour());
		}
		if (settings.printf_setpoint_z)
		{
			printf("%s sp_z |", __next_colour());
		}
		if (settings.printf_setpoint_xy_dot)
		{
			printf("%s +sp_xd| sp_yd|", __next_colour());
		}
		if (settings.printf_setpoint_z_dot)
		{
			printf("%s sp_zd|", __next_colour());
		}
		if (settings.printf_setpoint_att_dot)
		{
			printf("%s sp_r | sp_p | sp_y |", __next_colour());
		}
		if (settings.printf_setpoint_att)
		{
			printf("%s sp_rd| sp_pd| sp_yd|", __next_colour());
		}
	}
	if (settings.printf_u)
	{
		printf("%s U0X | U1Y | U2Z | U3r | U4p | U5y |", __next_colour());
	}
	if (settings.printf_mocap)
	{
		printf("%s x_mc | y_mc | z_mc | xdot_mc | ydot_mc | zdot_mc | qx_mc | qy_mc | qz_mc | qw_mc | sm_mc |", __next_colour());
	}
	if (settings.printf_gain_tunning)
	{
		printf("%s  gain_ch |", __next_colour());
	}
	if (settings.printf_gps)
	{
		printf("%sgps_lat|gps_lon|gps_ele|", __next_colour());
	}
	if (settings.printf_ext_mag)
	{
		printf("%ext_mag_x|ext_mag_y|ext_mag_z|vec_norm|", __next_colour());
	}
	if (settings.printf_motors)
	{
		printf("%s", __next_colour());
		for (i = 0; i < settings.num_rotors; i++)
		{
			printf("  M%d |", i + 1);
		}
	}
	if (settings.printf_int_mag)
	{
		printf("%s mag_x | mag_y | mag_z |mag_nrm|", __next_colour());
	}
 	if(settings.printf_rev){
 		printf("%s rev1 | rev2 | rev3 | rev4 ", __next_colour());
 	}
	printf(KNRM);
	if(settings.printf_mode){
		printf("   MODE ");
	}
	if(settings.printf_counter){
		printf(" counter ");
	}
	printf("\n");
	fflush(stdout);
	return 0;
}


char print_update_gains(void)
{
	/*
	Print update gain channel.
	*/
	if (!GS_RX.en_tunning || !settings.allow_remote_tuning)
	{
		printf("%s DISABLED |", __next_colour());
		return 0;
	}
	else
	{
		switch ((int)GS_RX.GainCH)
		{
		case 0:
			printf("%s  DEFAULT |", __next_colour());
			return 0;
		case 1: //roll
			printf("%s    Roll  |", __next_colour());
			return 0;

		case 2: //pitch
			printf("%s   Pitch  |", __next_colour());
			return 0;

		case 3: //yaw
			printf("%s    Yaw   |", __next_colour());
			return 0;

		case 4: //roll rate
			printf("%s Roll Rate|", __next_colour());
			return 0;

		case 5: //pitch rate
			printf("%sPitch Rate|", __next_colour());
			return 0;

		case 6: //yaw rate
			printf("%s Yaw Rate |", __next_colour());
			return 0;

		case 7: //x
			printf("%s    X     |", __next_colour());
			return 0;

		case 8: //y
			printf("%s    Y     |", __next_colour());
			return 0;

		case 9: //z
			printf("%s    Z     |", __next_colour());
			return 0;

		case 10: //x rate
			printf("%s  X Rate  |", __next_colour());
			return 0;

		case 11: //y rate
			printf("%s  Y Rate  |", __next_colour());
			return 0;

		case 12: //z rate
			printf("%s  Z Rate  |", __next_colour());
			return 0;

		default: //no changes
			printf("%s    NA    |", __next_colour());
			return 0;
		}
	}
	
	return 0;
}


static void* __printf_manager_func(__attribute__ ((unused)) void* ptr)
{
	int i;
	initialized = 1;
	printf("\nTurn your transmitter kill switch to arm.\n");
	printf("Then move throttle UP then DOWN to arm controller\n\n");

	// turn off linewrap to avoid runaway prints
	printf(WRAP_DISABLE);

	// print the header
	__print_header();

	//sleep so state_estimator can run first
	rc_usleep(100000);

	while(rc_get_state()!=EXITING)
	{
		__reset_colour();

		printf("\r");
		if(settings.printf_arm){
			if (fstate.get_arm_state() == ARMED)
			{
				printf("%s ARMED %s |", KRED, KNRM);
			}
			else
			{
				printf("%sDISARMED%s|", KGRN, KNRM);
			}
		}

		if (settings.printf_tracking)
		{
			printf("%s  %3i   |    %i     |", __next_colour(), gps_data.sat, GS_RX.trackingValid);
		}
		if (settings.printf_altitude)
		{
			printf("%s%+5.2f |%+5.2f |", __next_colour(), state_estimate.alt_bmp,
				state_estimate.alt_bmp_vel);
		}
		if (settings.printf_battery)
		{
			printf("%s%7.2f|", __next_colour(), state_estimate.v_batt_lp);
		}
		if (settings.printf_rpy)
		{
			printf(KCYN);
			printf("%s%+5.2f|%+5.2f|%+5.2f|%+7.2f|", __next_colour(), state_estimate.roll,
				state_estimate.pitch, state_estimate.yaw, state_estimate.continuous_yaw);
		}
		if(settings.printf_sticks){
			if (user_input.get_arm_switch() == ARMED)
			{
				printf("%s ARMED  ", KRED);
			}
			else
			{
				printf("%sDISARMED", KGRN);
			}
			printf(KGRN);
			printf("%s|%+5.2f|%+5.2f|%+5.2f|%+5.2f|",\
							__next_colour(),\
							user_input.throttle.get(),\
							user_input.roll.get(),\
							user_input.pitch.get(),\
							user_input.yaw.get());
		}
		if (settings.printf_setpoint)
		{
			if (settings.printf_setpoint_xy)
			{
				printf("%s%+6.2f|%+6.2f|", __next_colour(),
					setpoint.XY.x.value.get(), setpoint.XY.y.value.get());
			}
			if (settings.printf_setpoint_z)
			{
				printf("%s%+6.2f|", __next_colour(),
					setpoint.Z.value.get());
			}
			if (settings.printf_setpoint_xy_dot)
			{
				printf("%s%+6.2f|%+6.2f|", __next_colour(),
					setpoint.XY_dot.x.value.get(), setpoint.XY_dot.y.value.get());
			}
			if (settings.printf_setpoint_z_dot)
			{
				printf("%s%+6.2f|", __next_colour(),
					setpoint.Z_dot.value.get());
			}
			if (settings.printf_setpoint_att_dot)
			{
				printf("%s%+6.2f|%+6.2f|%+6.2f|", __next_colour(),
					setpoint.ATT_dot.x.value.get(), setpoint.ATT_dot.y.value.get(), setpoint.ATT_dot.z.value.get());
			}
			if (settings.printf_setpoint_att)
			{
				printf("%s%+6.2f|%+6.2f|%+6.2f|", __next_colour(),
					setpoint.ATT.x.value.get(), setpoint.ATT.y.value.get(), setpoint.ATT.z.value.get());
			}
		}
		if (settings.printf_u)
		{
			printf("%s%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|", __next_colour(), fstate.get_u(0),
				fstate.get_u(1), fstate.get_u(2), fstate.get_u(3), fstate.get_u(4), fstate.get_u(5));
		}
		if (settings.printf_mocap)
		{
			printf("%s%+6.2f|%+6.2f|%+6.2f|%+9.2f|%+9.2f|%+9.2f|%+7.2f|%+7.2f|%+7.2f|%+7.2f|  %3X  |", __next_colour(),
				state_estimate.pos_mocap[0], state_estimate.pos_mocap[1], state_estimate.pos_mocap[2], state_estimate.X_dot, state_estimate.Y_dot,
				state_estimate.Z_dot, state_estimate.quat_mocap[1], state_estimate.quat_mocap[2], state_estimate.quat_mocap[3], state_estimate.quat_mocap[0],
				GS_RX.sm_event);
		}
		if (settings.printf_gain_tunning)
		{
			print_update_gains();
		}
		if (settings.printf_gps)
		{
			printf("%s%+7.2f|%+7.2f|%+7.2f|", __next_colour(),
				gps_data.lla.lat, gps_data.lla.lon, gps_data.lla.alt);
		}
		if (settings.printf_ext_mag)
		{
			printf("%s%+8.3lf|%+8.3lf|%+8.3lf|%+8.3lf|", __next_colour(),
				ext_mag.x, ext_mag.y, ext_mag.z, ext_mag.norm);
		}
		if (settings.printf_motors)
		{
			printf("%s", __next_colour());
			for (i = 0; i < settings.num_rotors; i++)
			{
				printf("%+5.2f|", fstate.get_m(i));
			}
		}
		if (settings.printf_int_mag)
		{
			printf("%s%+7.2f|%+7.2f|%+7.2f|%+7.2f|", __next_colour(), state_estimate.mag[0],
				state_estimate.mag[1], state_estimate.mag[2],
				sqrt(pow(state_estimate.mag[0], 2) + pow(state_estimate.mag[1], 2) +
					pow(state_estimate.mag[2], 2)));
		}
		printf(KNRM);
		// we are not using encoders
 		if(settings.printf_rev){
			for(i=0;i<4;i++){
				printf("%10d|", state_estimate.rev[i]);
			}
 		}

		if(settings.printf_mode){
			print_flight_mode(user_input.get_flight_mode());
		}
		if(settings.printf_counter){
			printf("%d ",state_estimate.counter);
		}
		fflush(stdout);
		rc_usleep(1000000/PRINTF_MANAGER_HZ);
	}

	// put linewrap back on
	printf(WRAP_ENABLE);

	return NULL;
}

int printf_init()
{
	if(rc_pthread_create(&printf_manager_thread, __printf_manager_func, NULL,
				SCHED_FIFO, PRINTF_MANAGER_PRI)==-1){
		fprintf(stderr,"ERROR in start_printf_manager, failed to start thread\n");
		return -1;
	}
	rc_usleep(50000);
	return 0;
}


int printf_cleanup()
{
	int ret = 0;
	if(initialized){
		// wait for the thread to exit
		ret = rc_pthread_timed_join(printf_manager_thread,NULL,PRINTF_MANAGER_TOUT);
		if(ret==1) fprintf(stderr,"WARNING: printf_manager_thread exit timeout\n");
		else if(ret==-1) fprintf(stderr,"ERROR: failed to join printf_manager thread\n");
	}
	initialized = 0;
	return ret;
}


int print_flight_mode(flight_mode_t mode){
	switch(mode){
	case TEST_BENCH_4DOF:
		printf("%sTEST_BENCH_4DOF       %s", KYEL, KNRM);
		return 0;
	case TEST_BENCH_6DOF:
		printf("%sTEST_BENCH_6DOF       %s", KYEL, KNRM);
		return 0;
	case TEST_6xSERVOS_DIRECT:
		printf("%sTEST_6xSERVOS_DIRECT  %s", KYEL, KNRM);
		return 0;
	case DIRECT_THROTTLE_6DOF:
		printf("%sDIR_THRTLE_6DOF       %s", KCYN, KNRM);
		return 0;
	case ACRO_Axxxxx:
		printf("%sACRO_Axxxxx           %s", KCYN, KNRM);
		return 0;
	case ACRO_Fxxxxx:
		printf("%sACRO_Fxxxxx           %s", KCYN, KNRM);
		return 0;
	case MANUAL_xAxxxx:
		printf("%sMANUAL_xAxxxx         %s", KCYN, KNRM);
		return 0;
	case MANUAL_xFxxxx:
		printf("%sMANUAL_xFxxxx         %s", KCYN, KNRM);
		return 0;
	case MANUAL_AAxxxx:
		printf("%sMANUAL_AAxxxx         %s", KCYN, KNRM);
		return 0;
	case MANUAL_FAxxxx:
		printf("%sMANUAL_FAxxxx         %s", KCYN, KNRM);
		return 0;
	case MANUAL_FFxxxx:
		printf("%sMANUAL_FFxxxx         %s", KCYN, KNRM);
		return 0;
	case ALT_HOLD_AxAxxx:
		printf("%sALT_HOLD_AxAxxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FxAxxx:
		printf("%sALT_HOLD_FxAxxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FxFxxx:
		printf("%sALT_HOLD_FxFxxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_AxAAxx:
		printf("%sALT_HOLD_AxAAxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FxAAxx:
		printf("%sALT_HOLD_FxAAxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FxFAxx:
		printf("%sALT_HOLD_FxFAxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FxFFxx:
		printf("%sALT_HOLD_FxFFxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_xAxAxx:
		printf("%sALT_HOLD_xAxAxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_xFxAxx:
		printf("%sALT_HOLD_xFxAxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_xFxFxx:
		printf("%sALT_HOLD_xFxFxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_AAxAxx:
		printf("%sALT_HOLD_AAxAxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FAxAxx:
		printf("%sALT_HOLD_FAxAxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FFxAxx:
		printf("%sALT_HOLD_FFxAxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FFxFxx:
		printf("%sALT_HOLD_FFxFxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_AAAxxx:
		printf("%sALT_HOLD_AAAxxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FAAxxx:
		printf("%sALT_HOLD_FAAxxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FFAxxx:
		printf("%sALT_HOLD_FFAxxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FFFxxx:
		printf("%sALT_HOLD_FFFxxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_AAAAxx:
		printf("%sALT_HOLD_AAAAxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FAAAxx:
		printf("%sALT_HOLD_FAAAxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FFAAxx:
		printf("%sALT_HOLD_FFAAxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FFFAxx:
		printf("%sALT_HOLD_FFFAxx       %s", KBLU, KNRM);
		return 0;
	case ALT_HOLD_FFFFxx:
		printf("%sALT_HOLD_FFFFxx       %s", KBLU, KNRM);
		return 0;
	case POS_CTRL_AAAAAA:
		printf("%sPOS_CTRL_AAAAAA       %s", KBLU, KNRM);
		return 0;
	case POS_CTRL_FFFAAx:
		printf("%sPOS_CTRL_FFFAAx       %s", KBLU, KNRM);
		return 0;
	case POS_CTRL_FFFAFx:
		printf("%sPOS_CTRL_FFFAFx       %s", KBLU, KNRM);
		return 0;
	case POS_CTRL_FFFFAx:
		printf("%sPOS_CTRL_FFFFAx       %s", KBLU, KNRM);
		return 0;
	case POS_CTRL_FFFAAA:
		printf("%sPOS_CTRL_FFFAAA       %s", KBLU, KNRM);
		return 0;
	case POS_CTRL_FFFAFA:
		printf("%sPOS_CTRL_FFFAFA       %s", KBLU, KNRM);
		return 0;
	case POS_CTRL_FFFAFF:
		printf("%sPOS_CTRL_FFFAFF       %s", KBLU, KNRM);
		return 0;
	case POS_CTRL_FFFFFF:
		printf("%sPOS_CTRL_FFFFFF       %s", KBLU, KNRM);
		return 0;
	case EMERGENCY_LAND:
		printf("%sEMERGENCY_LAND        %s", KBLU, KNRM);
		return 0;
	case AUTO_FFFAFA:
		printf("%sAUTO_FFFAFA           %s", KBLU, KNRM);
		return 0;
	case ZEPPELIN:
		printf("%sZEPPELIN              %s", KBLU, KNRM);
		return 0;
	default:
		fprintf(stderr,"ERROR in print_flight_mode, unknown flight mode\n");
		return -1;
	}
}
