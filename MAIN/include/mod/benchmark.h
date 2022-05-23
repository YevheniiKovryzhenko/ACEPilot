/**
 * <benchmark.h>
 *
 * @brief       Structure for timing portions of the software.
 *              Recommended to use rc_nanos_since_boot() for timers for consistency with other
 *              timestamps throughout rc_pilot. 
 *
 * 
 * @author     Prince Kuevor 
 * @date       02/23/2020 (MM/DD/YYYY)
 * 
 * @Modified    Yevhenii Kovryzhenko
 * @Last Edit   05/23/2022 (MM/DD/YYYY)
 * 
 * @addtogroup  BENCHMARK
 * @{
 */

#ifndef __BENCHMARK__
#define __BENCHMARK__

#include <inttypes.h>
#include <time.h>	// for timespec

/**
 * @brief   User-specified timers for benchmarking rc_pilot functionality
 */
typedef struct benchmark_t{
    /** @name Timers for imu_isr() */
	///@{
    uint64_t tIMU_END, tSM, tCOMMS, tMOCAP, tGPS, tPNI, tNAV, tGUI, tCTR, tLOG, tNTP;
    ///@}
} benchmark_t;

extern benchmark_t benchmark_timers;


#endif //__BENCHMARK__