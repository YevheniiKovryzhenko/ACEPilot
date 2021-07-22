#pragma once
#include <rc/time.h> // for nanos
#include <inttypes.h> // for PRIu64
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief		finds time elapsed in seconds
 *
 * @return		time in seconds
 */
double finddt_s(uint64_t ti);

#ifdef __cplusplus
}
#endif /* __cplusplus */

