/**
 * @file tools.c
 */
#include <tools.h>

double finddt_s(uint64_t ti)
{
    double dt_s = (rc_nanos_since_boot() - ti) / (1e9);
    return dt_s;
}