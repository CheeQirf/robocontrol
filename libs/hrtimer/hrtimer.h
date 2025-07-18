/*****************************************************************
 *     _   __             __   ____   _  __        __
 *    / | / /___   _  __ / /_ / __ \ (_)/ /____   / /_
 *   /  |/ // _ \ | |/_// __// /_/ // // // __ \ / __/
 *  / /|  //  __/_>  < / /_ / ____// // // /_/ // /_
 * /_/ |_/ \___//_/|_| \__//_/    /_//_/ \____/ \__/
 *
 * Copyright All Reserved © 2015-2024 NextPilot Development Team
 ******************************************************************/

/**
 * @file drv_hrt.h
 *
 * High-resolution timer with callouts and timekeeping.
 */
#ifndef __HRT_TIMER
#define __HRT_TIMER

#include <stdbool.h>
#include <inttypes.h>



typedef uint64_t hrt_abstime;


/**
 * Get absolute time in [us] (does not wrap).
 */
extern hrt_abstime hrt_absolute_time(void);

/**
 * Convert a timespec to absolute time.
 */
// static inline hrt_abstime ts_to_abstime(const struct timespec *ts) {
//     hrt_abstime result;

//     result  = (hrt_abstime)(ts->tv_sec) * 1000000;
//     result += (hrt_abstime)(ts->tv_nsec / 1000);

//     return result;
// }

// /**
//  * Convert absolute time to a timespec.
//  */
// static inline void abstime_to_ts(struct timespec *ts, hrt_abstime abstime) {
//     ts->tv_sec   = /*(typeof(ts->tv_sec))*/ (abstime / 1000000);
//     abstime     -= (hrt_abstime)(ts->tv_sec) * 1000000;
//     ts->tv_nsec  = /*(typeof(ts->tv_nsec))*/ (abstime * 1000);
// }

/**
 * Compute the delta between a timestamp taken in the past
 * and now.
 *
 * This function is not interrupt save.
 */
static inline hrt_abstime hrt_elapsed_time(const hrt_abstime *then) {
    return hrt_absolute_time() - *then;
}

int hrtimer_init(void);

/**
 * Store the absolute time in an interrupt-safe fashion.
 *
 * This function ensures that the timestamp cannot be seen half-written by an interrupt handler.
 */

#endif

