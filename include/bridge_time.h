#ifndef BRIDGE_TIME_H
#define BRIDGE_TIME_H

#include <zephyr/kernel.h>

/**
 * @brief Free-running microsecond clock used to timestamp CAN frames.
 *
 * Wraps after about 71 minutes, so always compare values with
 * bridge_us_diff() instead of the relational operators.
 */
static inline uint32_t bridge_now_us(void)
{
	return k_cyc_to_us_floor32(k_cycle_get_32());
}

/** Signed difference a - b of two bridge_now_us() values. */
static inline int32_t bridge_us_diff(uint32_t a, uint32_t b)
{
	return (int32_t)(a - b);
}

#endif
