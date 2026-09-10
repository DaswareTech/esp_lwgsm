/**
 * @file esp_lwgsm_sntp_policy.h
 * @brief Step-vs-slew decision for applying an SNTP correction.
 *
 * Pure, dependency-free so it can be unit-tested on the host
 * (see test/host/test_sntp_policy.c).
 *
 * ESP-IDF adjtime() slews at 1/64 of real time: a correction of N seconds
 * takes 64*N seconds to complete, and adjtime() only refuses (forcing a step)
 * beyond ~35 minutes. After a soft reset the wall clock is re-derived from
 * the 150 kHz RC oscillator and is typically off by 1-2 % of the previous
 * uptime -- minutes, which would otherwise be slewed for hours. Only genuine
 * crystal drift between re-syncs (well under a second) is worth slewing.
 */

#ifndef ESP_LWGSM_SNTP_POLICY_H
#define ESP_LWGSM_SNTP_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/** Largest |correction| still applied gradually with adjtime(); anything
 *  beyond is stepped with settimeofday(). */
#define ESP_LWGSM_SNTP_SLEW_MAX_US   1000000LL   /* 1 second */

/**
 * @brief Decide whether an SNTP correction must be stepped instead of slewed.
 *
 * @param delta_us Correction to apply (server time - local time), microseconds
 * @return true to step the clock (settimeofday), false to slew (adjtime)
 */
static inline bool esp_lwgsm_sntp_should_step(int64_t delta_us)
{
    /* Compare without negating: -INT64_MIN overflows. */
    return delta_us > ESP_LWGSM_SNTP_SLEW_MAX_US || delta_us < -ESP_LWGSM_SNTP_SLEW_MAX_US;
}

#endif /* ESP_LWGSM_SNTP_POLICY_H */
