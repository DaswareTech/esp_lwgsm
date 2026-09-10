/**
 * @file test_sntp_policy.c
 * @brief Host-compilable unit tests for the SNTP step-vs-slew policy.
 *
 * Build and run: cd components/esp_lwgsm/test/host && make test (gcc/clang).
 * No ESP-IDF required.
 *
 * Background: ESP-IDF adjtime() slews at 1/64 of real time, so a clock error
 * of N seconds takes 64*N seconds to disappear. After a soft reset the ESP32
 * re-derives elapsed time from the 150 kHz RC oscillator and the wall clock
 * is typically off by 1-2 % of the previous uptime (minutes). Slewing that
 * leaves every timestamp wrong for hours; such errors must be stepped.
 */

#include <stdint.h>
#include <stdio.h>

#include "../../include/apps/esp_lwgsm_sntp_policy.h"

static int tests_run = 0;
static int tests_passed = 0;

#define CHECK(name, cond)                                        \
    do {                                                         \
        tests_run++;                                             \
        printf("  [%d] %-62s ", tests_run, (name));              \
        if (cond) { tests_passed++; printf("PASS\n"); }          \
        else      { printf("FAIL\n"); }                          \
    } while (0)

#define US_PER_SEC 1000000LL

int main(void)
{
    printf("SNTP step-vs-slew policy tests\n");

    /* Sub-second errors are the normal crystal drift between re-syncs: slew. */
    CHECK("zero delta slews",                       !esp_lwgsm_sntp_should_step(0));
    CHECK("+500 ms slews",                          !esp_lwgsm_sntp_should_step(500000));
    CHECK("-500 ms slews",                          !esp_lwgsm_sntp_should_step(-500000));
    CHECK("exactly +1 s slews (threshold inclusive)", !esp_lwgsm_sntp_should_step(US_PER_SEC));
    CHECK("exactly -1 s slews (threshold inclusive)", !esp_lwgsm_sntp_should_step(-US_PER_SEC));

    /* Anything beyond one second is a reboot artifact or a first sync: step. */
    CHECK("+1 s + 1 us steps",                      esp_lwgsm_sntp_should_step(US_PER_SEC + 1));
    CHECK("-1 s - 1 us steps",                      esp_lwgsm_sntp_should_step(-US_PER_SEC - 1));
    CHECK("+20 min (post-reboot RC error) steps",   esp_lwgsm_sntp_should_step(20 * 60 * US_PER_SEC));
    CHECK("-20 min (post-reboot RC error) steps",   esp_lwgsm_sntp_should_step(-20 * 60 * US_PER_SEC));
    CHECK("+56 years (clock at 1970) steps",        esp_lwgsm_sntp_should_step(56LL * 365 * 24 * 3600 * US_PER_SEC));
    CHECK("INT64_MAX steps",                        esp_lwgsm_sntp_should_step(INT64_MAX));
    CHECK("INT64_MIN steps",                        esp_lwgsm_sntp_should_step(INT64_MIN));

    printf("\n%d/%d tests passed\n", tests_passed, tests_run);
    return tests_passed == tests_run ? 0 : 1;
}
