/* SPDX-License-Identifier: MIT */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include <pmw3610/pointer_acceleration.h>

static const struct pmw3610_pointer_accel_curve curve = {
    .base_gain_milli = 500,
    .takeoff_speed = 32,
    .full_speed = 160,
    .max_gain_milli = 3000,
    .reference_interval_ms = 15,
    .idle_reset_ms = 60,
};

static void test_vector_and_time_normalization(void) {
    assert(pmw3610_pointer_accel_vector_speed(3, 4) == 5);
    assert(pmw3610_pointer_accel_vector_speed(-30, -40) == 50);
    assert(pmw3610_pointer_accel_normalize_speed(40, 15, 1) == 40);
    assert(pmw3610_pointer_accel_normalize_speed(40, 15, 15) == 40);
    assert(pmw3610_pointer_accel_normalize_speed(80, 15, 30) == 40);
}

static void test_curve_is_monotonic_and_bounded(void) {
    uint32_t previous = (uint32_t)curve.base_gain_milli * 1000U;

    assert(pmw3610_pointer_accel_multiplier(&curve, 0) == 500000);
    assert(pmw3610_pointer_accel_multiplier(&curve, 32) == 500000);
    for (uint32_t speed = 1; speed <= 100000; speed++) {
        const uint32_t current = pmw3610_pointer_accel_multiplier(&curve, speed);
        assert(current >= previous);
        assert(current <= (uint32_t)curve.max_gain_milli * 1000U);
        previous = current;
    }
}

static void test_fractional_motion_and_same_frame_direction(void) {
    struct pmw3610_pointer_accel_state state;
    int32_t x;
    int32_t y;

    pmw3610_pointer_accel_reset(&state);
    pmw3610_pointer_accel_apply_frame(&curve, &state, 1, 0, 1000, 15, &x, &y);
    assert(x == 0 && y == 0);
    pmw3610_pointer_accel_apply_frame(&curve, &state, 1, 0, 1015, 15, &x, &y);
    assert(x == 1 && y == 0);

    pmw3610_pointer_accel_reset(&state);
    pmw3610_pointer_accel_apply_frame(&curve, &state, 60, 80, 1000, 15, &x, &y);
    assert(x > 0 && y > 0);
    assert(x * 4 - y * 3 >= -4 && x * 4 - y * 3 <= 4);
}

static void test_delayed_backlog_is_not_classed_as_fast(void) {
    struct pmw3610_pointer_accel_state state;
    int32_t x;
    int32_t y;

    pmw3610_pointer_accel_reset(&state);
    pmw3610_pointer_accel_apply_frame(&curve, &state, 320, 0, 1075, 75, &x, &y);
    assert(x < 320);
}

int main(void) {
    test_vector_and_time_normalization();
    test_curve_is_monotonic_and_bounded();
    test_fractional_motion_and_same_frame_direction();
    test_delayed_backlog_is_not_classed_as_fast();
    puts("pointer_acceleration_test: PASS");
    return 0;
}
