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

static const struct pmw3610_pointer_accel_curve precision_curve = {
    .base_gain_milli = 500,
    .takeoff_speed = 32,
    .full_speed = 160,
    .max_gain_milli = 3000,
    .reference_interval_ms = 15,
    .idle_reset_ms = 60,
    .precision_enabled = true,
    .precision_gain_milli = 333,
    .precision_speed = 16,
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

static void test_precision_region_preserves_medium_and_high_speed_curve(void) {
    uint32_t previous = (uint32_t)precision_curve.precision_gain_milli * 1000U;

    assert(pmw3610_pointer_accel_multiplier(&precision_curve, 0) == 333000);
    assert(pmw3610_pointer_accel_multiplier(&precision_curve, 16) == 333000);
    assert(pmw3610_pointer_accel_multiplier(&precision_curve, 17) == 334875);
    assert(pmw3610_pointer_accel_multiplier(&precision_curve, 20) == 359094);
    assert(pmw3610_pointer_accel_multiplier(&precision_curve, 24) == 416500);
    assert(pmw3610_pointer_accel_multiplier(&precision_curve, 28) == 473906);
    assert(pmw3610_pointer_accel_multiplier(&precision_curve, 31) == 498125);
    assert(pmw3610_pointer_accel_multiplier(&precision_curve, 32) == 500000);

    for (uint32_t speed = 1; speed <= 100000; speed++) {
        const uint32_t current = pmw3610_pointer_accel_multiplier(&precision_curve, speed);
        assert(current >= previous);
        assert(current <= (uint32_t)precision_curve.max_gain_milli * 1000U);
        if (speed >= precision_curve.takeoff_speed) {
            assert(current == pmw3610_pointer_accel_multiplier(&curve, speed));
        }
        previous = current;
    }
}

static void test_precision_fractional_motion_is_retained(void) {
    struct pmw3610_pointer_accel_state state;
    int32_t x;
    int32_t y;
    int32_t total;

    pmw3610_pointer_accel_reset(&state);
    total = 0;
    for (int i = 0; i < 1000; i++) {
        pmw3610_pointer_accel_apply_frame(&precision_curve, &state, 1, 0,
                                          1000 + 15 * i, 15, &x, &y);
        total += x;
        assert(y == 0);
    }
    assert(total == 333);

    pmw3610_pointer_accel_reset(&state);
    total = 0;
    for (int i = 0; i < 1000; i++) {
        pmw3610_pointer_accel_apply_frame(&precision_curve, &state, -1, 0,
                                          20000 + 15 * i, 15, &x, &y);
        total += x;
        assert(y == 0);
    }
    assert(total == -333);
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

static void test_all_quadrants_preserve_sign(void) {
    static const int16_t inputs[][2] = {
        {60, 80},
        {-60, 80},
        {-60, -80},
        {60, -80},
    };

    for (size_t i = 0; i < sizeof(inputs) / sizeof(inputs[0]); i++) {
        struct pmw3610_pointer_accel_state state;
        int32_t x;
        int32_t y;

        pmw3610_pointer_accel_reset(&state);
        pmw3610_pointer_accel_apply_frame(&curve, &state, inputs[i][0], inputs[i][1],
                                          1000, 15, &x, &y);
        assert((x > 0) == (inputs[i][0] > 0));
        assert((y > 0) == (inputs[i][1] > 0));
    }
}

static void test_direction_change_drops_opposite_remainder(void) {
    struct pmw3610_pointer_accel_state state;
    int32_t x;
    int32_t y;

    pmw3610_pointer_accel_reset(&state);
    pmw3610_pointer_accel_apply_frame(&curve, &state, 1, 0, 1000, 15, &x, &y);
    assert(x == 0 && y == 0);
    pmw3610_pointer_accel_apply_frame(&curve, &state, -2, 0, 1015, 15, &x, &y);
    assert(x == -1 && y == 0);
}

int main(void) {
    test_vector_and_time_normalization();
    test_curve_is_monotonic_and_bounded();
    test_precision_region_preserves_medium_and_high_speed_curve();
    test_precision_fractional_motion_is_retained();
    test_fractional_motion_and_same_frame_direction();
    test_delayed_backlog_is_not_classed_as_fast();
    test_all_quadrants_preserve_sign();
    test_direction_change_drops_opposite_remainder();
    puts("pointer_acceleration_test: PASS");
    return 0;
}
