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
    .precision_full_speed = 32,
};

static const struct pmw3610_pointer_accel_curve legacy_precision_curve = {
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

static const struct pmw3610_pointer_accel_curve lism_precision_curve = {
    .base_gain_milli = 1000,
    .takeoff_speed = 20,
    .full_speed = 102,
    .max_gain_milli = 3000,
    .reference_interval_ms = 8,
    .idle_reset_ms = 60,
    .precision_enabled = true,
    .precision_gain_milli = 658,
    .precision_speed = 8,
    .precision_full_speed = 17,
};

static const struct pmw3610_pointer_accel_curve lism_standard_curve = {
    .base_gain_milli = 1000,
    .takeoff_speed = 20,
    .full_speed = 102,
    .max_gain_milli = 3000,
    .reference_interval_ms = 8,
    .idle_reset_ms = 60,
};

static const struct pmw3610_pointer_accel_curve mona_precision_curve = {
    .base_gain_milli = 1000,
    .takeoff_speed = 37,
    .full_speed = 189,
    .max_gain_milli = 3000,
    .reference_interval_ms = 15,
    .idle_reset_ms = 60,
    .precision_enabled = true,
    .precision_gain_milli = 658,
    .precision_speed = 15,
    .precision_full_speed = 31,
};

static const struct pmw3610_pointer_accel_curve mona_standard_curve = {
    .base_gain_milli = 1000,
    .takeoff_speed = 37,
    .full_speed = 189,
    .max_gain_milli = 3000,
    .reference_interval_ms = 15,
    .idle_reset_ms = 60,
};

static const struct pmw3610_pointer_accel_curve wide_precision_curve = {
    .base_gain_milli = 1000,
    .takeoff_speed = 65534,
    .full_speed = 65535,
    .max_gain_milli = 3000,
    .reference_interval_ms = 8,
    .idle_reset_ms = 60,
    .precision_enabled = true,
    .precision_gain_milli = 658,
    .precision_speed = 0,
    .precision_full_speed = 65534,
};

static uint64_t output_speed_units(const struct pmw3610_pointer_accel_curve *test_curve,
                                   uint32_t speed) {
    return (uint64_t)pmw3610_pointer_accel_multiplier(test_curve, speed) * speed;
}

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

static void test_legacy_precision_initializer_uses_takeoff_endpoint(void) {
    for (uint32_t speed = 0; speed <= 100000; speed++) {
        assert(pmw3610_pointer_accel_multiplier(&legacy_precision_curve, speed) ==
               pmw3610_pointer_accel_multiplier(&precision_curve, speed));
    }
}

static void test_lism_precision_stage_and_legacy_identity(void) {
    static const uint32_t expected_transition[] = {
        658000, 669726, 701157, 746662, 800617,
        857378, 911327, 956838, 988269, 1000000,
    };

    assert(pmw3610_pointer_accel_multiplier(&lism_precision_curve, 0) == 658000);
    for (size_t i = 0; i < sizeof(expected_transition) / sizeof(expected_transition[0]); i++) {
        assert(pmw3610_pointer_accel_multiplier(&lism_precision_curve, 8U + i) ==
               expected_transition[i]);
    }

    for (uint32_t speed = lism_precision_curve.precision_full_speed;
         speed <= 100000; speed++) {
        assert(pmw3610_pointer_accel_multiplier(&lism_precision_curve, speed) ==
               pmw3610_pointer_accel_multiplier(&lism_standard_curve, speed));
    }

    struct pmw3610_pointer_accel_curve disabled = lism_precision_curve;
    disabled.precision_enabled = false;
    for (uint32_t speed = 0; speed <= 100000; speed++) {
        assert(pmw3610_pointer_accel_multiplier(&disabled, speed) ==
               pmw3610_pointer_accel_multiplier(&lism_standard_curve, speed));
    }
}

static void test_wide_precision_transition_is_exhaustively_monotonic(void) {
    uint32_t previous_multiplier =
        pmw3610_pointer_accel_multiplier(&wide_precision_curve, 0);
    uint64_t previous_output = output_speed_units(&wide_precision_curve, 0);

    assert(previous_multiplier == 658000);
    for (uint32_t speed = 1;
         speed <= wide_precision_curve.precision_full_speed; speed++) {
        const uint32_t multiplier =
            pmw3610_pointer_accel_multiplier(&wide_precision_curve, speed);
        const uint64_t output = output_speed_units(&wide_precision_curve, speed);

        assert(multiplier >= previous_multiplier);
        assert(multiplier <= PMW3610_POINTER_ACCEL_GAIN_ONE);
        assert(output >= previous_output);
        previous_multiplier = multiplier;
        previous_output = output;
    }
    assert(previous_multiplier == PMW3610_POINTER_ACCEL_GAIN_ONE);
}

static void test_mona_scaled_precision_curve(void) {
    assert(pmw3610_pointer_accel_multiplier(&mona_precision_curve, 0) == 658000);
    assert(pmw3610_pointer_accel_multiplier(&mona_precision_curve, 15) == 658000);
    assert(pmw3610_pointer_accel_multiplier(&mona_precision_curve, 16) == 661841);
    assert(pmw3610_pointer_accel_multiplier(&mona_precision_curve, 23) == 829000);
    assert(pmw3610_pointer_accel_multiplier(&mona_precision_curve, 30) == 996159);
    assert(pmw3610_pointer_accel_multiplier(&mona_precision_curve, 31) == 1000000);

    uint32_t previous_multiplier =
        pmw3610_pointer_accel_multiplier(&mona_precision_curve, 0);
    uint64_t previous_output = output_speed_units(&mona_precision_curve, 0);
    for (uint32_t speed = 1; speed <= 100000; speed++) {
        const uint32_t multiplier =
            pmw3610_pointer_accel_multiplier(&mona_precision_curve, speed);
        const uint64_t output = output_speed_units(&mona_precision_curve, speed);

        assert(multiplier >= previous_multiplier);
        assert(multiplier <= (uint32_t)mona_precision_curve.max_gain_milli * 1000U);
        assert(output >= previous_output);
        if (speed >= mona_precision_curve.precision_full_speed) {
            assert(multiplier ==
                   pmw3610_pointer_accel_multiplier(&mona_standard_curve, speed));
        }
        previous_multiplier = multiplier;
        previous_output = output;
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
    test_legacy_precision_initializer_uses_takeoff_endpoint();
    test_lism_precision_stage_and_legacy_identity();
    test_wide_precision_transition_is_exhaustively_monotonic();
    test_mona_scaled_precision_curve();
    test_precision_fractional_motion_is_retained();
    test_fractional_motion_and_same_frame_direction();
    test_delayed_backlog_is_not_classed_as_fast();
    test_all_quadrants_preserve_sign();
    test_direction_change_drops_opposite_remainder();
    puts("pointer_acceleration_test: PASS");
    return 0;
}
