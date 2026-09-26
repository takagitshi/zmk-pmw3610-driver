/*
 * Copyright (c) 2026 Takashi Imai
 *
 * SPDX-License-Identifier: MIT
 */

#include <limits.h>
#include <stdint.h>

#include <pmw3610/pointer_acceleration.h>

static uint32_t integer_sqrt(uint64_t value) {
    uint64_t remainder = value;
    uint64_t root = 0;
    uint64_t bit = UINT64_C(1) << 62;

    while (bit > remainder) {
        bit >>= 2;
    }

    while (bit != 0) {
        if (remainder >= root + bit) {
            remainder -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }

    return root > UINT32_MAX ? UINT32_MAX : (uint32_t)root;
}

uint32_t pmw3610_pointer_accel_vector_speed(int32_t x, int32_t y) {
    const int64_t x64 = x;
    const int64_t y64 = y;
    const uint64_t abs_x = x64 < 0 ? (uint64_t)-x64 : (uint64_t)x64;
    const uint64_t abs_y = y64 < 0 ? (uint64_t)-y64 : (uint64_t)y64;

    return integer_sqrt((abs_x * abs_x) + (abs_y * abs_y));
}

uint32_t pmw3610_pointer_accel_normalize_speed(uint32_t speed, uint16_t reference_interval_ms,
                                               int64_t elapsed_ms) {
    if (elapsed_ms <= 0 || elapsed_ms <= reference_interval_ms) {
        return speed;
    }

    const uint64_t normalized =
        ((uint64_t)speed * reference_interval_ms + (uint64_t)elapsed_ms / 2U) /
        (uint64_t)elapsed_ms;
    return normalized > UINT32_MAX ? UINT32_MAX : (uint32_t)normalized;
}

static uint64_t integrated_gain_contribution_milli(uint32_t gain_span,
                                                   uint32_t transition_width,
                                                   uint32_t offset) {
    const uint64_t width = transition_width;
    const uint64_t x = offset;
    const uint64_t numerator = x * x * x * (2U * width - x);
    const uint64_t denominator = 2U * width * width * width;
    const uint64_t quotient = numerator / denominator;
    const uint64_t remainder = numerator % denominator;

    return (uint64_t)gain_span * quotient +
           ((uint64_t)gain_span * remainder + denominator / 2U) / denominator;
}

static uint32_t standard_multiplier(const struct pmw3610_pointer_accel_curve *curve,
                                    uint32_t speed) {
    if (speed == 0 || speed <= curve->takeoff_speed) {
        return (uint32_t)curve->base_gain_milli * 1000U;
    }

    const uint32_t gain_span = curve->max_gain_milli - curve->base_gain_milli;
    uint64_t output_milli;

    if (speed < curve->full_speed) {
        const uint32_t transition_width = curve->full_speed - curve->takeoff_speed;
        const uint32_t offset = speed - curve->takeoff_speed;

        output_milli = (uint64_t)curve->base_gain_milli * speed +
                       integrated_gain_contribution_milli(gain_span, transition_width, offset);
    } else {
        const uint32_t transition_width = curve->full_speed - curve->takeoff_speed;
        const uint64_t output_at_full_milli =
            (uint64_t)curve->base_gain_milli * curve->full_speed +
            ((uint64_t)gain_span * transition_width) / 2U;
        output_milli = output_at_full_milli +
                       (uint64_t)curve->max_gain_milli * (speed - curve->full_speed);
    }

    const uint64_t multiplier = (output_milli * 1000U + speed / 2U) / speed;
    const uint32_t maximum = (uint32_t)curve->max_gain_milli * 1000U;
    return multiplier > maximum ? maximum : (uint32_t)multiplier;
}

static uint32_t smoothstep_q16(uint32_t offset, uint32_t width) {
    const uint32_t one_q16 = UINT32_C(1) << 16;
    const uint32_t progress_q16 = (uint32_t)(((uint64_t)offset << 16) / width);
    const uint64_t squared_q32 = (uint64_t)progress_q16 * progress_q16;
    const uint32_t slope_q16 = 3U * one_q16 - 2U * progress_q16;

    return (uint32_t)((squared_q32 * slope_q16 + (UINT64_C(1) << 31)) >> 32);
}

uint32_t pmw3610_pointer_accel_multiplier(const struct pmw3610_pointer_accel_curve *curve,
                                          uint32_t speed) {
    const uint32_t standard = standard_multiplier(curve, speed);

    if (!curve->precision_enabled || speed >= curve->takeoff_speed) {
        return standard;
    }

    const uint32_t precision = (uint32_t)curve->precision_gain_milli * 1000U;
    if (speed <= curve->precision_speed) {
        return precision;
    }

    const uint32_t width = curve->takeoff_speed - curve->precision_speed;
    const uint32_t offset = speed - curve->precision_speed;
    const uint32_t blend_q16 = smoothstep_q16(offset, width);
    const uint32_t delta = standard - precision;

    return precision +
           (uint32_t)(((uint64_t)delta * blend_q16 + (UINT64_C(1) << 15)) >> 16);
}

static void reset_axis(struct pmw3610_pointer_accel_axis_state *axis) {
    axis->remainder = 0;
}

void pmw3610_pointer_accel_reset(struct pmw3610_pointer_accel_state *state) {
    reset_axis(&state->x);
    reset_axis(&state->y);
    state->last_frame_time_ms = 0;
    state->have_frame_time = false;
}

static int32_t scale_axis(struct pmw3610_pointer_accel_axis_state *state, int32_t value,
                          uint32_t multiplier) {
    if (value == 0) {
        return 0;
    }

    if ((value > 0 && state->remainder < 0) || (value < 0 && state->remainder > 0)) {
        state->remainder = 0;
    }

    const int64_t scaled_units = (int64_t)value * multiplier + state->remainder;
    const int64_t scaled = scaled_units / PMW3610_POINTER_ACCEL_GAIN_ONE;
    state->remainder = (int32_t)(scaled_units - scaled * PMW3610_POINTER_ACCEL_GAIN_ONE);

    if (scaled > INT32_MAX) {
        state->remainder = 0;
        return INT32_MAX;
    }
    if (scaled < INT32_MIN) {
        state->remainder = 0;
        return INT32_MIN;
    }
    return (int32_t)scaled;
}

void pmw3610_pointer_accel_apply_frame(const struct pmw3610_pointer_accel_curve *curve,
                                       struct pmw3610_pointer_accel_state *state, int32_t x,
                                       int32_t y, int64_t now_ms, int64_t elapsed_ms,
                                       int32_t *out_x, int32_t *out_y) {
    uint32_t speed = pmw3610_pointer_accel_vector_speed(x, y);

    if (state->have_frame_time) {
        const int64_t frame_gap_ms = now_ms - state->last_frame_time_ms;
        if (frame_gap_ms <= 0 || frame_gap_ms >= curve->idle_reset_ms) {
            reset_axis(&state->x);
            reset_axis(&state->y);
        }
    }
    speed = pmw3610_pointer_accel_normalize_speed(speed, curve->reference_interval_ms,
                                                  elapsed_ms);

    const uint32_t multiplier = pmw3610_pointer_accel_multiplier(curve, speed);
    *out_x = scale_axis(&state->x, x, multiplier);
    *out_y = scale_axis(&state->y, y, multiplier);
    state->last_frame_time_ms = now_ms;
    state->have_frame_time = true;
}
