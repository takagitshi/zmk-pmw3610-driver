/*
 * Copyright (c) 2026 Takashi Imai
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PMW3610_POINTER_ACCEL_GAIN_ONE 1000000U

struct pmw3610_pointer_accel_curve {
    uint16_t base_gain_milli;
    uint16_t takeoff_speed;
    uint16_t full_speed;
    uint16_t max_gain_milli;
    uint16_t reference_interval_ms;
    uint16_t idle_reset_ms;
    bool precision_enabled;
    uint16_t precision_gain_milli;
    uint16_t precision_speed;
};

struct pmw3610_pointer_accel_axis_state {
    int32_t remainder;
};

struct pmw3610_pointer_accel_state {
    struct pmw3610_pointer_accel_axis_state x;
    struct pmw3610_pointer_accel_axis_state y;
    int64_t last_frame_time_ms;
    bool have_frame_time;
};

uint32_t pmw3610_pointer_accel_vector_speed(int32_t x, int32_t y);

uint32_t pmw3610_pointer_accel_normalize_speed(uint32_t speed, uint16_t reference_interval_ms,
                                               int64_t elapsed_ms);

uint32_t pmw3610_pointer_accel_multiplier(const struct pmw3610_pointer_accel_curve *curve,
                                          uint32_t speed);

void pmw3610_pointer_accel_reset(struct pmw3610_pointer_accel_state *state);

void pmw3610_pointer_accel_apply_frame(const struct pmw3610_pointer_accel_curve *curve,
                                       struct pmw3610_pointer_accel_state *state, int32_t x,
                                       int32_t y, int64_t now_ms, int64_t elapsed_ms,
                                       int32_t *out_x, int32_t *out_y);
