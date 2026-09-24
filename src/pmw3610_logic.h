/* SPDX-License-Identifier: MIT */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

#define PMW3610_READ_RETRY_MIN_MS 1
#define PMW3610_READ_RETRY_MAX_MS 64

struct pmw3610_report_accumulator {
    int64_t dx;
    int64_t dy;
    int64_t last_report_time;
    int64_t accumulation_start_time;
    bool have_last_report;
    bool have_accumulation_start;
    bool report_scheduled;
};

struct pmw3610_frame_retry {
    int16_t x;
    int16_t y;
    bool send_y;
};

struct pmw3610_frame_chunk {
    int16_t x;
    int16_t y;
};

struct pmw3610_motion_delta {
    int16_t x;
    int16_t y;
};

struct pmw3610_output_state {
    int32_t pending_x;
    int32_t pending_y;
    int16_t retry_x;
    int16_t retry_y;
    bool pending_force_sync;
    bool retry_pending;
    bool retry_force_sync;
};

struct pmw3610_output_frame {
    int16_t x;
    int16_t y;
    bool force_sync;
    bool retrying;
};

static inline int16_t pmw3610_decode_delta12(uint8_t low, uint8_t high_nibble) {
    uint16_t raw = (uint16_t)low | (((uint16_t)high_nibble & 0x0fU) << 8);

    return (raw & 0x0800U) != 0U ? (int16_t)(raw - 0x1000U) : (int16_t)raw;
}

static inline struct pmw3610_motion_delta
pmw3610_decode_motion_delta(uint8_t x_low, uint8_t y_low, uint8_t xy_high) {
    return (struct pmw3610_motion_delta){
        .x = pmw3610_decode_delta12(x_low, xy_high >> 4),
        .y = pmw3610_decode_delta12(y_low, xy_high),
    };
}

static inline struct pmw3610_frame_chunk pmw3610_frame_chunk_from_pending(int32_t x,
                                                                          int32_t y) {
    const int64_t abs_x = x < 0 ? -(int64_t)x : x;
    const int64_t abs_y = y < 0 ? -(int64_t)y : y;
    const int64_t maximum = abs_x > abs_y ? abs_x : abs_y;

    if (maximum <= INT16_MAX) {
        return (struct pmw3610_frame_chunk){.x = (int16_t)x, .y = (int16_t)y};
    }

    return (struct pmw3610_frame_chunk){
        .x = (int16_t)(((int64_t)x * INT16_MAX) / maximum),
        .y = (int16_t)(((int64_t)y * INT16_MAX) / maximum),
    };
}

static inline void pmw3610_output_init(struct pmw3610_output_state *state) {
    *state = (struct pmw3610_output_state){0};
}

static inline bool pmw3610_output_has_pending(const struct pmw3610_output_state *state) {
    return state->retry_pending || state->pending_x != 0 || state->pending_y != 0 ||
           state->pending_force_sync;
}

static inline void pmw3610_output_queue(struct pmw3610_output_state *state, int32_t x,
                                        int32_t y, bool force_sync) {
    state->pending_x = x;
    state->pending_y = y;
    state->pending_force_sync = force_sync;
}

static inline struct pmw3610_output_frame
pmw3610_output_take_next(struct pmw3610_output_state *state) {
    if (state->retry_pending) {
        return (struct pmw3610_output_frame){
            .x = state->retry_x,
            .y = state->retry_y,
            .force_sync = state->retry_force_sync,
            .retrying = true,
        };
    }

    const struct pmw3610_frame_chunk chunk =
        pmw3610_frame_chunk_from_pending(state->pending_x, state->pending_y);
    state->pending_x -= chunk.x;
    state->pending_y -= chunk.y;
    const bool force_sync = state->pending_force_sync;
    state->pending_force_sync = false;
    return (struct pmw3610_output_frame){
        .x = chunk.x,
        .y = chunk.y,
        .force_sync = force_sync,
        .retrying = false,
    };
}

static inline void pmw3610_output_complete(struct pmw3610_output_state *state,
                                           struct pmw3610_frame_retry retry,
                                           bool zero_sync_failed) {
    state->retry_x = retry.x;
    state->retry_y = retry.y;
    state->retry_pending = retry.x != 0 || retry.y != 0 || zero_sync_failed;
    state->retry_force_sync = zero_sync_failed;
}

static inline uint8_t pmw3610_next_retry_delay(uint8_t current_ms) {
    return current_ms < (PMW3610_READ_RETRY_MAX_MS / 2)
               ? (uint8_t)(current_ms * 2)
               : PMW3610_READ_RETRY_MAX_MS;
}

static inline struct pmw3610_frame_retry
pmw3610_frame_retry_result(int16_t x, int16_t y, int x_error, int y_error) {
    bool have_x = x != 0;
    bool have_y = y != 0;
    bool send_y = have_y && (!have_x || x_error == 0);

    return (struct pmw3610_frame_retry){
        .x = have_x && x_error != 0 ? x : 0,
        .y = have_y && (!send_y || y_error != 0) ? y : 0,
        .send_y = send_y,
    };
}

static inline void pmw3610_report_accumulator_init(struct pmw3610_report_accumulator *state) {
    *state = (struct pmw3610_report_accumulator){0};
}

static inline void pmw3610_report_accumulate(struct pmw3610_report_accumulator *state,
                                             int64_t now, int32_t dx, int32_t dy) {
    if (!state->have_accumulation_start && (dx != 0 || dy != 0)) {
        state->accumulation_start_time = now;
        state->have_accumulation_start = true;
    }
    state->dx += dx;
    state->dy += dy;
    if (state->dx == 0 && state->dy == 0) {
        state->accumulation_start_time = 0;
        state->have_accumulation_start = false;
    }
}

static inline bool
pmw3610_report_prepare_schedule(struct pmw3610_report_accumulator *state, int64_t now,
                                int32_t interval_ms, int64_t *delay_ms) {
    if (state->report_scheduled || (state->dx == 0 && state->dy == 0)) {
        return false;
    }

    *delay_ms = 0;
    if (state->have_last_report) {
        int64_t elapsed = now - state->last_report_time;
        if (elapsed < interval_ms) {
            *delay_ms = interval_ms - elapsed;
        }
    }

    state->report_scheduled = true;
    return true;
}

static inline int16_t pmw3610_clamp_report_delta(int64_t value) {
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static inline bool pmw3610_report_take(struct pmw3610_report_accumulator *state, int64_t now,
                                       int16_t *dx, int16_t *dy,
                                       int64_t *collection_elapsed_ms) {
    state->report_scheduled = false;
    *dx = pmw3610_clamp_report_delta(state->dx);
    *dy = pmw3610_clamp_report_delta(state->dy);

    if (*dx == 0 && *dy == 0) {
        return false;
    }

    *collection_elapsed_ms = state->have_accumulation_start
                                 ? now - state->accumulation_start_time
                                 : 0;
    state->dx -= *dx;
    state->dy -= *dy;
    if (state->dx == 0 && state->dy == 0) {
        state->accumulation_start_time = 0;
        state->have_accumulation_start = false;
    }
    state->last_report_time = now;
    state->have_last_report = true;
    return true;
}
