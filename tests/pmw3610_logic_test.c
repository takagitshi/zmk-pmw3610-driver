/* SPDX-License-Identifier: MIT */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../src/pmw3610_logic.h"

static void test_delta12_boundaries(void) {
    assert(pmw3610_decode_delta12(0x00, 0x0) == 0);
    assert(pmw3610_decode_delta12(0xff, 0x7) == 2047);
    assert(pmw3610_decode_delta12(0x00, 0x8) == -2048);
    assert(pmw3610_decode_delta12(0xff, 0xf) == -1);
}

static void test_packed_motion_nibble_mapping_and_sign(void) {
    struct pmw3610_motion_delta delta =
        pmw3610_decode_motion_delta(0x23, 0xdd, 0x1e);
    assert(delta.x == 0x123);
    assert(delta.y == -0x123);

    delta = pmw3610_decode_motion_delta(0xdd, 0x23, 0xe1);
    assert(delta.x == -0x123);
    assert(delta.y == 0x123);
}

static void test_retry_semantics(void) {
    struct pmw3610_frame_retry retry = pmw3610_frame_retry_result(10, 20, -1, 0);
    assert(!retry.send_y && retry.x == 10 && retry.y == 20);
    retry = pmw3610_frame_retry_result(10, 20, 0, -1);
    assert(retry.send_y && retry.x == 0 && retry.y == 20);
    retry = pmw3610_frame_retry_result(10, 20, 0, 0);
    assert(retry.send_y && retry.x == 0 && retry.y == 0);
}

static void test_overflow_chunk_and_retry_order(void) {
    struct pmw3610_output_state state;
    pmw3610_output_init(&state);
    pmw3610_output_queue(&state, 49150, 24575, true);

    struct pmw3610_output_frame first = pmw3610_output_take_next(&state);
    assert(first.x == 32767 && first.y == 16383);
    pmw3610_output_complete(&state,
                            pmw3610_frame_retry_result(first.x, first.y, 0, -1), false);

    struct pmw3610_output_frame second = pmw3610_output_take_next(&state);
    assert(second.retrying && second.x == 0 && second.y == 16383);
    pmw3610_output_complete(&state,
                            pmw3610_frame_retry_result(second.x, second.y, 0, 0), false);

    struct pmw3610_output_frame third = pmw3610_output_take_next(&state);
    assert(!third.retrying && third.x == 16383 && third.y == 8192);
    pmw3610_output_complete(&state,
                            pmw3610_frame_retry_result(third.x, third.y, 0, 0), false);
    assert(!pmw3610_output_has_pending(&state));
}

static void test_repeated_failure_and_zero_sync_retry(void) {
    struct pmw3610_output_state state;
    pmw3610_output_init(&state);
    pmw3610_output_queue(&state, 10, 20, true);

    struct pmw3610_output_frame frame = pmw3610_output_take_next(&state);
    pmw3610_output_complete(&state,
                            pmw3610_frame_retry_result(frame.x, frame.y, -1, 0), false);
    frame = pmw3610_output_take_next(&state);
    assert(frame.retrying && frame.x == 10 && frame.y == 20);
    pmw3610_output_complete(&state,
                            pmw3610_frame_retry_result(frame.x, frame.y, -1, 0), false);
    frame = pmw3610_output_take_next(&state);
    assert(frame.retrying && frame.x == 10 && frame.y == 20);
    pmw3610_output_complete(&state,
                            pmw3610_frame_retry_result(frame.x, frame.y, 0, 0), false);
    assert(!pmw3610_output_has_pending(&state));

    pmw3610_output_queue(&state, 0, 0, true);
    frame = pmw3610_output_take_next(&state);
    assert(frame.force_sync && frame.x == 0 && frame.y == 0);
    pmw3610_output_complete(&state, (struct pmw3610_frame_retry){0}, true);
    frame = pmw3610_output_take_next(&state);
    assert(frame.retrying && frame.force_sync && frame.x == 0 && frame.y == 0);
    pmw3610_output_complete(&state, (struct pmw3610_frame_retry){0}, false);
    assert(!pmw3610_output_has_pending(&state));
}

static void test_negative_overflow_chunk(void) {
    const struct pmw3610_frame_chunk chunk =
        pmw3610_frame_chunk_from_pending(-49150, -24575);
    assert(chunk.x == -32767 && chunk.y == -16383);
}

static void test_fixed_deadline_and_tail_flush(void) {
    struct pmw3610_report_accumulator state;
    int64_t delay;
    int64_t elapsed;
    int16_t x;
    int16_t y;

    pmw3610_report_accumulator_init(&state);
    pmw3610_report_accumulate(&state, 1000, 10, -5);
    assert(pmw3610_report_prepare_schedule(&state, 1000, 15, &delay));
    assert(delay == 0);
    assert(pmw3610_report_take(&state, 1000, &x, &y, &elapsed));
    assert(x == 10 && y == -5 && elapsed == 0);

    pmw3610_report_accumulate(&state, 1008, 2, 3);
    assert(pmw3610_report_prepare_schedule(&state, 1008, 15, &delay));
    assert(delay == 7);
    pmw3610_report_accumulate(&state, 1012, 4, -1);
    assert(pmw3610_report_take(&state, 1015, &x, &y, &elapsed));
    assert(x == 6 && y == 2 && elapsed == 7);

    pmw3610_report_accumulate(&state, 1020, 1, 0);
    assert(pmw3610_report_prepare_schedule(&state, 1020, 15, &delay));
    assert(delay == 10);
    assert(pmw3610_report_take(&state, 1030, &x, &y, &elapsed));
    assert(x == 1 && y == 0 && elapsed == 10);
}

int main(void) {
    test_delta12_boundaries();
    test_packed_motion_nibble_mapping_and_sign();
    test_retry_semantics();
    test_overflow_chunk_and_retry_order();
    test_repeated_failure_and_zero_sync_retry();
    test_negative_overflow_chunk();
    test_fixed_deadline_and_tail_flush();
    puts("pmw3610_logic_test: PASS");
    return 0;
}
