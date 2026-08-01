#include <stdio.h>
#include <stdlib.h>
#include "track_controller.h"

#define CHECK(expr) do { if (!(expr)) { \
    printf("FAIL line %d: %s\n", __LINE__, #expr); exit(1); \
} } while (0)

static int abs_i(int value) { return value < 0 ? -value : value; }

static void test_center_has_no_artificial_weave(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;
    int i;
    track_controller_init(&c);
    for (i = 0; i < 40; ++i)
    {
        out = track_controller_step(&c, 0x18, 10);
        CHECK(out.error == 0);
        CHECK(out.turn == 0);
        CHECK(out.left_speed == out.right_speed);
    }
}

static void test_turn_grows_continuously_before_outer_sensor(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t a;
    Track_Controller_Output_t b;
    track_controller_init(&c);
    (void)track_controller_step(&c, 0x18, 10);
    a = track_controller_step(&c, 0x20, 10);
    b = track_controller_step(&c, 0x20, 10);
    CHECK(a.error == 55);
    CHECK(a.turn > 0 && a.turn <= 18);
    CHECK(b.turn > a.turn);
    CHECK(b.turn - a.turn <= 18);
    CHECK(b.phase == TRACK_PHASE_CURVE);
    CHECK(b.base_speed <= 190 && b.base_speed >= 145);
}

static void test_negative_turn_growth_is_limited_to_18(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t a;
    Track_Controller_Output_t b;

    track_controller_init(&c);
    (void)track_controller_step(&c, 0x18, 10);
    a = track_controller_step(&c, 0x04, 10);
    b = track_controller_step(&c, 0x04, 10);
    CHECK(a.error == -55);
    CHECK(a.turn < 0 && a.turn >= -18);
    CHECK(b.turn < a.turn);
    CHECK(a.turn - b.turn <= 18);
}

static void test_exit_damping_cannot_cross_zero_in_one_frame(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t before;
    Track_Controller_Output_t after;
    int i;
    track_controller_init(&c);
    for (i = 0; i < 4; ++i)
        before = track_controller_step(&c, 0x20, 10);
    after = track_controller_step(&c, 0x18, 10);
    CHECK(before.turn > 0);
    CHECK(after.turn >= 0);
    CHECK(before.turn - after.turn <= 24);
    CHECK(abs_i(after.turn) <= 140);
}

static void test_pd_filter_and_input_clamp(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;
    int i;

    track_controller_init(&c);
    track_controller_set_gains(&c, 20, 100);
    out = track_controller_step(&c, 0x20, 10);
    CHECK(out.derivative == 18);
    CHECK(out.turn == 18);
    out = track_controller_step(&c, 0x20, 10);
    CHECK(out.derivative == 12);
    CHECK(out.turn == 23);

    track_controller_init(&c);
    track_controller_set_gains(&c, 100, 0);
    out = track_controller_step(&c, 0x80, 10);
    CHECK(out.error == 160);
    CHECK(out.derivative == 26);
    CHECK(out.turn == 18);
    for (i = 0; i < 7; ++i)
        out = track_controller_step(&c, 0x80, 10);
    CHECK(out.derivative == 0);
    CHECK(out.turn == 140);
    out = track_controller_step(&c, 0x01, 10);
    CHECK(out.error == -160);
    CHECK(out.derivative == -26);
    CHECK(out.turn == 116);
}

static void test_curve_exit_requires_six_frames_and_holds_speed(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;
    int i;

    track_controller_init(&c);
    (void)track_controller_step(&c, 0x20, 10);
    out = track_controller_step(&c, 0x20, 10);
    CHECK(out.phase == TRACK_PHASE_CURVE);
    CHECK(out.base_speed == 190);
    for (i = 0; i < 5; ++i)
    {
        out = track_controller_step(&c, 0x18, 10);
        CHECK(out.phase == TRACK_PHASE_CURVE);
        CHECK(out.base_speed == 190);
    }
    out = track_controller_step(&c, 0x18, 10);
    CHECK(out.phase == TRACK_PHASE_STRAIGHT);
    CHECK(out.base_speed == 190);
    for (i = 0; i < 9; ++i)
    {
        out = track_controller_step(&c, 0x18, 10);
        CHECK(out.base_speed == 190);
    }
    out = track_controller_step(&c, 0x18, 10);
    CHECK(out.base_speed == 198);
}

static void test_speed_targets_and_ramp_limits(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t a;
    Track_Controller_Output_t b;
    Track_Controller_Output_t d;

    track_controller_init(&c);
    a = track_controller_step(&c, 0x80, 10);
    b = track_controller_step(&c, 0x80, 10);
    d = track_controller_step(&c, 0x80, 10);
    CHECK(a.base_speed == 210);
    CHECK(b.base_speed == 180);
    CHECK(d.base_speed == 150);
    CHECK(a.base_speed - b.base_speed == 30);
    CHECK(b.base_speed - d.base_speed == 30);
    d = track_controller_step(&c, 0x80, 10);
    CHECK(d.base_speed == 145);

    track_controller_init(&c);
    (void)track_controller_step(&c, 0x40, 10);
    (void)track_controller_step(&c, 0x40, 10);
    d = track_controller_step(&c, 0x40, 10);
    CHECK(d.base_speed == 165);

    track_controller_init(&c);
    (void)track_controller_step(&c, 0x90, 10);
    d = track_controller_step(&c, 0x90, 10);
    CHECK(d.base_speed == 180);

    track_controller_init(&c);
    (void)track_controller_step(&c, 0x20, 10);
    d = track_controller_step(&c, 0x20, 10);
    CHECK(d.base_speed == 190);
}

static void test_defaults_and_reset_preserves_gains(void)
{
    Track_Controller_t c;
    Track_Controller_Gains_t gains;

    track_controller_init(&c);
    gains = track_controller_get_gains(&c);
    CHECK(gains.kp_x100 == 75);
    CHECK(gains.kd_x100 == 55);
    CHECK(c.base_speed == 210);
    track_controller_set_gains(&c, 123, 45);
    (void)track_controller_step(&c, 0x80, 10);
    track_controller_reset(&c);
    gains = track_controller_get_gains(&c);
    CHECK(gains.kp_x100 == 123);
    CHECK(gains.kd_x100 == 45);
    CHECK(c.error == 0);
    CHECK(c.last_error == 0);
    CHECK(c.derivative == 0);
    CHECK(c.turn == 0);
    CHECK(c.base_speed == 210);
    CHECK(c.phase == TRACK_PHASE_STRAIGHT);
}

static void test_lost_line_keeps_last_curve_direction(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;
    int i;
    track_controller_init(&c);
    for (i = 0; i < 3; ++i)
        (void)track_controller_step(&c, 0x20, 10);
    for (i = 0; i < 11; ++i)
        out = track_controller_step(&c, 0x00, 10);
    CHECK(out.phase == TRACK_PHASE_RECOVERY_HOLD);
    CHECK(out.turn > 0);
    out = track_controller_step(&c, 0x00, 10);
    CHECK(out.phase == TRACK_PHASE_RECOVERY_SEARCH);
    CHECK(out.left_speed > 0);
    CHECK(out.right_speed < out.left_speed);
}

static void test_lost_line_mirrors_negative_curve_direction(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;
    int i;

    track_controller_init(&c);
    for (i = 0; i < 3; ++i)
        (void)track_controller_step(&c, 0x04, 10);
    for (i = 0; i < 11; ++i)
        out = track_controller_step(&c, 0x00, 10);
    CHECK(out.phase == TRACK_PHASE_RECOVERY_HOLD);
    CHECK(out.base_speed == 90);
    CHECK(out.turn == -80);
    CHECK(out.left_speed == 10);
    CHECK(out.right_speed == 170);
    out = track_controller_step(&c, 0x00, 10);
    CHECK(out.phase == TRACK_PHASE_RECOVERY_SEARCH);
    CHECK(out.base_speed == 45);
    CHECK(out.turn == -70);
    CHECK(out.left_speed == -25);
    CHECK(out.right_speed == 115);
}

static void test_recovery_needs_two_center_frames(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;
    track_controller_init(&c);
    (void)track_controller_step(&c, 0x40, 10);
    (void)track_controller_step(&c, 0x00, 130);
    out = track_controller_step(&c, 0x18, 10);
    CHECK(out.phase == TRACK_PHASE_RECOVERY_SEARCH);
    out = track_controller_step(&c, 0x18, 10);
    CHECK(out.phase == TRACK_PHASE_STRAIGHT);
}

static void test_recovery_center_frames_must_be_consecutive(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;

    track_controller_init(&c);
    (void)track_controller_step(&c, 0x40, 10);
    (void)track_controller_step(&c, 0x00, 130);
    out = track_controller_step(&c, 0x18, 10);
    CHECK(out.phase == TRACK_PHASE_RECOVERY_SEARCH);
    out = track_controller_step(&c, 0x04, 10);
    CHECK(out.phase == TRACK_PHASE_RECOVERY_SEARCH);
    out = track_controller_step(&c, 0x18, 10);
    CHECK(out.phase == TRACK_PHASE_RECOVERY_SEARCH);
    out = track_controller_step(&c, 0x18, 10);
    CHECK(out.phase == TRACK_PHASE_STRAIGHT);
    CHECK(out.error == 0);
    CHECK(out.derivative == 0);
    CHECK(out.turn == 0);
    CHECK(c.last_direction == 0);
}

static void test_lost_line_requests_stop_at_600_ms(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;
    int i;
    track_controller_init(&c);
    (void)track_controller_step(&c, 0x20, 10);
    for (i = 0; i < 60; ++i)
        out = track_controller_step(&c, 0x00, 10);
    CHECK(out.phase == TRACK_PHASE_LOST_STOP);
    CHECK(out.stop_requested == 1);
}

static void test_lost_line_stops_at_600_ms_not_599_ms(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;

    track_controller_init(&c);
    (void)track_controller_step(&c, 0x20, 10);
    out = track_controller_step(&c, 0x00, 599);
    CHECK(out.phase == TRACK_PHASE_RECOVERY_SEARCH);
    CHECK(out.stop_requested == 0);
    out = track_controller_step(&c, 0x00, 1);
    CHECK(out.phase == TRACK_PHASE_LOST_STOP);
    CHECK(out.stop_requested == 1);
}

static void test_finish_marker_rejects_adjacent_curve_bits(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;
    track_controller_init(&c);
    out = track_controller_step(&c, 0x0F, 10);
    CHECK(out.finish_detected == 0);
    out = track_controller_step(&c, 0x0F, 10);
    CHECK(out.finish_detected == 0);
}

static void test_finish_marker_requires_both_sensor_halves(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;

    track_controller_init(&c);
    (void)track_controller_step(&c, 0x0F, 10);
    out = track_controller_step(&c, 0x0F, 10);
    CHECK(out.finish_detected == 0);
}

static void test_finish_marker_requires_wide_span(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;

    track_controller_init(&c);
    (void)track_controller_step(&c, 0x3C, 10);
    out = track_controller_step(&c, 0x3C, 10);
    CHECK(out.finish_detected == 0);
}

static void test_finish_marker_requires_two_wide_frames(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;
    track_controller_init(&c);
    out = track_controller_step(&c, 0xC3, 10);
    CHECK(out.finish_detected == 0);
    out = track_controller_step(&c, 0xC3, 10);
    CHECK(out.finish_detected == 1);
}

static void test_finish_candidate_holds_previous_output(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t previous;
    Track_Controller_Output_t out;

    track_controller_init(&c);
    (void)track_controller_step(&c, 0x20, 10);
    previous = track_controller_step(&c, 0x20, 10);
    CHECK(previous.turn != 0);
    out = track_controller_step(&c, 0xC3, 10);
    CHECK(out.error == previous.error);
    CHECK(out.derivative == previous.derivative);
    CHECK(out.turn == previous.turn);
    CHECK(out.base_speed == previous.base_speed);
    CHECK(out.left_speed == previous.left_speed);
    CHECK(out.right_speed == previous.right_speed);
    CHECK(out.finish_detected == 0);
    out = track_controller_step(&c, 0xC3, 10);
    CHECK(out.error == previous.error);
    CHECK(out.derivative == previous.derivative);
    CHECK(out.turn == previous.turn);
    CHECK(out.base_speed == previous.base_speed);
    CHECK(out.left_speed == previous.left_speed);
    CHECK(out.right_speed == previous.right_speed);
    CHECK(out.finish_detected == 1);
}

static void test_finish_marker_requires_consecutive_wide_frames(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;
    track_controller_init(&c);
    (void)track_controller_step(&c, 0xC3, 10);
    (void)track_controller_step(&c, 0x00, 10);
    out = track_controller_step(&c, 0xC3, 10);
    CHECK(out.finish_detected == 0);
}

static void test_finish_marker_noncandidate_frame_resets_sequence(void)
{
    Track_Controller_t c;
    Track_Controller_Output_t out;

    track_controller_init(&c);
    (void)track_controller_step(&c, 0xC3, 10);
    (void)track_controller_step(&c, 0x18, 10);
    out = track_controller_step(&c, 0xC3, 10);
    CHECK(out.finish_detected == 0);
}

static void test_brake_speed_is_opposite_and_bounded(void)
{
    CHECK(track_controller_brake_speed(210) == -70);
    CHECK(track_controller_brake_speed(-150) == 50);
    CHECK(track_controller_brake_speed(45) == -20);
    CHECK(track_controller_brake_speed(20) == 0);
    CHECK(track_controller_brake_speed(30) == -20);
    CHECK(track_controller_brake_speed(-30) == 20);
}

int main(void)
{
    test_center_has_no_artificial_weave();
    test_turn_grows_continuously_before_outer_sensor();
    test_negative_turn_growth_is_limited_to_18();
    test_exit_damping_cannot_cross_zero_in_one_frame();
    test_pd_filter_and_input_clamp();
    test_curve_exit_requires_six_frames_and_holds_speed();
    test_speed_targets_and_ramp_limits();
    test_defaults_and_reset_preserves_gains();
    test_lost_line_keeps_last_curve_direction();
    test_lost_line_mirrors_negative_curve_direction();
    test_recovery_needs_two_center_frames();
    test_recovery_center_frames_must_be_consecutive();
    test_lost_line_requests_stop_at_600_ms();
    test_lost_line_stops_at_600_ms_not_599_ms();
    test_finish_marker_rejects_adjacent_curve_bits();
    test_finish_marker_requires_both_sensor_halves();
    test_finish_marker_requires_wide_span();
    test_finish_marker_requires_two_wide_frames();
    test_finish_candidate_holds_previous_output();
    test_finish_marker_requires_consecutive_wide_frames();
    test_finish_marker_noncandidate_frame_resets_sequence();
    test_brake_speed_is_opposite_and_bounded();
    puts("track_controller core tests passed");
    return 0;
}
