#include <stdio.h>
#include <stdlib.h>

#include "task3_controller.h"

#define CHECK(expr) do { if (!(expr)) { \
    printf("FAIL line %d: %s\n", __LINE__, #expr); exit(1); \
} } while (0)

static const task3_controller_config_t config = {
    740, 635, 800, 6, 120, 24, 1
};

static void push(task3_controller_t *c, int valid, int x, int dt)
{
    task3_controller_input_t in;
    in.has_frame = 1;
    in.frame_valid = (uint8_t)valid;
    in.position_0p1mm = (int16_t)x;
    task3_controller_update(c, &in, (uint16_t)dt);
}

static void wait_center(task3_controller_t *c)
{
    int i;
    push(c, 1, 100, 50);
    push(c, 1, 0, 50);
    for (i = 0; i < 40; ++i) push(c, 1, 0, 20);
}

static void test_early_start_waits_for_center(void)
{
    task3_controller_t c;
    task3_controller_output_t out;
    task3_controller_init(&c, &config);
    task3_controller_request_start(&c);
    push(&c, 1, 250, 20);
    out = task3_controller_get_output(&c);
    CHECK(out.state == TASK3_CENTER_READY);
    CHECK(out.center_ready == 0);
    CHECK(out.start_pending == 1);
    wait_center(&c);
    out = task3_controller_get_output(&c);
    CHECK(out.state == TASK3_GO_PLUS);
}

static void test_plus_requires_measured_acceptance_band(void)
{
    task3_controller_t c;
    task3_controller_output_t out;
    task3_controller_init(&c, &config);
    wait_center(&c);
    task3_controller_request_start(&c);
    push(&c, 1, 399, 50);
    out = task3_controller_get_output(&c);
    CHECK(out.state == TASK3_GO_PLUS);
    push(&c, 1, 650, 50);
    out = task3_controller_get_output(&c);
    CHECK(out.state == TASK3_GO_PLUS);
    push(&c, 1, 590, 50);
    out = task3_controller_get_output(&c);
    CHECK(out.state == TASK3_GO_MINUS);
    CHECK(out.plus_reached == 1);
}

static void test_invalid_frame_ages_camera_out(void)
{
    task3_controller_t c;
    task3_controller_output_t out;
    task3_controller_init(&c, &config);
    wait_center(&c);
    task3_controller_request_start(&c);
    push(&c, 1, 0, 20);
    push(&c, 0, 200, 149);
    out = task3_controller_get_output(&c);
    CHECK(out.camera_valid == 1);
    CHECK(out.state == TASK3_GO_PLUS);
    push(&c, 0, 200, 1);
    out = task3_controller_get_output(&c);
    CHECK(out.camera_valid == 0);
    CHECK(out.state == TASK3_CAMERA_LOST);
}

static void test_return_sequence_reaches_hold_minus(void)
{
    task3_controller_t c;
    task3_controller_output_t out;
    int i;
    const int positions[] = {500, 300, 100, -100, -300, -500};

    task3_controller_init(&c, &config);
    wait_center(&c);
    task3_controller_request_start(&c);
    push(&c, 1, 0, 20);
    for (i = 0; i < 6; ++i) push(&c, 1, positions[i], 100);
    out = task3_controller_get_output(&c);
    CHECK(out.state == TASK3_GO_MINUS);
    for (i = 0; i < 20; ++i) push(&c, 1, -500, 100);
    out = task3_controller_get_output(&c);
    CHECK(out.state == TASK3_HOLD_MINUS);
    CHECK(out.completed == 1);
}

static void test_plus_timeout_is_independent_of_position(void)
{
    task3_controller_t c;
    task3_controller_output_t out;
    int i;

    task3_controller_init(&c, &config);
    wait_center(&c);
    task3_controller_request_start(&c);
    push(&c, 1, 0, 20);
    for (i = 0; i < 110; ++i) push(&c, 1, 0, 20);
    out = task3_controller_get_output(&c);
    CHECK(out.state == TASK3_TIMEOUT);
}

static void test_jump_filter_keeps_position_and_servo_safe(void)
{
    task3_controller_t c;
    task3_controller_output_t before;
    task3_controller_output_t after;

    task3_controller_init(&c, &config);
    wait_center(&c);
    task3_controller_request_start(&c);
    push(&c, 1, 0, 20);
    before = task3_controller_get_output(&c);
    push(&c, 1, 1200, 20);
    after = task3_controller_get_output(&c);
    CHECK(after.position_0p1mm == before.position_0p1mm);
    CHECK(after.servo_command - before.servo_command <= 6 ||
          before.servo_command - after.servo_command <= 6);
}

static void test_servo_command_is_rate_and_range_limited(void)
{
    task3_controller_t c;
    task3_controller_output_t before;
    task3_controller_output_t after;
    int i;

    task3_controller_init(&c, &config);
    wait_center(&c);
    task3_controller_request_start(&c);
    push(&c, 1, 0, 20);
    before = task3_controller_get_output(&c);
    for (i = 0; i < 20; ++i)
    {
        push(&c, 1, -600, 100);
        after = task3_controller_get_output(&c);
        CHECK(after.servo_command >= before.servo_command ?
              after.servo_command - before.servo_command <= 6 :
              before.servo_command - after.servo_command <= 6);
        CHECK(after.servo_command <= 800);
        before = after;
    }
    CHECK(after.servo_command == 800);

    task3_controller_init(&c, &config);
    wait_center(&c);
    task3_controller_request_start(&c);
    push(&c, 1, 0, 20);
    push(&c, 1, 500, 100);
    before = task3_controller_get_output(&c);
    for (i = 0; i < 20; ++i)
    {
        push(&c, 1, 600, 100);
        after = task3_controller_get_output(&c);
        CHECK(after.servo_command >= before.servo_command ?
              after.servo_command - before.servo_command <= 6 :
              before.servo_command - after.servo_command <= 6);
        CHECK(after.servo_command >= 635);
        before = after;
    }
    CHECK(after.servo_command == 635);
}

static void test_running_request_aborts_to_safe_wait(void)
{
    task3_controller_t c;
    task3_controller_output_t out;

    task3_controller_init(&c, &config);
    wait_center(&c);
    task3_controller_request_start(&c);
    push(&c, 1, 0, 20);
    task3_controller_request_start(&c);
    out = task3_controller_get_output(&c);
    CHECK(out.state == TASK3_WAIT_CAMERA);
    CHECK(out.start_pending == 0);
    CHECK(out.camera_valid == 0);
    CHECK(out.velocity_0p1mm_s == 0);
}

static void test_center_speed_violation_resets_stability(void)
{
    task3_controller_t c;
    task3_controller_output_t out;
    int i;

    task3_controller_init(&c, &config);
    push(&c, 1, 0, 20);
    for (i = 0; i < 10; ++i) push(&c, 1, 0, 20);
    CHECK(c.center_stable_ms == 200);
    push(&c, 1, 200, 20);
    out = task3_controller_get_output(&c);
    CHECK(out.position_0p1mm == 100);
    CHECK(out.velocity_0p1mm_s > 150);
    CHECK(c.center_stable_ms == 0);
    CHECK(out.center_ready == 0);
}

static void test_total_timeout_wins_over_same_cycle_completion(void)
{
    task3_controller_t c;
    task3_controller_output_t out;

    task3_controller_init(&c, &config);
    c.output.state = TASK3_GO_MINUS;
    c.output.target_0p1mm = -500;
    c.output.camera_valid = 1;
    c.output.task_elapsed_ms = 4700;
    c.output.position_0p1mm = -500;
    c.output.velocity_0p1mm_s = 0;
    c.output.servo_command = 740;
    c.raw_position_0p1mm = -500;
    c.previous_position_0p1mm = -500;
    c.frame_elapsed_ms = 20;
    c.final_stable_ms = 200;
    c.have_position = 1;
    push(&c, 1, -500, 100);
    out = task3_controller_get_output(&c);
    CHECK(out.task_elapsed_ms == 4800);
    CHECK(out.state == TASK3_TIMEOUT);
    CHECK(out.completed == 0);
}

static void test_total_timeout_wins_over_same_cycle_camera_loss(void)
{
    task3_controller_t c;
    task3_controller_output_t out;
    task3_controller_input_t in = {0};

    task3_controller_init(&c, &config);
    c.output.state = TASK3_GO_MINUS;
    c.output.target_0p1mm = -500;
    c.output.camera_valid = 1;
    c.output.task_elapsed_ms = 4650;
    c.output.servo_command = 740;
    c.have_position = 1;

    task3_controller_update(&c, &in, 150);
    out = task3_controller_get_output(&c);
    CHECK(out.task_elapsed_ms == 4800);
    CHECK(out.state == TASK3_TIMEOUT);
}

int main(void)
{
    test_early_start_waits_for_center();
    test_plus_requires_measured_acceptance_band();
    test_invalid_frame_ages_camera_out();
    test_return_sequence_reaches_hold_minus();
    test_plus_timeout_is_independent_of_position();
    test_jump_filter_keeps_position_and_servo_safe();
    test_servo_command_is_rate_and_range_limited();
    test_running_request_aborts_to_safe_wait();
    test_center_speed_violation_resets_stability();
    test_total_timeout_wins_over_same_cycle_completion();
    test_total_timeout_wins_over_same_cycle_camera_loss();
    printf("task3_controller tests passed\n");
    return 0;
}
