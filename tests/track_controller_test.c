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

int main(void)
{
    test_center_has_no_artificial_weave();
    test_turn_grows_continuously_before_outer_sensor();
    test_exit_damping_cannot_cross_zero_in_one_frame();
    puts("track_controller core tests passed");
    return 0;
}
