#include "headfile.h"
#include "app_board.h"
#include "app_module_test.h"
#include "track_control.h"
#include "camera.h"
#include "ball_balance.h"

/* ── Timing constants ── */
#define LOOP_DT_MS              10
#define OLED_UPDATE_MS          200
#define INIT_STEP_DELAY_MS      300

/* ── Stop line detection ── */
#define STOP_LINE_ACTIVE_COUNT  7
#define STOP_LINE_HOLD_COUNT    3
#define STOP_LINE_DEAD_TIME_MS  2000

/* ── Task time limits (ms) ── */
#define TASK_2_TIME_LIMIT       20000
#define TASK_4_TIME_LIMIT       8000
#define TASK_5_TIME_LIMIT       30000
#define TASK_6_TIME_LIMIT       30000

/* ── Timer period ── */
#define TIMER_PERIOD_MS         5

/* ── Task enumeration ── */
typedef enum
{
    TASK_SELECT = 0,
    TASK_2_RUN,
    TASK_2_DONE,
    TASK_3_RUN,
    TASK_3_DONE,
    TASK_4_RUN,
    TASK_4_DONE,
    TASK_5_RUN,
    TASK_5_DONE,
    TASK_6_RUN,
    TASK_6_DONE
} System_State_t;

/* ── Task 3 sub-states ── */
typedef enum
{
    T3_START = 0,
    T3_MOVE_TO_PLUS,
    T3_HOLD_PLUS,
    T3_MOVE_TO_MINUS,
    T3_HOLD_MINUS
} Task3_Phase_t;

/* ── Global state ── */
static System_State_t sys_state = TASK_SELECT;
static uint8_t current_task = 2;
/* ── Timing ── */
static uint32_t sys_time_ms = 0;
/* task elapsed time now comes from oled_timer_get_elapsed(&g_stopwatch) — TIM4 1ms hardware tick */

/* ── Lap tracking ── */
static uint8_t lap_count = 0;
static uint8_t target_laps = 0;
static uint8_t stop_line_hold = 0;
static uint32_t last_stop_line_time_ms = 0;

/* ── Task 3 ── */
static Task3_Phase_t t3_phase = T3_START;
static uint32_t t3_phase_start_ms = 0;
/* ── Task 6 ball target ── */
static int task6_ball_target = 300;

/* ── OLED tick ── */
static uint32_t oled_tick = 0;

/* ── Keys ── */
static app_key_t start_key;
static app_key_t task_key;

/* ── IMU yaw from isr.c ── */
extern float yaw_gyro;


/* ── Display helpers ── */

static void show_time(uint8_t row, uint8_t col, uint32_t time_ms)
{
    uint16_t sec = (uint16_t)(time_ms / 1000);
    uint16_t ms = (uint16_t)(time_ms % 1000);

    OLED_ShowNum(row, col, sec, 2);
    OLED_ShowString(row, col + 2, ".");
    OLED_ShowNum(row, col + 3, ms / 10, 2);
}

/* ── Motor stop on boot ── */

static void motor_stop_on_boot(void)
{
    uint8_t i;
    uart_init(UART_2, 115200, 1);
    USART2->CR1 &= ~(1 << 5);
    for (i = 0; i < 5; i++)
    {
        control_speed(0, 0, 0, 0);
        delay_ms(20);
    }
}

/* ── IMU initialization (yaw only, for heading reference) ── */

static void imu_init(void)
{
    I2C_Init();
    MPU6050_Init();
    HMC5883L_Init();
    exti_init(EXTI_PA7, RISING, 0);
    delay_ms(300);
    yaw_gyro = 0.0f;
}

/* ── Key initialization ── */

static void keys_init(void)
{
    app_key_init(&start_key, BOARD_START_KEY_PORT, BOARD_START_KEY_PIN,
                 BOARD_START_KEY_ACTIVE, 20);
    app_key_init(&task_key, BOARD_TASK_KEY_PORT, BOARD_TASK_KEY_PIN,
                 BOARD_TASK_KEY_ACTIVE, 20);
}

/* ── Stop line detection ── */

static uint8_t detect_stop_line(void)
{
    Track_Info_t info = track_get_info();

    if (info.active_count >= STOP_LINE_ACTIVE_COUNT)
    {
        if (stop_line_hold < 255)
            stop_line_hold++;
    }
    else
    {
        if (stop_line_hold > 0)
            stop_line_hold--;
    }

    if (stop_line_hold >= STOP_LINE_HOLD_COUNT &&
        (sys_time_ms - last_stop_line_time_ms) > STOP_LINE_DEAD_TIME_MS)
    {
        last_stop_line_time_ms = sys_time_ms;
        stop_line_hold = 0;
        return 1;
    }

    return 0;
}

/* ── OLED display for task selection ── */

static void oled_show_task_select(void)
{
    OLED_ShowString(1, 1, "Select Task:");
    OLED_ShowString(2, 1, "Task");
    OLED_ShowNum(2, 6, current_task, 1);
    OLED_ShowString(2, 8, "- PA6");
    OLED_ShowString(3, 1, "PB1: Start");
}

/* ── OLED display during task run ── */

static void oled_show_task_run(void)
{
    /* Line 2: task number */
    OLED_ShowString(1, 1, "Task");
    OLED_ShowNum(1, 6, current_task, 1);
    OLED_ShowString(1, 8, "Running");

    /* Line 3: time */
    OLED_ShowString(2, 3, "Time:");
    show_time(2, 9, oled_timer_get_elapsed(&g_stopwatch));

    /* clear unused lines */
    OLED_ShowString(3, 1, "                ");
    OLED_ShowString(4, 1, "                ");
}

/* ── OLED display for task result ── */

static void oled_show_task_result(void)
{
    OLED_ShowString(1, 1, "Task");
    OLED_ShowNum(1, 6, current_task, 1);
    OLED_ShowString(1, 8, "Done!");

    OLED_ShowString(2, 3, "Time:");
    show_time(2, 9, oled_timer_get_elapsed(&g_stopwatch));

    OLED_ShowString(3, 1, "PB1: Next Task");
    OLED_ShowString(4, 1, "                ");
}

/* ── System initialization ── */

static void system_init(void)
{
    motor_stop_on_boot();

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(1, 1, "Ball Balance Car");
    OLED_ShowString(2, 1, "Initializing...");
    delay_ms(500);

    /* Init hardware 1ms timer for stopwatch */
    oled_timer_hw_init();

    /* Init tracking module (UART1) */
    OLED_ShowString(3, 1, "Track Init...");
    track_control_init();
    OLED_ShowString(3, 1, "Track OK     ");
    delay_ms(INIT_STEP_DELAY_MS);

    /* Init IMU (I2C on PB6/PB7, INT on PA7) — yaw reference only */
    OLED_ShowString(3, 1, "IMU Init...  ");
    imu_init();
    OLED_ShowString(3, 1, "IMU OK       ");
    delay_ms(INIT_STEP_DELAY_MS);

    /* Init camera UART (UART3 on PB10/PB11) */
    OLED_ShowString(3, 1, "Camera Init..");
    camera_init();
    OLED_ShowString(3, 1, "Camera OK    ");
    delay_ms(INIT_STEP_DELAY_MS);

    /* Init motor (UART2) */
    OLED_ShowString(3, 1, "Motor Init...");
    motor_init();
    track_car_stop();
    OLED_ShowString(3, 1, "Motor OK     ");
    delay_ms(INIT_STEP_DELAY_MS);

    /* Init ball balance servo (TIM3_CH3 on PB0) */
    OLED_ShowString(3, 1, "Servo Init...");
    ball_balance_init();
    OLED_ShowString(3, 1, "Servo OK     ");
    delay_ms(INIT_STEP_DELAY_MS);

    /* Init keys */
    keys_init();

    /* Timer interrupt for PID control (5ms) */
    tim_interrupt_ms_init(TIM_3, TIMER_PERIOD_MS, 1);

    OLED_Clear();
    OLED_ShowString(1, 1, "Init Complete");
    delay_ms(500);
    OLED_Clear();

    sys_state = TASK_SELECT;
    sys_time_ms = 0;
}

/* ── Task 2: One lap line following ── */

static void task_2_start(void)
{
    track_car_stop();
    ball_balance_set_target(0);
    ball_balance_servo_center();
    lap_count = 0;
    target_laps = 1;
    stop_line_hold = 0;
    last_stop_line_time_ms = 0;

    /* Start hardware stopwatch from 0 */
    oled_timer_init(&g_stopwatch);
    oled_timer_start(&g_stopwatch);
}

static void task_2_update(void)
{
    uint32_t elapsed = oled_timer_get_elapsed(&g_stopwatch);

    if (elapsed >= TASK_2_TIME_LIMIT)
    {
        track_car_stop();
        oled_timer_pause(&g_stopwatch);
        sys_state = TASK_2_DONE;
        return;
    }

    track_follow_update();

    if (detect_stop_line())
    {
        lap_count++;
        if (lap_count >= target_laps)
        {
            track_car_stop();
            oled_timer_pause(&g_stopwatch);
            sys_state = TASK_2_DONE;
            return;
        }
    }

    ball_balance_update();
}

/* ── Task 3: Static ball positioning ── */

static void task_3_start(void)
{
    track_car_stop();
    ball_balance_set_target(0);
    ball_balance_servo_center();
    t3_phase = T3_START;
    t3_phase_start_ms = sys_time_ms;

    /* Start hardware stopwatch from 0 */
    oled_timer_init(&g_stopwatch);
    oled_timer_start(&g_stopwatch);
}

static void task_3_update(void)
{
    uint32_t phase_elapsed;
    uint32_t elapsed = oled_timer_get_elapsed(&g_stopwatch);

    phase_elapsed = sys_time_ms - t3_phase_start_ms;

    track_car_stop();

    switch (t3_phase)
    {
        case T3_START:
            ball_balance_set_target(TASK_3_TARGET_PLUS);
            t3_phase = T3_MOVE_TO_PLUS;
            t3_phase_start_ms = sys_time_ms;
            break;

        case T3_MOVE_TO_PLUS:
            if (phase_elapsed >= TASK_3_MOVE_TIME_MS)
            {
                t3_phase = T3_HOLD_PLUS;
                t3_phase_start_ms = sys_time_ms;
            }
            break;

        case T3_HOLD_PLUS:
            if (phase_elapsed >= TASK_3_HOLD_TIME_MS)
            {
                ball_balance_pos_error();
                ball_balance_set_target(TASK_3_TARGET_MINUS);
                t3_phase = T3_MOVE_TO_MINUS;
                t3_phase_start_ms = sys_time_ms;
            }
            break;

        case T3_MOVE_TO_MINUS:
            if (phase_elapsed >= TASK_3_MOVE_TIME_MS)
            {
                t3_phase = T3_HOLD_MINUS;
                t3_phase_start_ms = sys_time_ms;
            }
            break;

        case T3_HOLD_MINUS:
            if (phase_elapsed >= TASK_3_HOLD_TIME_MS || elapsed >= TASK_3_TOTAL_MS)
            {
                ball_balance_pos_error();
                ball_balance_set_target(0);
                oled_timer_pause(&g_stopwatch);
                sys_state = TASK_3_DONE;
                return;
            }
            break;
    }

    ball_balance_update();
}

/* ── Task 4: AB segment with ball at center ── */

static void task_4_start(void)
{
    track_car_stop();
    ball_balance_set_target(0);
    ball_balance_servo_center();
    lap_count = 0;
    target_laps = 0;
    stop_line_hold = 0;
    last_stop_line_time_ms = 0;

    oled_timer_init(&g_stopwatch);
    oled_timer_start(&g_stopwatch);
}

static void task_4_update(void)
{
    uint32_t elapsed = oled_timer_get_elapsed(&g_stopwatch);

    if (elapsed >= TASK_4_TIME_LIMIT)
    {
        track_car_stop();
        oled_timer_pause(&g_stopwatch);
        sys_state = TASK_4_DONE;
        return;
    }

    track_follow_update();

    if (detect_stop_line())
    {
        lap_count++;
        if (lap_count >= 1)
        {
            track_car_stop();
            oled_timer_pause(&g_stopwatch);
            sys_state = TASK_4_DONE;
            return;
        }
    }

    ball_balance_update();
}

/* ── Task 5: Full lap with ball at center ── */

static void task_5_start(void)
{
    track_car_stop();
    ball_balance_set_target(0);
    ball_balance_servo_center();
    lap_count = 0;
    target_laps = 1;
    stop_line_hold = 0;
    last_stop_line_time_ms = 0;

    oled_timer_init(&g_stopwatch);
    oled_timer_start(&g_stopwatch);
}

static void task_5_update(void)
{
    uint32_t elapsed = oled_timer_get_elapsed(&g_stopwatch);

    if (elapsed >= TASK_5_TIME_LIMIT)
    {
        track_car_stop();
        oled_timer_pause(&g_stopwatch);
        sys_state = TASK_5_DONE;
        return;
    }

    track_follow_update();

    if (detect_stop_line())
    {
        lap_count++;
        if (lap_count >= target_laps)
        {
            track_car_stop();
            oled_timer_pause(&g_stopwatch);
            sys_state = TASK_5_DONE;
            return;
        }
    }

    ball_balance_update();
}

/* ── Task 6: Full lap with ball at arbitrary position ── */

static void task_6_start(void)
{
    track_car_stop();
    ball_balance_set_target(task6_ball_target);
    ball_balance_servo_center();
    lap_count = 0;
    target_laps = 1;
    stop_line_hold = 0;
    last_stop_line_time_ms = 0;

    oled_timer_init(&g_stopwatch);
    oled_timer_start(&g_stopwatch);
}

static void task_6_update(void)
{
    uint32_t elapsed = oled_timer_get_elapsed(&g_stopwatch);

    if (elapsed >= TASK_6_TIME_LIMIT)
    {
        track_car_stop();
        oled_timer_pause(&g_stopwatch);
        sys_state = TASK_6_DONE;
        return;
    }

    track_follow_update();

    if (detect_stop_line())
    {
        lap_count++;
        if (lap_count >= target_laps)
        {
            track_car_stop();
            oled_timer_pause(&g_stopwatch);
            sys_state = TASK_6_DONE;
            return;
        }
    }

    ball_balance_update();
}

/* ── Main ── */

int main(void)
{
    system_init();

    while (1)
    {
        sys_time_ms += LOOP_DT_MS;
        camera_update_timeout(LOOP_DT_MS);

        app_key_update(&start_key, LOOP_DT_MS);
        app_key_update(&task_key, LOOP_DT_MS);

        switch (sys_state)
        {
            case TASK_SELECT:
                if (app_key_take_pressed(&task_key))
                {
                    current_task++;
                    if (current_task > 6) current_task = 2;
                }

                if (app_key_take_pressed(&start_key))
                {
                    switch (current_task)
                    {
                        case 2: task_2_start(); sys_state = TASK_2_RUN; break;
                        case 3: task_3_start(); sys_state = TASK_3_RUN; break;
                        case 4: task_4_start(); sys_state = TASK_4_RUN; break;
                        case 5: task_5_start(); sys_state = TASK_5_RUN; break;
                        case 6: task_6_start(); sys_state = TASK_6_RUN; break;
                        default: break;
                    }
                }
                break;

            case TASK_2_RUN:  task_2_update();  break;
            case TASK_3_RUN:  task_3_update();  break;
            case TASK_4_RUN:  task_4_update();  break;
            case TASK_5_RUN:  task_5_update();  break;
            case TASK_6_RUN:  task_6_update();  break;

            case TASK_2_DONE:
                if (app_key_take_pressed(&start_key))
                { current_task = 3; sys_state = TASK_SELECT; }
                break;
            case TASK_3_DONE:
                if (app_key_take_pressed(&start_key))
                { current_task = 4; sys_state = TASK_SELECT; }
                break;
            case TASK_4_DONE:
                if (app_key_take_pressed(&start_key))
                { current_task = 5; sys_state = TASK_SELECT; }
                break;
            case TASK_5_DONE:
                if (app_key_take_pressed(&start_key))
                { current_task = 6; sys_state = TASK_SELECT; }
                break;
            case TASK_6_DONE:
                if (app_key_take_pressed(&start_key))
                { current_task = 2; sys_state = TASK_SELECT; }
                break;

            default:
                sys_state = TASK_SELECT;
                break;
        }

        oled_tick += LOOP_DT_MS;
        if (oled_tick >= OLED_UPDATE_MS)
        {
            oled_tick = 0;

            switch (sys_state)
            {
                case TASK_SELECT:
                    oled_show_task_select();
                    break;
                case TASK_2_RUN: case TASK_3_RUN:
                case TASK_4_RUN: case TASK_5_RUN: case TASK_6_RUN:
                    oled_show_task_run();
                    break;
                default:
                    oled_show_task_result();
                    break;
            }
        }

        delay_ms(LOOP_DT_MS);
    }
}

#include "track_control.c"
