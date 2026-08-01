# 大底盘四驱小车循迹控制优化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将现有突跳式循迹控制替换为按新数字帧更新的连续 PD、弯道速度调度、定向丢线恢复和终点主动制动，使 30 cm x 25 cm 四驱小车稳定完成小于 20 s 的整圈循迹。

**Architecture:** 新增不依赖 STM32 外设的 `track_controller` 纯 C 模块，输入八路黑线位图和帧间隔，输出误差、微分、阶段及左右轮目标速度。`track_control` 保留 UART 和电机适配职责，并实现非阻塞制动；`main` 只处理运行、停止、超时和显示。纯算法在电脑端用 GCC 运行传感器序列测试，最终工程在临时副本中用 Keil ARMCC 全量编译。

**Tech Stack:** STM32F103C8、Keil ARMCC 5.06、C99 兼容 C、八路串口循迹模块、UART2 四路电机驱动、MinGW GCC 主机测试。

## Global Constraints

- 保持 UART1 `PA9/PA10` 循迹模块、UART2 `PA2/PA3` 电机驱动和 OLED `PB8/PB9` 接线不变。
- 控制主循环周期保持 `10 ms`，控制算法仅在新的 `$D...#` 数字帧到达时更新。
- 不启用 MPU6050、磁力计、模拟量循迹或积分项。
- 初始参数固定为 `Kp=0.75`、`Kd=0.55`、最大转向 `140`。
- 直线目标速度 `210`，大误差最低目标速度 `145`，整圈目标小于 `20 s`。
- 终点需至少 4 路、左右半区均有黑色、探头跨度至少 5，并连续确认 2 帧。
- 丢线保持阶段 `120 ms`，最迟 `600 ms` 请求停车。
- 主动制动持续 `40 ms`，反向制动力按停车前逻辑速度的三分之一计算并限制为 `20-70`。
- 现有 `openmv/` 未提交修改不暂存、不改写、不包含在任何提交中。

---

## File Structure

- Create `user/track_controller.h`: 纯控制器公开类型和接口，无 STM32 头文件依赖。
- Create `user/track_controller.c`: 位置误差、PD、转向限幅、速度调度、丢线恢复、终点识别和制动力计算。
- Create `tests/track_controller_test.c`: 可在电脑上直接运行的传感器序列回归测试。
- Modify `user/track_control.c`: 稳定发布数字帧、只消费新帧、调用控制器、执行电机斜坡和非阻塞制动。
- Modify `user/track_control.h`: 新的周期更新、启动、停止和状态接口。
- Modify `user/main.c`: 使用控制器终点判定和停止请求，显示当前控制阶段。
- Modify `user/Project.uvprojx`: 将新增控制器源文件和头文件加入 `user` 组。
- Modify `pid_tuning_log.md`: 记录新基准参数、调参顺序和首轮实车记录表。
- Modify `openmv/code_generate/hardware_wiring.md`: 补充当前循迹阶段、终点和主动制动说明。

---

### Task 1: 连续位置误差、PD 和速度调度核心

**Files:**
- Create: `user/track_controller.h`
- Create: `user/track_controller.c`
- Create: `tests/track_controller_test.c`

**Interfaces:**
- Consumes: `uint8_t bits`，bit 0-7 分别对应从左到右的八路探头，`1` 表示检测到黑线；`uint16_t dt_ms` 表示相邻新数字帧间隔。
- Produces: `track_controller_init()`、`track_controller_reset()`、`track_controller_set_gains()`、`track_controller_get_gains()`、`track_controller_step()` 和 `Track_Controller_Output_t`。

- [ ] **Step 1: 写出公开接口和第一组失败测试**

创建 `user/track_controller.h`，公开以下准确类型和函数：

```c
#ifndef __TRACK_CONTROLLER_H__
#define __TRACK_CONTROLLER_H__

#include <stdint.h>

typedef enum
{
    TRACK_PHASE_STRAIGHT = 0,
    TRACK_PHASE_CURVE,
    TRACK_PHASE_RECOVERY_HOLD,
    TRACK_PHASE_RECOVERY_SEARCH,
    TRACK_PHASE_LOST_STOP
} Track_Controller_Phase_t;

typedef struct
{
    int kp_x100;
    int kd_x100;
} Track_Controller_Gains_t;

typedef struct
{
    int error;
    int last_error;
    int derivative;
    int turn;
    int base_speed;
    int8_t last_direction;
    uint8_t curve_enter_frames;
    uint8_t curve_exit_frames;
    uint8_t center_frames;
    uint8_t finish_frames;
    uint16_t lost_ms;
    uint16_t exit_hold_ms;
    Track_Controller_Gains_t gains;
    Track_Controller_Phase_t phase;
    uint8_t finish_detected;
    uint8_t stop_requested;
} Track_Controller_t;

typedef struct
{
    int error;
    int derivative;
    int turn;
    int base_speed;
    int left_speed;
    int right_speed;
    uint8_t active_count;
    Track_Controller_Phase_t phase;
    uint8_t finish_detected;
    uint8_t stop_requested;
} Track_Controller_Output_t;

void track_controller_init(Track_Controller_t *controller);
void track_controller_reset(Track_Controller_t *controller);
void track_controller_set_gains(Track_Controller_t *controller, int kp_x100, int kd_x100);
Track_Controller_Gains_t track_controller_get_gains(const Track_Controller_t *controller);
Track_Controller_Output_t track_controller_step(Track_Controller_t *controller,
                                                uint8_t bits,
                                                uint16_t dt_ms);
int track_controller_brake_speed(int speed);

#endif
```

创建 `tests/track_controller_test.c`。测试框架只使用标准库，失败时打印行号并返回非零：

```c
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
```

- [ ] **Step 2: 建立可链接的空实现并确认行为断言失败**

先运行一次编译，确认测试已经引用尚未实现的真实接口；随后创建仅用于进入行为 RED 阶段的 `user/track_controller.c`：

```c
#include "track_controller.h"
#include <string.h>

void track_controller_init(Track_Controller_t *c) { memset(c, 0, sizeof(*c)); }
void track_controller_reset(Track_Controller_t *c) { Track_Controller_Gains_t g = c->gains; memset(c, 0, sizeof(*c)); c->gains = g; }
void track_controller_set_gains(Track_Controller_t *c, int kp, int kd) { c->gains.kp_x100 = kp; c->gains.kd_x100 = kd; }
Track_Controller_Gains_t track_controller_get_gains(const Track_Controller_t *c) { return c->gains; }
Track_Controller_Output_t track_controller_step(Track_Controller_t *c, uint8_t bits, uint16_t dt_ms)
{
    Track_Controller_Output_t out;
    (void)c; (void)bits; (void)dt_ms;
    memset(&out, 0, sizeof(out));
    return out;
}
int track_controller_brake_speed(int speed) { (void)speed; return 0; }
```

Run:

```powershell
$testExe = Join-Path $env:TEMP 'track_controller_test.exe'
gcc -std=c99 -Wall -Wextra -Werror tests/track_controller_test.c user/track_controller.c -Iuser -o $testExe
& $testExe
```

Expected: 测试程序成功编译并运行后 FAIL，首个行为失败为 `a.error == 55` 或等价的连续转向断言。不得把缺文件、语法错误或链接错误当作 RED 证据。

- [ ] **Step 3: 实现最小连续控制核心**

在 `user/track_controller.c` 中实现：

```c
#include "track_controller.h"
#include <string.h>

#define TRACK_KP_DEFAULT_X100       75
#define TRACK_KD_DEFAULT_X100       55
#define TRACK_TURN_MAX             140
#define TRACK_TURN_RISE_STEP        18
#define TRACK_TURN_FALL_STEP        24
#define TRACK_D_INPUT_MAX           80
#define TRACK_CURVE_ENTER_ERROR     45
#define TRACK_CURVE_EXIT_ERROR      28
#define TRACK_CURVE_ENTER_D         35
#define TRACK_CURVE_EXIT_D          12
#define TRACK_CURVE_ENTER_FRAMES     2
#define TRACK_CURVE_EXIT_FRAMES      6
#define TRACK_EXIT_HOLD_MS          100

static const int track_sensor_weight[8] =
    {-160, -105, -55, -18, 18, 55, 105, 160};
```

核心行为必须按以下公式实现：

```c
error = sum_of_active_weights / active_count;
delta = clamp(error - last_error, -80, 80);
derivative = (2 * derivative + delta) / 3;
target_turn = (kp_x100 * error + kd_x100 * derivative) / 100;
target_turn = clamp(target_turn, -140, 140);
```

转向变化规则：同方向增大最多 18；同方向减小最多 24；目标反向时当前帧只能向 0 移动且不得跨过 0。弯道进入与退出使用设计文档中的 2 帧/6 帧迟滞。速度目标严格采用 `210/190/180/165/145`，降速每新帧最多 30，加速每新帧最多 8，退出弯道后的 100 ms 速度不得超过 190。左右轮输出使用：

```c
out.left_speed = out.base_speed + out.turn;
out.right_speed = out.base_speed - out.turn;
```

`track_controller_reset()` 清除动态状态但保留当前 gains；`track_controller_init()` 先清零，再设置 `75/55` 和直线基础速度 210。

- [ ] **Step 4: 运行核心测试并确认通过**

Run:

```powershell
$testExe = Join-Path $env:TEMP 'track_controller_test.exe'
gcc -std=c99 -Wall -Wextra -Werror tests/track_controller_test.c user/track_controller.c -Iuser -o $testExe
& $testExe
```

Expected: `track_controller core tests passed`，进程退出码为 0。

- [ ] **Step 5: 提交控制核心**

```powershell
git add -- user/track_controller.h user/track_controller.c tests/track_controller_test.c
git commit -m "feat: add continuous line tracking controller"
```

---

### Task 2: 丢线恢复、终点识别和制动力计算

**Files:**
- Modify: `user/track_controller.c`
- Modify: `tests/track_controller_test.c`

**Interfaces:**
- Consumes: Task 1 的 `track_controller_step()` 状态和输出。
- Produces: `TRACK_PHASE_RECOVERY_HOLD`、`TRACK_PHASE_RECOVERY_SEARCH`、`TRACK_PHASE_LOST_STOP`，以及 `finish_detected`、`stop_requested` 和 `track_controller_brake_speed()`。

- [ ] **Step 1: 增加失败测试**

在 `tests/track_controller_test.c` 增加并从 `main()` 调用：

```c
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

static void test_brake_speed_is_opposite_and_bounded(void)
{
    CHECK(track_controller_brake_speed(210) == -70);
    CHECK(track_controller_brake_speed(-150) == 50);
    CHECK(track_controller_brake_speed(45) == -20);
    CHECK(track_controller_brake_speed(20) == 0);
}
```

- [ ] **Step 2: 运行测试并确认新增行为失败**

Run:

```powershell
$testExe = Join-Path $env:TEMP 'track_controller_test.exe'
gcc -std=c99 -Wall -Wextra -Werror tests/track_controller_test.c user/track_controller.c -Iuser -o $testExe
& $testExe
```

Expected: FAIL，首个失败来自丢线阶段仍是直线阶段、终点未识别或制动力结果不匹配。

- [ ] **Step 3: 实现恢复和终点状态**

在 `user/track_controller.c` 增加准确常量：

```c
#define TRACK_RECOVERY_HOLD_MS       120
#define TRACK_RECOVERY_STOP_MS       600
#define TRACK_RECOVERY_HOLD_BASE      90
#define TRACK_RECOVERY_HOLD_TURN      80
#define TRACK_RECOVERY_SEARCH_BASE    45
#define TRACK_RECOVERY_SEARCH_TURN    70
#define TRACK_RECOVERY_CENTER_FRAMES   2
#define TRACK_FINISH_ACTIVE_MIN        4
#define TRACK_FINISH_SPAN_MIN          5
#define TRACK_FINISH_FRAMES            2
#define TRACK_BRAKE_INPUT_MIN          30
#define TRACK_BRAKE_OUTPUT_MIN         20
#define TRACK_BRAKE_OUTPUT_MAX         70
```

行为必须是：

- `bits == 0` 时累计 `lost_ms`；`lost_ms < 120` 输出 `base=90`、`turn=last_direction*80`。
- `120 <= lost_ms < 600` 输出 `base=45`、`turn=last_direction*70`，允许内侧轮变为 `-25`。
- `lost_ms >= 600` 进入 `TRACK_PHASE_LOST_STOP` 并置 `stop_requested=1`。
- 恢复状态中只有 `bits & 0x18` 连续 2 帧才退出恢复；退出时清空微分和转向，防止旧误差冲击。
- 终点候选要求 `count_bits(bits)>=4`、`bits&0x0F`、`bits&0xF0`，且最高与最低有效 bit 的索引差至少 5。
- 第一帧终点候选保持上一输出；第二帧置 `finish_detected=1`，但控制器不自行覆盖停车前的硬件实际速度。
- `track_controller_brake_speed(speed)` 对绝对值小于 30 返回 0，否则返回反号后的 `clamp(abs(speed)/3,20,70)`。

- [ ] **Step 4: 运行全部主机测试**

Run:

```powershell
$testExe = Join-Path $env:TEMP 'track_controller_test.exe'
gcc -std=c99 -Wall -Wextra -Werror tests/track_controller_test.c user/track_controller.c -Iuser -o $testExe
& $testExe
```

Expected: 所有测试通过，输出 `track_controller core tests passed`，GCC 无 warning。

- [ ] **Step 5: 提交恢复与终点行为**

```powershell
git add -- user/track_controller.c tests/track_controller_test.c
git commit -m "feat: add line recovery and finish detection"
```

---

### Task 3: 接入串口数字帧、电机斜坡和非阻塞制动

**Files:**
- Modify: `user/track_control.h`
- Modify: `user/track_control.c`
- Modify: `user/main.c`
- Modify: `user/Project.uvprojx`

**Interfaces:**
- Consumes: `track_controller_step()`、`track_controller_brake_speed()`、UART1 `$D...#` 和现有 `control_speed()`。
- Produces: `track_control_start()`、`track_follow_update(uint16_t dt_ms)`、`track_car_request_stop()`、`track_car_stop_update(uint16_t dt_ms)`、`track_car_stop_immediate()`、`track_car_is_braking()` 及扩展后的 `Track_Info_t`。

- [ ] **Step 1: 先修改头文件建立预期失败的新接口契约**

在 `user/track_control.h`：

```c
#include "track_controller.h"

typedef struct
{
    uint8_t raw;
    uint8_t bits;
    uint8_t active_count;
    uint16_t no_frame_ms;
    uint16_t frame_count;
    uint16_t d_frame_count;
    uint16_t a_frame_count;
    uint16_t lost_ms;
    int error;
    int derivative;
    int turn;
    int base_speed;
    int left_speed;
    int right_speed;
    int kp_x100;
    int kd_x100;
    Track_Controller_Phase_t phase;
    uint8_t finish_detected;
    uint8_t stop_requested;
    uint8_t braking;
    uint16_t analog[8];
} Track_Info_t;

void track_control_start(void);
void track_follow_update(uint16_t dt_ms);
void track_car_request_stop(void);
void track_car_stop_update(uint16_t dt_ms);
void track_car_stop_immediate(void);
uint8_t track_car_is_braking(void);
```

删除旧的 `track_follow_update(void)` 和 `track_car_stop(void)` 声明，保留 `track_car_drive()`、读取接口和 PD 调参接口。

- [ ] **Step 2: 用 Keil 编译确认调用方尚未迁移而失败**

在临时目录复制工程后编译，避免修改仓库内 `Objects/` 和 `Listings/`：

```powershell
$buildRoot = Join-Path $env:TEMP ('problem-h-track-red-build-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
Copy-Item user,code,ml_libs,sys -Destination $buildRoot -Recurse -Force
$proc = Start-Process -FilePath 'E:\Keil5MDK\UV4\UV4.exe' `
    -ArgumentList @('-b','Project.uvprojx','-j0','-o','red-build.log') `
    -WorkingDirectory (Join-Path $buildRoot 'user') -WindowStyle Hidden -Wait -PassThru
Get-Content -LiteralPath (Join-Path $buildRoot 'user\red-build.log')
```

Expected: FAIL，`main.c` 报告 `track_follow_update` 参数不匹配或 `track_car_stop` 未声明。这是本任务的接口接入 RED 证据；本任务结束前必须迁移调用方并恢复全工程可编译状态。

- [ ] **Step 3: 将 UART 接收改成原子发布完整数字帧**

在 `user/track_control.c` 使用：

```c
static volatile uint8_t track_pending_bits = 0;
static volatile uint16_t track_d_sequence = 0;
static uint16_t track_consumed_sequence = 0;
static uint16_t track_frame_elapsed_ms = 0;
static Track_Controller_t track_controller;
static Track_Controller_Output_t track_output;
```

解析 `$D...#` 时先在局部变量中构造完整位图，全部 8 路解析完成后按顺序写入 `track_pending_bits`，最后递增 `track_d_sequence`。不要再让主循环逐个读取 ISR 正在改写的 `ir_data_number[]`。

`track_control_init()` 初始化 UART 后调用 `track_controller_init()`。`track_control_start()` 调用 `track_controller_reset()`，清除上一圈的终点、丢线、误差、转向、帧超时和制动状态，但保留运行时设置的 Kp/Kd；同时把 `track_consumed_sequence` 对齐到当前 `track_d_sequence`，等待下一帧新数据再驱动车轮。

`track_follow_update(dt_ms)` 每次累计 `track_frame_elapsed_ms`，只有 `track_d_sequence != track_consumed_sequence` 时复制位图并调用：

```c
track_output = track_controller_step(&track_controller,
                                     bits,
                                     track_frame_elapsed_ms);
```

随后清零帧计时并更新 `Track_Info_t`。无新数字帧时保持上一电机目标，超过 `1000 ms` 无数字帧则置 `stop_requested=1`。

- [ ] **Step 4: 用控制器输出替换旧的强制猛转和人工摆动代码**

删除 `TRACK_MIN_EDGE_TURN`、`TRACK_WIDE_MIN_TURN`、`TRACK_WEAVE_*`、旧恢复投票和 `keep_min_turn_for_error()` 等不再调用的逻辑。正常帧仅在 `finish_detected==0 && stop_requested==0` 时执行：

```c
track_car_drive(track_output.left_speed, track_output.right_speed);
```

PD 运行时调参改为直接调用 `track_controller_set_gains()`；Kp 限制为 `0-300`，Kd 限制为 `0-200`。`track_pd_get()` 和 `Track_Info_t.kp_x100/kd_x100` 均从 `track_controller_get_gains()` 返回真实参数，禁止保留第二份参数变量。

- [ ] **Step 5: 实现非对称电机斜坡和非阻塞主动制动**

定义：

```c
#define DRIVE_ACCEL_STEP       35
#define DRIVE_DECEL_STEP       80
#define TRACK_BRAKE_MS         40
```

`track_car_drive()` 对同方向增大绝对速度使用 35，对减速或换向先向零点移动 80，并且单帧不得跨过零点。正常电机映射继续使用现有四驱方向和 `TRACK_TRIM=10`。

`track_car_request_stop()` 只在未制动时保存 `drive_left_now/right_now`，通过 `track_controller_brake_speed()` 计算左右制动力并立即发送。`track_car_stop_update(dt_ms)` 累计 40 ms 后发送 `control_speed(0,0,0,0)` 并清零斜坡状态。`track_car_stop_immediate()` 用于上电初始化，直接清零且不反向制动。

- [ ] **Step 6: 主机测试和单文件 ARMCC 编译检查**

先将 `track_controller.c/.h` 加入 `user/Project.uvprojx` 的 `user` 组：

```xml
<File>
  <FileName>track_controller.c</FileName>
  <FileType>1</FileType>
  <FilePath>.\track_controller.c</FilePath>
</File>
<File>
  <FileName>track_controller.h</FileName>
  <FileType>5</FileType>
  <FilePath>.\track_controller.h</FilePath>
</File>
```

对 `user/main.c` 做最小编译迁移：`track_follow_update()` 改为 `track_follow_update(LOOP_DT_MS)`，上电和旧停止位置暂时改用 `track_car_stop_immediate()`；完整的停止请求和 OLED 流程留在 Task 4。

Run:

```powershell
$testExe = Join-Path $env:TEMP 'track_controller_test.exe'
gcc -std=c99 -Wall -Wextra -Werror tests/track_controller_test.c user/track_controller.c -Iuser -o $testExe
& $testExe
& 'E:\Keil5MDK\ARM\ARMCC\Bin\Armcc.exe' --c99 -c user/track_controller.c -Iuser -o (Join-Path $env:TEMP 'track_controller.o')
```

再执行完整 Keil 构建：

```powershell
$buildRoot = Join-Path $env:TEMP ('problem-h-track-task3-build-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
Copy-Item user,code,ml_libs,sys -Destination $buildRoot -Recurse -Force
$proc = Start-Process -FilePath 'E:\Keil5MDK\UV4\UV4.exe' `
    -ArgumentList @('-b','Project.uvprojx','-j0','-o','task3-build.log') `
    -WorkingDirectory (Join-Path $buildRoot 'user') -WindowStyle Hidden -Wait -PassThru
$buildLog = Get-Content -Raw -LiteralPath (Join-Path $buildRoot 'user\task3-build.log')
$buildLog
if ($buildLog -notmatch '0 Error\(s\)') { exit 1 }
```

Expected: 主机测试通过，ARMCC 单文件编译为 0 error，Keil 全工程链接为 `0 Error(s)`。

- [ ] **Step 7: 提交硬件适配层**

```powershell
git add -- user/track_control.h user/track_control.c user/main.c user/Project.uvprojx
git commit -m "feat: integrate frame-driven tracking and active braking"
```

---

### Task 4: 主程序、OLED 和 Keil 工程接入

**Files:**
- Modify: `user/main.c`

**Interfaces:**
- Consumes: Task 3 的周期更新、停止请求和 `Track_Info_t`。
- Produces: 运行/制动流程、终点停车、20 s 超时、阶段显示，以及包含新模块的可编译 Keil 工程。

- [ ] **Step 1: 迁移主程序停止流程**

在 `user/main.c` 增加：

```c
static void request_run_stop(void)
{
    if (running)
    {
        running = 0;
        track_car_request_stop();
    }
}
```

启动时只有 `track_car_is_braking()==0` 才允许：

```c
track_control_start();
elapsed_ms = 0;
running = 1;
```

运行时调用 `track_follow_update(LOOP_DT_MS)`。当 `info.finish_detected`、`info.stop_requested` 或 `elapsed_ms >= TIME_LIMIT_MS` 任一成立时调用 `request_run_stop()`。停止状态每个循环调用 `track_car_stop_update(LOOP_DT_MS)`，不得重复启动制动计时。上电初始化使用 `track_car_stop_immediate()`。

- [ ] **Step 2: 更新 OLED 状态**

第一行在 RUN/STOP 后显示阶段字符：

```c
static char phase_char(Track_Controller_Phase_t phase)
{
    switch (phase)
    {
        case TRACK_PHASE_CURVE:          return 'C';
        case TRACK_PHASE_RECOVERY_HOLD:  return 'H';
        case TRACK_PHASE_RECOVERY_SEARCH:return 'R';
        case TRACK_PHASE_LOST_STOP:      return 'L';
        default:                         return 'S';
    }
}
```

第二行继续显示 `error/turn`，制动期间阶段位置显示 `B`。第四行每 1 s 交替显示两页：`F<数字帧数> L<丢线毫秒>` 与 `P<Kp x100> D<Kd x100>`，保证诊断信息和实际生效参数都可见。不要改变 OLED 引脚或驱动。

- [ ] **Step 3: 在临时副本中全量编译 Keil 工程**

Run:

```powershell
$buildRoot = Join-Path $env:TEMP ('problem-h-track-green-build-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
Copy-Item user,code,ml_libs,sys -Destination $buildRoot -Recurse -Force
$proc = Start-Process -FilePath 'E:\Keil5MDK\UV4\UV4.exe' `
    -ArgumentList @('-b','Project.uvprojx','-j0','-o','green-build.log') `
    -WorkingDirectory (Join-Path $buildRoot 'user') -WindowStyle Hidden -Wait -PassThru
$buildLog = Get-Content -Raw -LiteralPath (Join-Path $buildRoot 'user\green-build.log')
$buildLog
if ($buildLog -notmatch '0 Error\(s\)') { exit 1 }
```

Expected: 链接成功并包含 `0 Error(s)`；新增文件不得产生 warning。旧文件既有的换行警告单独记录，不计为新增问题。

- [ ] **Step 4: 重新运行主机算法测试**

```powershell
$testExe = Join-Path $env:TEMP 'track_controller_test.exe'
gcc -std=c99 -Wall -Wextra -Werror tests/track_controller_test.c user/track_controller.c -Iuser -o $testExe
& $testExe
```

Expected: 全部测试通过。

- [ ] **Step 5: 提交主程序流程**

```powershell
git add -- user/main.c
git commit -m "feat: use smooth tracking flow in main program"
```

---

### Task 5: 文档同步、最终验证和实车交接

**Files:**
- Modify: `pid_tuning_log.md`
- Modify: `openmv/code_generate/hardware_wiring.md`

**Interfaces:**
- Consumes: 最终控制参数、OLED 阶段字符和制动流程。
- Produces: 与实际代码一致的调参基准、接线文档行为说明和完整验证记录。

- [ ] **Step 1: 更新 PID 调参基准**

将 `pid_tuning_log.md` 的当前基准改为：

```markdown
## 连续 PD 版本基准

- Kp: 75（0.75）
- Kd: 55（0.55）
- 直线速度: 210
- 弯道速度: 190 / 180 / 165 / 145
- 最大转向: 140
- 调参顺序: Kp -> Kd -> 转向变化率 -> 速度表 -> 制动时间
```

追加实车记录列：圈速、直线摆动、入弯、出弯、丢线、停车距离。保留旧参数记录作为对照，不删除历史数据。

- [ ] **Step 2: 更新硬件与 OLED 行为说明**

在 `hardware_wiring.md` 最终接线部分后追加当前循迹阶段：`S=直线`、`C=弯道`、`H=短时保持方向`、`R=定向找线`、`L=丢线停车`、`B=主动制动`。明确终点采用 2 帧宽线确认，接线仍是 UART1 循迹、UART2 电机。

- [ ] **Step 3: 完成源代码静态检查**

Run:

```powershell
git diff --check
Select-String -Path user\track_control.c -Pattern 'TRACK_WEAVE|TRACK_MIN_EDGE_TURN|TRACK_WIDE_MIN_TURN|keep_min_turn_for_error'
```

Expected: `git diff --check` 无输出；旧人工摆动和强制跳变符号搜索无结果。

- [ ] **Step 4: 完成最终自动验证**

先运行主机测试，再按 Task 4 的临时副本命令全量编译 Keil 工程。必须读取完整输出并确认：

- 主机测试退出码 0。
- GCC `-Wall -Wextra -Werror` 无 warning。
- Keil 构建为 `0 Error(s)`。
- `git status --short` 中只包含本任务文件和用户原有 `openmv/` 修改，没有临时 `.exe/.o` 或构建产物。

- [ ] **Step 5: 架空四轮硬件检查**

烧录前先架空车轮，依次确认：

1. 中心线时四轮同向前进。
2. 黑线偏右时左轮快、右轮慢；偏左时相反。
3. 从右偏切换到左偏时，轮速差经过零点，不直接反打。
4. 请求停车后四轮先出现约 40 ms 的小幅反向制动，再保持零速。
5. 若任一轮在制动时继续正转，立即断电，并只修正逻辑左右轮到四路电机的映射，不增加制动力。

- [ ] **Step 6: 实车分阶段验收**

按固定顺序测试并写入 `pid_tuning_log.md`：

1. 3 m 直线：不得出现幅度持续增大的左右摆动。
2. 单个入弯：相邻探头出现偏差时开始转向，不等最外侧探头。
3. 单个出弯：进入直线后不得斜行到最边缘才反打。
4. 连续弯：不得丢线；只允许一次修改 Kp 或 Kd 后重测。
5. 完整一圈：计时小于 20 s。
6. 终点宽线：记录停车距离，并与旧版本比较。

- [ ] **Step 7: 提交文档与调参基准**

```powershell
git add -- pid_tuning_log.md openmv/code_generate/hardware_wiring.md
git commit -m "docs: record smooth tracking tuning workflow"
```

---

## Final Review Checklist

- [ ] 每个新增控制行为都先运行过失败测试，再实现并转绿。
- [ ] 纯控制模块不包含 `headfile.h`、STM32 寄存器或 UART 调用。
- [ ] 控制算法只消费新的数字帧，模拟帧不会重置数字帧超时。
- [ ] 不存在人工摆动、外侧强制 120/200/245 转向或丢线长时间直走。
- [ ] 所有正常转向都限制在 `[-140,140]`，反向必须经过零点。
- [ ] 宽黑线终点不会被 4 个相邻弯道探头误触发。
- [ ] 主动制动非阻塞且只启动一次。
- [ ] `Project.uvprojx` 包含 `track_controller.c/.h`。
- [ ] 主机测试、ARMCC 编译和 Keil 全量构建均有新鲜通过证据。
- [ ] 用户原有 OpenMV 修改未被暂存、提交或覆盖。
