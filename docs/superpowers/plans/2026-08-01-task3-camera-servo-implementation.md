# 第三问摄像机与舵机协同控制 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 构建独立第三问固件，使小钢球从中心出发，实际进入 `+5 cm ±1 cm` 后折返，并在 `-5 cm ±1 cm` 稳定，总时间不超过 `4.8 s`。

**Architecture:** 新增不依赖 STM32 外设的 `task3_controller` 纯 C 状态机，负责位置/速度估计、启动门控、PD、舵机限幅、折返、稳定判定和故障处理；现有 `task3_control` 变为 OpenMV、SCS 舵机、OLED 和调试串口适配层。`main.c` 使用编译期常量默认构建独立第三问分支，同时保留当前第二问代码分支。

**Tech Stack:** STM32F103C8、OpenMV MicroPython、PA4/TIM4 软件串口 9600、SC09/SCS USART3 半双工 1 Mbps、Keil ARMCC 5.06、C99 兼容 C、MinGW GCC 主机测试。

## Global Constraints

- 第三问控制循环固定为 `10 ms`，OLED 和 UART1 诊断输出周期为 `100 ms`。
- OpenMV 数据协议保持 `$B,<whole_mm>#` 和 `$L#`，控制器内部位置单位为 `0.1 mm`。
- OpenMV 只使用 PA4 软件串口接收；USART3/PB10 只用于 SCS 舵机。
- 舵机 ID 为 `1`，中位 `740`，安全范围 `635..800`，每个控制周期命令变化不超过 `6`。
- 目标位置为 `+500` 和 `-500` 个 `0.1 mm`；正向、负向合格区均为目标 `±100` 个 `0.1 mm`。
- 启动前中心范围为 `±100` 个 `0.1 mm`，低速连续 `300 ms` 后才允许运行。
- 单个 `$L#` 不终止控制；连续 `150 ms` 没有可信位置才进入 `CAMERA_LOST`。
- 最终合格区和低速条件连续 `300 ms` 后完成；总超时固定为 `4800 ms`。
- 车轮电机在第三问固件中始终发送零速。
- 不调用旧 `camera.c`、`ball_balance.c`、`task3_openloop.c` 或 `task3_asym_move.c`。
- 保留当前基线中 `user/main.c`、`user/Project.uvprojx`、`user/track_control.c` 和 `user/track_control.h` 的第二问修改；不得回退或覆盖。
- 用户未要求创建 Git 提交；每个任务以差异检查和测试结果作为审查点，不自动提交。

---

## File Structure

- Create `code/task3_controller.h`: 纯控制器状态、配置、输入、输出和公开接口，不包含 `headfile.h`。
- Create `code/task3_controller.c`: 帧可信度、位置/速度估计、状态机、PD、限幅和超时。
- Create `tests/task3_controller_test.c`: 使用模拟摄像机帧验证第三问完整时序和安全边界。
- Modify `code/task3_control.h`: 保留任务入口，增加串口诊断接口。
- Modify `code/task3_control.c`: 只做 OpenMV收帧、控制器调用、SCS命令、OLED和UART诊断。
- Modify `user/main.c`: 增加默认启用的第三问独立编译分支，原第二问代码完整放入另一分支。
- Modify `user/Project.uvprojx`: 在现有 `code` 组追加 `task3_controller.c/.h`，不改变其他条目。

---

### Task 1: 纯第三问控制状态机

**Files:**
- Create: `code/task3_controller.h`
- Create: `code/task3_controller.c`
- Create: `tests/task3_controller_test.c`

**Interfaces:**
- Consumes: 每 `10 ms` 调用一次的 `task3_controller_update()`；有新 OpenMV 帧时传入 `has_frame=1`、`frame_valid` 和 `position_0p1mm`。
- Produces: `task3_controller_init()`、`task3_controller_request_start()`、`task3_controller_update()`、`task3_controller_get_output()` 和确定的 `task3_state_t`。

- [ ] **Step 1: 建立公开接口和可链接桩实现**

创建 `code/task3_controller.h`，使用以下准确接口：

```c
#ifndef __TASK3_CONTROLLER_H__
#define __TASK3_CONTROLLER_H__

#include <stdint.h>

typedef enum
{
    TASK3_WAIT_CAMERA = 0,
    TASK3_CENTER_READY,
    TASK3_GO_PLUS,
    TASK3_GO_MINUS,
    TASK3_HOLD_MINUS,
    TASK3_CAMERA_LOST,
    TASK3_TIMEOUT
} task3_state_t;

typedef struct
{
    uint16_t servo_neutral;
    uint16_t servo_min;
    uint16_t servo_max;
    uint16_t servo_step;
    int16_t kp_x100;
    int16_t kd_x100;
    int8_t servo_sign;
} task3_controller_config_t;

typedef struct
{
    uint8_t has_frame;
    uint8_t frame_valid;
    int16_t position_0p1mm;
} task3_controller_input_t;

typedef struct
{
    task3_state_t state;
    int16_t position_0p1mm;
    int16_t velocity_0p1mm_s;
    int16_t target_0p1mm;
    uint16_t servo_command;
    uint16_t task_elapsed_ms;
    uint16_t camera_age_ms;
    uint8_t camera_valid;
    uint8_t center_ready;
    uint8_t start_pending;
    uint8_t plus_reached;
    uint8_t completed;
} task3_controller_output_t;

typedef struct
{
    task3_controller_config_t config;
    task3_controller_output_t output;
    int16_t raw_position_0p1mm;
    int16_t previous_position_0p1mm;
    uint16_t frame_elapsed_ms;
    uint16_t center_stable_ms;
    uint16_t final_stable_ms;
    uint16_t phase_elapsed_ms;
    uint8_t have_position;
} task3_controller_t;

void task3_controller_init(task3_controller_t *controller,
                           const task3_controller_config_t *config);
void task3_controller_request_start(task3_controller_t *controller);
void task3_controller_update(task3_controller_t *controller,
                             const task3_controller_input_t *input,
                             uint16_t dt_ms);
task3_controller_output_t task3_controller_get_output(
    const task3_controller_t *controller);

#endif
```

创建 `code/task3_controller.c` 桩实现：初始化时复制配置、状态设为 `TASK3_WAIT_CAMERA`、舵机命令设为中位；请求启动只置 `start_pending=1`；更新函数暂不改变状态；getter 返回 `output`。该桩必须能用 GCC `-Wall -Wextra -Werror` 编译。

- [ ] **Step 2: 写启动门控和折返失败测试**

创建 `tests/task3_controller_test.c`，使用标准库 `CHECK` 宏，并定义固定配置：

```c
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
    for (i = 0; i < 20; ++i) push(c, 1, 0, 20);
}
```

测试必须从 `main()` 调用并覆盖：

```c
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
```

- [ ] **Step 3: 运行测试，确认桩实现发生行为失败**

Run:

```powershell
$exe = Join-Path $env:TEMP 'task3_controller_test.exe'
gcc -std=c99 -Wall -Wextra -Werror tests/task3_controller_test.c code/task3_controller.c -Icode -o $exe
& $exe
```

Expected: 编译成功，测试在 `TASK3_CENTER_READY`、`TASK3_GO_PLUS` 或折返断言处 FAIL；缺文件、语法错误和链接错误不能作为 RED 结果。

- [ ] **Step 4: 实现帧过滤、速度估计和状态机**

在 `code/task3_controller.c` 使用以下常量：

```c
#define T3_TARGET_PLUS                 500
#define T3_TARGET_MINUS              -500
#define T3_TARGET_TOL                  100
#define T3_CENTER_TOL                  100
#define T3_CENTER_SPEED_MAX            150
#define T3_FINAL_SPEED_MAX             200
#define T3_CENTER_STABLE_MS             300
#define T3_FINAL_STABLE_MS              300
#define T3_CAMERA_TIMEOUT_MS            150
#define T3_PLUS_TIMEOUT_MS             2200
#define T3_TOTAL_TIMEOUT_MS            4800
#define T3_POSITION_LIMIT              1250
#define T3_VELOCITY_LIMIT              5000
#define T3_JUMP_BASE                    120
#define T3_JUMP_PER_MS                    5
```

每次调用先饱和累加 `camera_age_ms` 和 `frame_elapsed_ms`。可信帧必须满足 `abs(position)<=1250`；已有位置时还需满足：

```c
abs(position - raw_position) <= 120 + 5 * min(frame_elapsed_ms, 250)
```

接受可信帧后执行：

```c
filtered = have_position ? (old_filtered + new_raw) / 2 : new_raw;
raw_velocity = (filtered - previous_filtered) * 1000 / frame_elapsed_ms;
velocity = (3 * old_velocity + clamp(raw_velocity, -5000, 5000)) / 4;
```

首次帧速度为零；`frame_elapsed_ms` 除数最低按 `1` 处理。可信帧将 `camera_age_ms` 清零。`$L#` 只是不产生可信帧，不立刻清除 `camera_valid`；`camera_age_ms >= 150` 才置无效并清除 `have_position`。

状态规则严格为：

- `WAIT_CAMERA`: 第一帧可信位置后进入 `CENTER_READY`。
- `CENTER_READY`: 目标为 0；位置 `±100` 且速度 `±150` 连续 300 ms 时置 `center_ready=1`。若 `start_pending=1`，立即进入 `GO_PLUS`。
- `GO_PLUS`: 目标为 500；只有最新原始可信位置位于 `[400,600]` 时置 `plus_reached=1` 并进入 `GO_MINUS`。原始位置 399 或 650 均不得折返。
- `GO_MINUS`: 目标为 -500；滤波位置位于 `[-600,-400]` 且速度绝对值不大于 200，连续 300 ms 后进入 `HOLD_MINUS` 并置 `completed=1`。
- `HOLD_MINUS`: 持续闭环保持 -500。
- 活动状态持续丢帧 150 ms 进入 `CAMERA_LOST`；舵机命令逐步回中。
- `GO_PLUS` 超过 2200 ms 或运行总时间达到 4800 ms 进入 `TIMEOUT`。
- 在 `GO_PLUS/GO_MINUS/HOLD_MINUS` 按键请求会中止运行，清除启动请求和估计器，回到 `WAIT_CAMERA`，舵机逐步回中。
- 在 `CAMERA_LOST/TIMEOUT` 请求启动只执行重新准备，不自动运行；用户在等待状态再次按键才设置启动请求。

- [ ] **Step 5: 实现整数 PD、限幅和命令变化率**

活动状态和 `CENTER_READY` 的目标命令使用：

```c
error = target_0p1mm - position_0p1mm;
correction = (kp_x100 * error - kd_x100 * velocity_0p1mm_s) / 1000;
desired = servo_neutral + servo_sign * correction;
desired = clamp(desired, servo_min, servo_max);
```

每次更新把 `servo_command` 向 `desired` 移动，单次最多 `servo_step=6`，不得越过目标。`WAIT_CAMERA/CAMERA_LOST/TIMEOUT` 的 `desired` 固定为 `servo_neutral`。所有乘法使用 `int32_t`，防止 ARMCC 16 位中间值溢出。

- [ ] **Step 6: 增加完整时序和安全测试**

在 `tests/task3_controller_test.c` 增加并调用以下测试场景：

- 单个无效帧后继续保持 `camera_valid=1`；累计到 149 ms 仍有效，达到 150 ms 后进入 `TASK3_CAMERA_LOST`。
- 从 `+500` 按 `+300,+100,-100,-300,-500` 的可信序列返回，并在 `-500` 保持超过 300 ms 后进入 `TASK3_HOLD_MINUS`。
- `GO_PLUS` 一直接收中心位置，运行达到 2200 ms 后进入 `TASK3_TIMEOUT`。
- 一帧从 0 跳到 1200 且间隔 20 ms 时位置保持原可信值，舵机不得突跳。
- 正向大误差下连续更新时，每次舵机命令增量不超过 6，最终值不超过 800；负向同理不低于 635。
- 运行中调用 `task3_controller_request_start()` 后回到安全准备状态，`start_pending=0`。

- [ ] **Step 7: 运行纯控制器全部测试**

Run:

```powershell
$exe = Join-Path $env:TEMP 'task3_controller_test.exe'
gcc -std=c99 -Wall -Wextra -Werror tests/task3_controller_test.c code/task3_controller.c -Icode -o $exe
& $exe
```

Expected: 输出 `task3_controller tests passed`，退出码 0，GCC 无 warning。

- [ ] **Step 8: 检查 Task 1 差异**

Run:

```powershell
git diff --check -- code/task3_controller.c code/task3_controller.h tests/task3_controller_test.c
git diff --stat -- code/task3_controller.c code/task3_controller.h tests/task3_controller_test.c
```

Expected: `diff --check` 无错误；差异只包含三个 Task 1 文件。

---

### Task 2: OpenMV、舵机、OLED和调试串口适配

**Files:**
- Modify: `code/task3_control.h`
- Modify: `code/task3_control.c`

**Interfaces:**
- Consumes: Task 1 的 `task3_controller_*`、`openmv_uart_read_ball()`、`scs_write_pos()`、OLED 和 `app_uart_sendf()`。
- Produces: `task3_init()`、`task3_start()`、`task3_update()`、`task3_show_oled()`、`task3_send_debug()` 和 `task3_get_state()`。

- [ ] **Step 1: 修改公开接口并确认旧实现尚未满足**

将 `code/task3_control.h` 改为包含 `task3_controller.h`，删除重复的 `task3_state_t` 定义，公开：

```c
void task3_init(void);
void task3_start(void);
void task3_update(uint16_t dt_ms);
void task3_show_oled(void);
void task3_send_debug(void);
task3_state_t task3_get_state(void);
task3_controller_output_t task3_get_output(void);
```

Run:

```powershell
Select-String -Path code\task3_control.c -Pattern 'task3_send_debug|task3_controller_update'
```

Expected: 修改前无匹配，证明硬件适配尚未接入新接口。

- [ ] **Step 2: 将旧控制计算替换为纯控制器调用**

`task3_init()` 按以下顺序执行：

```c
scs_init(BOARD_SCS_SERVO_ID);
scs_torque_enable(BOARD_SCS_SERVO_ID, 1);
openmv_uart_init();
task3_controller_init(&controller, &config);
scs_write_pos(BOARD_SCS_SERVO_ID,
              BOARD_BALANCE_SERVO_NEUTRAL, 0, 500);
```

硬件配置固定为 `{740,635,800,6,120,24,1}`，前三项从 `app_board.h` 宏读取。删除旧文件内重复的状态计时、速度估算、提前预测折返和 PD 计算，避免两套算法同时生效。

`task3_start()` 只调用 `task3_controller_request_start(&controller)`。

`task3_update(dt_ms)` 先清零输入的 `has_frame`，再排空 `openmv_uart_read_ball()` 队列，只保留序号最新的一帧并设置 `has_frame/frame_valid/position_0p1mm`。随后调用 `task3_controller_update()`，读取输出；舵机命令变化时立即发送，命令不变时每 `100 ms` 发送一次保持帧。

- [ ] **Step 3: 更新 OLED 和固定格式串口输出**

OLED 四行固定显示：

```text
T3 <WAIT/CENTER/+50/-50/HOLD/LOST/TIME>
X:<signed mm> T:<signed mm>
V:<signed mm/s> S:<servo>
C:<Y/N> t:<seconds.centiseconds>
```

`task3_send_debug()` 使用 UART1：

```c
app_uart_sendf(BOARD_UART_DEBUG,
    "T3,state=%u,t=%u,x=%d,v=%d,target=%d,servo=%u,cam=%u\r\n",
    (unsigned)out.state,
    (unsigned)out.task_elapsed_ms,
    (int)out.position_0p1mm,
    (int)out.velocity_0p1mm_s,
    (int)out.target_0p1mm,
    (unsigned)out.servo_command,
    (unsigned)out.camera_valid);
```

`task3_get_state()` 和 `task3_get_output()` 返回纯控制器快照，不公开可变全局变量。

- [ ] **Step 4: ARMCC 单文件编译检查**

Run:

```powershell
$obj = Join-Path $env:TEMP 'task3_controller.o'
& 'E:\Keil5MDK\ARM\ARMCC\Bin\Armcc.exe' --c99 -c code/task3_controller.c -Icode -o $obj
```

Expected: ARMCC 返回 0；纯控制器不依赖 STM32 头文件。

- [ ] **Step 5: 检查硬件边界**

Run:

```powershell
Select-String -Path code\task3_control.c -Pattern 'camera_init|ball_balance|task3_openloop|task3_asym_move|uart_init\(UART_3'
git diff --check -- code/task3_control.c code/task3_control.h
```

Expected: 禁用路径搜索无匹配，`diff --check` 无错误。

---

### Task 3: 独立第三问 main 和 Keil 工程接入

**Files:**
- Modify: `user/main.c`
- Modify: `user/Project.uvprojx`

**Interfaces:**
- Consumes: Task 2 的第三问入口、PB1按键、UART1调试、UART2电机驱动。
- Produces: 默认烧录后只运行第三问的 `main()` 和包含新控制器文件的 Keil 工程。

- [ ] **Step 1: 保存并核对第二问基线**

Run:

```powershell
git show HEAD:user/main.c | Select-String -Pattern 'track_follow_update|track_car_stop_immediate'
git show HEAD:user/Project.uvprojx | Select-String -Pattern 'track_controller.c|track_controller.h'
```

Expected: 能看到第二问 `track_follow_update(LOOP_DT_MS)`、`track_car_stop_immediate()` 和 `track_controller.c/.h` 工程条目；后续编辑必须保留这些内容。

- [ ] **Step 2: 在 main.c 增加编译期独立入口**

文件顶部定义：

```c
#define APP_STANDALONE_TASK3 1
```

当值为 1 时编译以下第三问主循环：

```c
#include "task3_control.h"

#define LOOP_DT_MS       10
#define DISPLAY_DT_MS   100

int main(void)
{
    app_key_t start_key;
    uint16_t display_ms = 0;

    uart_init(BOARD_UART_DEBUG, BOARD_UART_DEBUG_BAUD, 1);
    uart_init(BOARD_UART_MOTOR, BOARD_UART_MOTOR_BAUD, 1);
    control_speed(0, 0, 0, 0);
    delay_ms(50);
    motor_init();
    control_speed(0, 0, 0, 0);

    OLED_Init();
    OLED_Clear();
    app_key_init(&start_key, BOARD_START_KEY_PORT,
                 BOARD_START_KEY_PIN, BOARD_START_KEY_ACTIVE, 20);
    task3_init();

    while (1)
    {
        app_key_update(&start_key, LOOP_DT_MS);
        if (app_key_take_pressed(&start_key)) task3_start();
        task3_update(LOOP_DT_MS);

        display_ms += LOOP_DT_MS;
        if (display_ms >= DISPLAY_DT_MS)
        {
            display_ms = 0;
            task3_show_oled();
            task3_send_debug();
        }
        delay_ms(LOOP_DT_MS);
    }
}
```

当前第二问 include、显示辅助函数和 `main()` 整体放在 `#else` 分支，内容保持当前工作树版本；末尾用 `#endif` 结束。不得复制出第二个同时参与编译的 `main` 符号。

- [ ] **Step 3: 向工程文件追加纯控制器条目**

在 `user/Project.uvprojx` 现有 `code` 组、`task3_control.c/.h` 附近追加且只追加：

```xml
<File>
  <FileName>task3_controller.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\code\task3_controller.c</FilePath>
</File>
<File>
  <FileName>task3_controller.h</FileName>
  <FileType>5</FileType>
  <FilePath>..\code\task3_controller.h</FilePath>
</File>
```

保留用户已加入的 `user/track_controller.c/.h` 条目。

- [ ] **Step 4: 在临时副本中全量构建 Keil 工程**

Run:

```powershell
$buildRoot = Join-Path $env:TEMP ('problem-h-task3-build-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
Copy-Item user,code,ml_libs,sys -Destination $buildRoot -Recurse -Force
$proc = Start-Process -FilePath 'E:\Keil5MDK\UV4\UV4.exe' `
    -ArgumentList @('-b','Project.uvprojx','-j0','-o','task3-build.log') `
    -WorkingDirectory (Join-Path $buildRoot 'user') `
    -WindowStyle Hidden -Wait -PassThru
$log = Get-Content -Raw -LiteralPath (Join-Path $buildRoot 'user\task3-build.log')
$log
if ($log -notmatch '0 Error\(s\)') { exit 1 }
```

Expected: 链接日志包含 `0 Error(s)`，新增文件无 warning。构建产物只存在于临时目录。

- [ ] **Step 5: 检查 main 和工程差异**

Run:

```powershell
git diff --check -- user/main.c user/Project.uvprojx
Select-String -Path user\Project.uvprojx -Pattern 'task3_controller.c|task3_controller.h|track_controller.c|track_controller.h'
```

Expected: 两套控制器各有一份 `.c` 和一份 `.h` 条目；无重复条目、无空白错误。

---

### Task 4: 最终自动验证与实机交接

**Files:**
- Verify: `openmv/main.py`
- Verify: `code/task3_controller.c/.h`
- Verify: `code/task3_control.c/.h`
- Verify: `user/main.c`
- Verify: `user/Project.uvprojx`

**Interfaces:**
- Consumes: 最终固件、现有 OpenMV 检测程序和真实硬件接线。
- Produces: 主机测试、OpenMV静态测试、Keil构建证据及固定顺序的现场调参步骤。

- [ ] **Step 1: 运行全部自动测试**

Run:

```powershell
$exe = Join-Path $env:TEMP 'task3_controller_test.exe'
gcc -std=c99 -Wall -Wextra -Werror tests/task3_controller_test.c code/task3_controller.c -Icode -o $exe
& $exe
python -m pytest tests/test_openmv_main_static.py -q
```

Expected: C 测试输出 `task3_controller tests passed`；OpenMV 静态测试全部通过。

- [ ] **Step 2: 重新进行新鲜 Keil 全量构建**

重新执行 Task 3 的临时副本构建命令，使用新的 GUID 目录。必须读取完整日志并确认 `0 Error(s)`；不得复用旧构建结果。

- [ ] **Step 3: 完成静态边界检查**

Run:

```powershell
git diff --check
Select-String -Path code\task3_controller.c,code\task3_controller.h -Pattern 'headfile|stm32|uart_|scs_|OLED_'
Select-String -Path user\main.c -Pattern '#define APP_STANDALONE_TASK3 1'
git status --short
```

Expected: 纯控制器不包含硬件符号；第三问编译开关为 1；工作区没有 `.exe`、`.o`、`Objects/` 或 `Listings/` 新产物。基线中的第二问修改仍然保留。

- [ ] **Step 4: 上电前确认舵机方向与中位**

架空或固定水管，先不放钢球：

1. 上电后舵机应缓慢到 `740`，不得撞击机械限位。
2. 手动将小球放到图像正方向，观察终端 `x` 应为正。
3. 在安全支撑下确认舵机位置从 `740` 增大时，小球加速度方向为图像正方向。
4. 若方向相反，只把 `servo_sign` 从 `1` 改为 `-1`；不得交换坐标协议或同时修改 Kp 符号。
5. 若水平位置不是 `740`，先运行现有平衡标定程序得到新中位，再同步修改 `BOARD_BALANCE_SERVO_NEUTRAL`。

- [ ] **Step 5: 分阶段实机验证**

按固定顺序执行，每一步连续成功三次再进入下一步：

1. 中心门控：球离开中心超过 10 mm 时按 PB1，不得启动；放回中心稳定后自动开始。
2. 正向到达：临时托住负向行程，只检查球是否进入 `+40..+60 mm` 后状态切到 `GO_MINUS`。
3. 负向稳定：完整放行，检查最终位置连续 300 ms 位于 `-60..-40 mm`。
4. 丢帧保护：短暂遮挡不足 150 ms 时舵机不突跳；持续遮挡后状态显示 `LOST` 并回中。
5. 完整计时：记录 UART 中首次 `GO_PLUS` 到 `HOLD_MINUS` 的时间，必须小于 4800 ms。

- [ ] **Step 6: 固定调参顺序**

每次只修改一项，并连续测试三次：

1. 先校准 `BOARD_BALANCE_SERVO_NEUTRAL`，使球在中心附近静止。
2. 只调 `kp_x100`，使球能够到达两端；每次调整 10。
3. 保持 Kp，只调 `kd_x100` 抑制正向和负向过冲；每次调整 4。
4. 只调 `servo_step` 改变管道倾斜建立速度；每次调整 1。
5. 最后才调整中心/最终速度阈值，禁止通过放宽 `±10 mm` 位置合格区获得通过状态。

- [ ] **Step 7: 最终验收记录**

连续运行至少五次，串口保存每次 `t`、正向最大 `x`、最终稳定平均 `x`、是否出现 `LOST/TIMEOUT`。五次都满足：正向峰值 `40..60 mm`、最终 `-60..-40 mm`、完成时间 `<4800 ms`，才视为第三问完成。

---

## Final Review Checklist

- [ ] 纯控制器每个新增行为都先出现行为测试失败，再实现转绿。
- [ ] 正向折返使用最新原始可信位置，399 和 650 均不能触发，`400..600` 才能触发。
- [ ] 单帧 `$L#` 不改变运行状态，持续 150 ms 丢失才安全回中。
- [ ] 位置异常跳变不会驱动舵机，速度计算不存在除零。
- [ ] 舵机命令始终位于 `635..800`，每周期变化不超过 6。
- [ ] 第三问固件中 UART1、UART2、USART3 和 PA4 软件串口职责互不冲突。
- [ ] `APP_STANDALONE_TASK3` 为 1，第二问代码保留但不参与固件运行。
- [ ] `Project.uvprojx` 同时保留用户的 `track_controller` 条目并新增 `task3_controller` 条目。
- [ ] GCC测试、OpenMV静态测试、ARMCC检查和Keil全量构建都有新鲜通过证据。
- [ ] 实机方向、中位、正向到达、负向稳定、丢帧保护和五次完整计时均按顺序验证。
