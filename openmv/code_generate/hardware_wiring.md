# Hardware Wiring

本文件用于记录当前小车工程的硬件接线。后续每增加一个硬件模块，都在本文档末尾追加对应的接线说明。

## 当前硬件连接总览

### 电机驱动板

用途：通过串口控制四路电机驱动板，当前代码使用 `UART_2`，波特率 `115200`。

| STM32 引脚 | 电机驱动板接口 | 说明 |
| --- | --- | --- |
| PA2 / UART2_TX | RX | STM32 发送控制命令到电机板 |
| PA3 / UART2_RX | TX | 电机板回传数据到 STM32，当前代码主要用于预留 |
| GND | GND | 必须共地 |

电源：

| 电源 | 连接 |
| --- | --- |
| 7.4V 电池正极 | 电机驱动板 VM/VIN/电机电源正 |
| 7.4V 电池负极 | 电机驱动板 GND，并与 STM32 GND 共地 |

注意：

- 当前电机驱动已改为串口控制，不再使用 `PB10/PB11` 连接电机板 I2C。5
- 串口线要交叉连接：STM32 `TX` 接电机板 `RX`，STM32 `RX` 接电机板 `TX`。
- 电机电源和 STM32 逻辑电源可以分开，但 `GND` 必须连接在一起。

### MPU6050 + 磁力计模块

用途：读取陀螺仪、加速度计和磁力计数据，用于角度闭环原地转向。

| STM32 引脚 | 模块接口 | 说明 |
| --- | --- | --- |
| PB10 | SCL | 软件 I2C 时钟 |
| PB11 | SDA | 软件 I2C 数据 |
| 3.3V | VCC | 模块供电 |
| GND | GND | 共地 |

注意：

- 当前代码中的传感器 I2C 使用 `PB10/PB11`。
- 若模块标注支持 `3.3V-5V`，优先使用 `3.3V`，避免 I2C 电平不匹配。
- 磁力计容易受电机、电池、大电流线干扰，安装时尽量远离电机和电源线。

### OLED 显示屏

用途：显示程序状态、当前角度、目标角度和误差。

| STM32 引脚 | OLED 接口 | 说明 |
| --- | --- | --- |
| PB8 | SCL | OLED 软件 I2C 时钟 |
| PB9 | SDA | OLED 软件 I2C 数据 |
| 3.3V | VCC/VDD | OLED 供电 |
| GND | GND | 共地 |

注意：

- OLED 与 MPU6050/磁力计使用不同的软件 I2C 引脚。
- 当前 OLED 地址代码使用 `0x78` 写地址，常见 7 位地址为 `0x3C`。

## 后续新增硬件记录模板

### 模块名称

用途：

| STM32 引脚 | 模块接口 | 说明 |
| --- | --- | --- |
|  |  |  |

电源：

| 电源 | 连接 |
| --- | --- |
|  |  |

注意：

- 

### 八路循迹模块

用途：读取黑线位置，当前代码使用 8 路数字量输入做巡线闭环。

| STM32 引脚 | 循迹模块接口 | 说明 |
| --- | --- | --- |
| PB12 | D1 | 最左侧传感器 |
| PB13 | D2 | 左侧传感器 |
| PB14 | D3 | 左中传感器 |
| PB15 | D4 | 中左传感器 |
| PA8 | D5 | 中右传感器 |
| PC13 | D6 | 右中传感器 |
| PC14 | D7 | 右侧传感器 |
| PC15 | D8 | 最右侧传感器 |

电源：
| 电源 | 连接 |
| --- | --- |
| 3.3V | VCC |
| GND | GND |

注意：
- 当前代码按 `0 = 检测到黑线`、`1 = 白底/未检测到黑线` 处理。
- OLED 第二行 `Sen` 显示 8 路传感器 bit，D1 对应最低位，D8 对应最高位。
- 如果模块明确标注只支持 5V 供电，VCC 可接 5V，但输出信号必须确认 STM32 引脚能安全读取；优先使用 3.3V 供电。
- PC13/PC14/PC15 驱动能力较弱但作为数字输入可以用，若 D6-D8 抖动明显，先检查接线、共地和模块输出电平。

### 八路循迹模块（串口版，当前代码使用）

用途：通过串口接收官方协议的 8 路循迹数字量数据，当前主程序解析 `$D,x1:0,x2:0,...,x8:0#`。

| STM32 引脚 | 循迹模块接口 | 说明 |
| --- | --- | --- |
| PB10 / UART3_TX | RX | STM32 向循迹模块发送控制命令 `$0,0,1#`，请求数字量数据 |
| PB11 / UART3_RX | TX | STM32 接收循迹模块回传数据 |
| 3.3V | VCC | 优先使用 3.3V 供电 |
| GND | GND | 必须与 STM32、电机驱动板共地 |

注意：
- 电机驱动板仍使用 UART2：PA2/TX -> 电机板 RX，PA3/RX -> 电机板 TX。
- 循迹模块不要再接 D1-D8 到 PB12/PB13/PB14/PB15/PA8/PC13/PC14/PC15。
- OLED 上 `F` 是收到的循迹数据帧计数；如果一直是 0，说明串口接线、波特率或模块输出模式不对。
- 官方数字量协议中 `0` 表示检测到黑线，`1` 表示白底。
## 当前最终接线：串口八路循迹 + 串口电机

本版本已经启用循迹代码，电机驱动板和八路循迹模块分别使用两个串口，避免互相占用。

### 电机驱动板（UART2）

| STM32 | 电机驱动板 | 说明 |
| --- | --- | --- |
| PA2 / USART2_TX | RX | STM32 发送速度命令到电机板 |
| PA3 / USART2_RX | TX | 电机板回传数据，可接可不接 |
| GND | GND | 必须共地 |
| 电池 7.4V | VM / 电机电源 | 给电机供电 |

注意：PA2、PA3 已经用于电机串口，不要再接八路循迹模块。

### 八路循迹模块（UART1）

| STM32 | 八路循迹模块 | 说明 |
| --- | --- | --- |
| PA9 / USART1_TX | RX | 发送配置命令，例如 `$0,1,1#` |
| PA10 / USART1_RX | TX | 接收 `$D...#` 和 `$A...#` 数据 |
| 3.3V | VCC | 优先使用 3.3V，避免 IO 电平风险 |
| GND | GND | 必须和 STM32、电机驱动板共地 |

协议说明：数字量中 `0` 表示检测到黑线，`1` 表示白底；程序里 OLED 的 `D` 会显示 8 路数字值，`Sen` 是程序识别出的黑线位图。

### OLED

| STM32 | OLED | 说明 |
| --- | --- | --- |
| PB8 | SCL | I2C 时钟 |
| PB9 | SDA | I2C 数据 |
| 3.3V | VCC | OLED 供电 |
| GND | GND | 共地 |

### MPU6050 / 磁力计预留

| STM32 | 模块 | 说明 |
| --- | --- | --- |
| PB10 | SCL | I2C 时钟，保留给姿态模块 |
| PB11 | SDA | I2C 数据，保留给姿态模块 |
| PA7 | MPU6050 INT | 姿态更新外部中断，用于刷新 yaw_gyro / yaw_Kalman |
| 3.3V | VCC | 模块供电 |
| GND | GND | 共地 |

PB10、PB11 是 I2C 预留线，不要再拿去接串口循迹模块。

### OLED 状态说明

当前操场形轨迹程序使用事件状态机，OLED 第一行 `S` 后面的数字表示：

| OLED | 状态 | 动作 |
| --- | --- | --- |
| S0 | SEEK_LINE | 找线，低速直走，检测到黑线立刻进入循迹 |
| S1 | FOLLOW_LINE | 沿黑线循迹 |
| S2 | EXIT_STOP | 连续确认出线后短暂停车 |
| S3 | TANGENT_ALIGN | 使用 MPU yaw 对出线切线方向做小角度微调 |
| S4 | TURN_STOP | 微调完成后短暂停车 |
| S5 | GAP_DRIVE | 空白区左右轮同速直走，重新捕获下一段线 |

入线确认 `LINE_ENTER_COUNT = 1`，检测到一次黑线就进入循迹；出线确认 `LINE_EXIT_COUNT = 20`，连续约 200ms 无黑线才认为真正出线。
当前版本不再出线后强制转 180 度。程序会在第一次检测到出线时记录 `target_yaw = yaw_gyro`，把它作为圆弧出口切线方向；S3 只做短时间小角度闭环微调。为避免陀螺仪误差在长直线中放大，S5 不再使用 yaw 保持，改为左右轮同速直走，由下一段黑线重新校正轨迹。
### Track PD tune keys

The current program can tune line-following PD values while running.
Change these pins in `user/app_board.h` if your wiring is different.

| STM32 | Key | Function |
| --- | --- | --- |
| PA6 | UP | Increase selected value |
| PB12 | DOWN | Decrease selected value |
| GND | Key common pin | Active low, key pressed = 0 |

OLED tune page:

| OLED | Meaning |
| --- | --- |
| `PD SET Kp/Kd` | Stop and tune before tracking |
| `PD RUN Kp/Kd` | Tracking is running |
| `Kp` / `Kd` | Current PD values, shown as real value |
| `E` / `T` / `L` | Track error / turn output / lost count |

Button usage:

| Key | Short press | Long press |
| --- | --- | --- |
| UP | Increase selected value in `PD SET` | Hold UP+DOWN to start |
| DOWN | Decrease selected value in `PD SET` | Hold UP+DOWN to start |
| UP+DOWN short press | Select Kp/Kd in `PD SET` | - |

`PD SET` page shows `U0 D0`; each value changes to `1` when that key is detected as pressed. PB1/MODE is not used by the current tuning flow.

### HC-SR04 ultrasonic ranging module

Default pins are defined in `user/app_board.h`.

| STM32 | HC-SR04 | Note |
| --- | --- | --- |
| PA4 | Trig | STM32 output trigger pulse |
| PA5 | Echo | Module echo input to STM32, must be divided to 3.3V |
| 5V | VCC | HC-SR04 power |
| GND | GND | Common ground with STM32 and motor driver |

Echo level note:

HC-SR04 Echo is usually 5V. Do not connect Echo directly to STM32 IO.
Use a divider, for example:

| Echo path | Resistor |
| --- | --- |
| HC-SR04 Echo -> STM32 PA5 | 1k |
| STM32 PA5 -> GND | 2k |

Test entry:

Call `hcsr04_test_run()` in `main()` after board initialization. OLED shows distance in mm, OK flag, and `OBS` obstacle flag. The test threshold is 200mm.
