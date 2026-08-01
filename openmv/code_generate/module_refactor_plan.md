# 模块化重构计划

## 目标

把当前工程整理成“模块单独能跑、接口清晰、引脚集中修改、方便移植”的结构。后续新增硬件时，先补接线文档和板级引脚配置，再写独立驱动和测试例程，最后再接入主流程。

## 现有库盘点

- OLED：已有 `ml_oled.c/h`，支持字符串、整数、带符号整数、十六进制、二进制、浮点显示。
- 串口：已有 `ml_uart.c/h`，支持 UART1/2/3 初始化和收发，后续补 printf 可配置重定向和帧解析工具。
- ADC：已有 `ml_adc.c/h`，支持单通道采集，后续补多通道扫描和滤波外壳。
- PWM：已有 `ml_pwm.c/h`，支持定时器 PWM 输出，后续补占空比工具和舵机角度接口。
- 定时器：已有 `ml_tim.c/h`，支持 ms 级定时中断初始化。
- 电机：已有 `motor.c/h`，当前使用串口电机板，后续补统一的正反转、调速、停止接口。
- I2C：已有 `ml_i2c.c/h`，当前用于 MPU6050、磁力计等模块。
- MPU6050/磁力计：已有 `ml_mpu6050.c/h`、`ml_hmc5883l.c/h`。

## 需要补齐

- `user/app_board.h`：全工程统一引脚配置，换线时优先改这里。
- `ml_libs/app_key.*`：按键扫描，带消抖、按下/松开事件。
- `ml_libs/app_adc_filter.*`：多通道 ADC 采集，一阶滤波，原始值/滤波值读取。
- `ml_libs/app_servo.*`：基于 PWM 的舵机控制，支持角度和脉宽。
- `ml_libs/app_spi_soft.*`：软件 SPI，方便后续接 SPI 传感器或屏幕。
- `ml_libs/app_encoder.*`：通用 AB 相编码器轮询测速接口，避免先占用中断资源。
- `user/app_motor.*`：当前小车串口电机板应用层，提供前进、后退、转向、停止接口。
- `user/app_module_test.*`：基础测试例程入口，OLED/UART/ADC/PWM/舵机/电机能单独验证。
- `user/*_test.c`：每个模块一个独立测试文件，只暴露一个 `xxx_test_run()` 入口，OLED 信息不超过三行。

## 清理策略

1. 先新增干净模块，不直接删除当前能运行的主流程。
2. 每个模块提供 `init/update/test` 形式接口，测试通过后再接入 `main.c`。
3. 旧的临时巡线、角度、串口解析代码，迁移到对应模块后再删除。
4. 所有新增硬件的接线说明同步写入 `hardware_wiring.md`。
5. 引脚、串口号、PWM 通道等硬件差异只放在 `app_board.h`。

## 第一阶段代码

- 新建 `app_board.h`。
- 通用模块已放入 `ml_libs`，并加入 `ml_libs/headfile.h`。
- `Project.uvprojx` 已加入新增 `.c/.h` 文件，Keil 工程可以直接看到这些模块。
- 新建按键、ADC 滤波、舵机、软件 SPI、编码器、模块测试接口。
- 新建串口帧解析 `ml_libs/app_uart_frame.*`，提供 `$...#` 这类协议的通用解析和 `app_uart_sendf()`。
- 新建电机应用层 `user/app_motor.*`，提供前进、后退、左右原地转、停止和左右差速接口。
- `ml_uart.c` 当前是非 UTF-8 文件，`printf` 硬重定向默认仍在 USART1；后续单独用编码安全方式改成可配置。
- 暂不编译，由你本地编译后把错误贴过来，我再逐项修。

## 测试入口说明

- OLED：调用 `oled_test_run()`，检查字符串、整数、浮点显示。
- UART：调用 `uart_test_run()`，检查串口发送和 `$...#` 帧解析。
- ADC：调用 `adc_filter_test_run()`，OLED 显示两个 ADC 滤波值。
- KEY：调用 `key_test_run()`，OLED 显示按键状态和计数。
- SERVO：调用 `servo_test_run()`，舵机在 0/90/180 度循环。
- SPI：调用 `spi_soft_test_run()`，软件 SPI 循环发送 `0x55` 并显示接收值。
- ENCODER：调用 `encoder_test_run()`，OLED 显示计数和 CPS。
- MOTOR：调用 `motor_test_run()`，依次前进、后退、左转、右转、停止。

测试时在 `main.c` 中包含 `app_module_test.h`，一次只调用一个测试函数，确认该模块正常后再换下一个。旧接口 `app_test_oled_basic()` 等仍保留为兼容包装。
