# 车载平衡滚球 - STM32F103C8T6 引脚分配

## 当前使用（纯循迹测试）

| 功能 | 引脚 | 外设 | 说明 |
|------|------|------|------|
| 循迹模块 TX | PA9 | UART1_TX | 8路红外循迹模块 |
| 循迹模块 RX | PA10 | UART1_RX | 8路红外循迹模块 |
| 电机驱动 TX | PA2 | UART2_TX | 电机驱动板 |
| 电机驱动 RX | PA3 | UART2_RX | 电机驱动板 |
| OLED SCL | PB8 | 软件I2C | 128x64 OLED |
| OLED SDA | PB9 | 软件I2C | 128x64 OLED |

## 待接线（后续功能）

| 功能 | 引脚 | 外设 | 说明 |
|------|------|------|------|
| 传感器 SCL | PB6 | 软件I2C | MPU6050 + HMC5883L |
| 传感器 SDA | PB7 | 软件I2C | MPU6050 + HMC5883L |
| MPU6050 INT | PA7 | EXTI7 | 数据就绪中断 |
| 摄像头 TX | PB10 | UART3_TX | OpenMV 球位置数据 |
| 摄像头 RX | PB11 | UART3_RX | OpenMV 球位置数据 |
| 舵机信号 | PB0 | TIM3_CH3 PWM | SC09舵机，控制摆杆 |
| 启动按键 | PB1 | GPIO | 低有效，任务启动 |
| 任务切换键 | PA6 | GPIO | 低有效，切换任务编号 |
| 舵机电传反馈 | PA1 | ADC1_CH1 | 舵机内部电位器，可选 |

## 预留（不接）

| 功能 | 引脚 | 说明 |
|------|------|------|
| HC-SR04 Trig | PA4 | 超声波，不用 |
| HC-SR04 Echo | PA5 | 超声波，不用 |
| 编码器A | PA0 | 不用 |
| 编码器B | PA1 | 不用（如需舵机ADC则用此脚）|

## 舵机接线

```
SC09 红线(正) → 5V降压模块输出
SC09 棕线(负) → 电池GND（与STM32共地）
SC09 黄线(信号) → STM32 PB0
```

## OpenMV接线

```
OpenMV P4(TX)  → STM32 PB11(UART3_RX)
OpenMV P5(RX)  → STM32 PB10(UART3_TX)
OpenMV GND     → STM32 GND
```

## 供电架构

```
2S锂电(7.4V) ──→ 5V降压模块 ──→ SC09舵机（红灯）
            │
            └──→ STM32 Vin ──→ 板载LDO → 3.3V（32+传感器）
            │
            └──→ 电机驱动板电源
```

## 与 car_example 原版的引脚变更

| 功能 | 原引脚 | 新引脚 | 原因 |
|------|--------|--------|------|
| I2C SCL | PB10 | PB6 | PB10/PB11腾给摄像头UART3 |
| I2C SDA | PB11 | PB7 | 同上 |
| 舵机 | TIM3_CH1(PA6) | TIM3_CH3(PB0) | PA6用作任务切换按键 |
