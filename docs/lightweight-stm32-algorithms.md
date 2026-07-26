# 在 STM32 上搭一条轻量、可靠的传感器控制链

在资源有限的 MCU 上，“算法先进”不一定等于“系统好用”。真正决定产品体验的，
往往是数据链条中的几个小环节：传感器毛刺有没有被挡住，噪声有没有被平滑，
姿态是否会随时间漂移，控制输出会不会突然跳变，串口数据出错时能不能被发现。

本文以本项目的轻量算法库为例，介绍如何用固定内存、无动态分配的 C 模块，
搭建一条适合 STM32 裸机或 RTOS 工程的处理链。

## 一、为什么要把小算法做成独立模块

很多 STM32 工程会把滤波、PID 和通信校验直接写在 `main.c` 或中断函数里。
项目小时看起来很快，功能增加后却容易出现三个问题：

1. 参数和状态散落在全局变量中，多个传感器无法复用同一套逻辑。
2. 算法与 HAL、外设句柄耦合，换芯片或做主机测试时很困难。
3. 边界条件没有统一处理，例如采样周期为零、窗口长度越界或 CRC 输入为空。

本库中的算法只接收数值和字节，不直接读 ADC、I2C、UART，也不调用 HAL。
外设负责“取得数据”，算法负责“处理数据”，控制层负责“使用结果”。这种边界
让每个模块可以单独测试，也能在 STM32F1、F4、G0、H7 等平台间复用。

## 二、常用轻量信号处理算法

### 1. EMA：只用一个状态量的低通滤波

指数移动平均（EMA）的公式为：

```text
y(k) = y(k-1) + α[x(k) - y(k-1)]
```

`α` 位于 0 到 1 之间。值越小，输出越平滑，但响应越慢；值越大，输出越快
跟随输入。它每次更新只需要少量加法和乘法，不需要保存历史数组，很适合温度、
电压、光照等慢变量。

```c
#include "filters.h"

EmaFilter temperature_filter;

EmaFilter_Init(&temperature_filter, 0.1f, first_temperature);

/* 固定周期调用 */
float temperature =
    EmaFilter_Update(&temperature_filter, raw_temperature);
```

如果采样频率改变，同一个 `α` 对应的截止特性也会改变。因此工程中应尽量保持
固定采样周期，或根据采样周期重新计算参数。

### 2. 滑动平均与三点中值：处理噪声和毛刺

滑动平均保存最近 N 个样本。本项目使用循环数组和累计和，因此每次更新是 O(1)，
不需要重新遍历整个窗口。

```c
MovingAverageFilter current_filter;

MovingAverage_Init(&current_filter, 8U);
float current = MovingAverage_Update(&current_filter, raw_current);
```

窗口越大，平滑效果越明显，同时延迟和 RAM 占用也越大。默认最大窗口为 16，
可通过 `MOVING_AVERAGE_MAX_WINDOW` 在编译时调整。

对于偶发的极大或极小毛刺，三点中值通常比平均值更有效：

```c
float cleaned = Median3_Filter(previous, current, next);
```

它只比较三个数，不需要数组或排序库。代价是必须暂存相邻样本，并会引入一个
采样周期左右的观察延迟。

### 3. 互补滤波：低成本融合陀螺仪与参考角

陀螺仪短期响应快，但积分后会漂移；加速度计计算的倾角长期稳定，却容易受到
振动影响。互补滤波将两者组合：

```text
angle = α(angle + gyro_rate × dt) + (1 - α)reference_angle
```

```c
#include "complementary_filter.h"

ComplementaryFilter roll_filter;
ComplementaryFilter_Init(&roll_filter, 0.98f, initial_roll);

float roll = ComplementaryFilter_Update(
    &roll_filter,
    gyro_roll_rate,
    accelerometer_roll,
    0.001f);
```

与完整卡尔曼滤波相比，互补滤波参数少、计算量低，很适合固定周期的倾角估计。
`dt` 必须使用秒，并应来自稳定的定时器周期。若系统需要动态估计噪声或同时处理
多个耦合状态，再考虑矩阵卡尔曼滤波。

### 4. 斜率限制器：保护执行器和电源

PID 输出即使已经限幅，也可能从一个极值瞬间跳到另一个极值。电机、电热丝、
阀门或 LED 电源不一定能安全承受这种变化。斜率限制器分别约束每秒最大上升量
和下降量：

```c
#include "slew_rate_limiter.h"

SlewRateLimiter heater_output;
SlewRateLimiter_Init(&heater_output,
                     20.0f,  /* 每秒最多上升 20% */
                     50.0f,  /* 每秒最多下降 50% */
                     0.0f);

float safe_output =
    SlewRateLimiter_Update(&heater_output, pid_output, 0.1f);
```

下降速率通常可以设得比上升速率更快，让系统既能软启动，也能在过温时迅速关闭。
安全保护逻辑仍应有独立的硬限幅或关断路径，不能只依赖斜率限制器。

### 5. CRC：发现通信链路中的位错误

项目提供无查表的 CRC‑8/SMBUS 与 CRC‑16/MODBUS。无查表实现比 256 项查表略慢，
但几乎不占常量表空间，适合低频短帧通信。

```c
#include "crc.h"

uint16_t crc = CRC16_Modbus(frame, frame_length);
frame[frame_length] = (uint8_t)(crc & 0xFFU);
frame[frame_length + 1U] = (uint8_t)(crc >> 8U);
```

注意 CRC 不是加密或身份认证。它适合发现噪声、丢位和帧内容损坏，不能阻止恶意
修改。协议双方还必须使用完全一致的多项式、初值、反射方式和字节顺序。

## 三、三种轻量控制算法

### 1. 回差开关控制：简单系统不必强上 PID

加热器、制冷器和液位泵常常只支持开关。若直接用 `measurement < setpoint`
控制，测量噪声会让继电器在设定点附近频繁动作。回差控制设置两个阈值：

```text
measurement <= setpoint - half_band  -> 高输出
measurement >= setpoint + half_band  -> 低输出
中间区域                              -> 保持上一次状态
```

这种控制器易于验证，并能显著减少继电器和压缩机启停次数。它的代价是被控量会在
回差带附近周期波动，因此不适合要求高精度连续输出的系统。

```c
HysteresisController heater;
HysteresisController_Init(&heater,
                          35.0f, 0.5f,
                          0.0f, 100.0f, 0U);
float power = HysteresisController_Update(&heater, temperature);
```

### 2. 二状态反馈：把已设计好的增益落到 MCU

对小车平衡、位置—速度控制等二状态系统，如果已经通过极点配置或 LQR 得到
`K1`、`K2`，运行时只需计算：

```text
u = Kr*r - K1*x1 - K2*x2
```

本模块提供输出限幅，但不负责计算反馈增益。增益应来自系统模型和离线设计；状态
单位、符号和采样位置必须与设计模型一致。

```c
StateFeedback2Controller controller;
StateFeedback2_Init(&controller, k1, k2, kr, max_output);
float u = StateFeedback2_Update(&controller,
                                reference, position, velocity);
```

### 3. 一阶 LADRC：在线估计并补偿总扰动

线性自抗扰控制（LADRC）把未知模型项和外部扰动合并为“总扰动”，由扩张状态
观测器估计，再在控制量中补偿。项目实现的是一阶对象版本，内部只有两个观测器
状态，计算量仍为 O(1)。

主要参数是：

- `plant_gain`（b0）：输入对输出作用强度的粗略估计，符号必须正确。
- `controller_bandwidth`：越大跟踪越快，但控制动作和噪声敏感度也更高。
- `observer_bandwidth`：通常高于控制带宽；过高会放大测量噪声并加剧离散误差。
- `dt_seconds`：固定控制周期，必须使用秒。

```c
FirstOrderLadrc ladrc;
FirstOrderLadrc_Init(&ladrc,
                     20.0f, 5.0f,
                     estimated_b0, 100.0f,
                     first_measurement);

float u = FirstOrderLadrc_Update(
    &ladrc, reference, measurement, control_period_seconds);
```

LADRC 不是“无需模型和整定”。当 b0 符号错误、采样周期不稳定或带宽接近离散系统
极限时，控制仍可能失稳。建议先用保守带宽和较低输出限幅，在仿真与低功率条件下
逐步放开。
## 四、组合成一条温度控制链

以恒温设备为例，一条实用的数据路径可以是：

```text
ADC/数字传感器
    -> 三点中值去毛刺
    -> EMA 平滑
    -> PID 计算
    -> 斜率限制
    -> PWM 驱动加热器
```

伪代码如下：

```c
float cleaned = Median3_Filter(sample_older, sample_previous, sample_now);
float temperature = EmaFilter_Update(&temperature_filter, cleaned);
float requested_power = PID_Calc(&heater_pid, setpoint, temperature);
float safe_power = SlewRateLimiter_Update(
    &heater_limiter, requested_power, control_period_seconds);

set_heater_pwm(safe_power);
```

如果温度和设定值需要通过串口下发，可在帧尾加入 CRC。这样，滤波、控制、执行器
保护和通信完整性各自负责一个清晰问题，调试时也容易定位。

## 五、在 STM32 工程中的使用原则

### 固定采样周期

PID、互补滤波和斜率限制器都与时间相关。推荐用硬件定时器或 RTOS 周期任务触发，
不要依靠执行时间不稳定的主循环延时。

### 中断只做必要工作

UART/ADC 中断中可以把数据放入环形缓冲区，但复杂处理最好留到主循环或任务。
如果中断与任务共享同一个状态对象，应使用平台提供的临界区；`volatile` 不能
自动让复合读写变得原子。

### 参数要有物理意义

- EMA 的 `α` 决定平滑与响应速度的平衡。
- 滑动平均窗口决定 RAM、噪声抑制和延迟。
- 互补滤波的 `α` 决定更相信陀螺仪还是参考角。
- 斜率限制单位是“输出单位/秒”，`dt` 的单位是秒。
- PID 系数应与实际控制周期匹配。

### 在上板前先做主机测试

算法不依赖 HAL，因此可以在 PC 上快速运行单元测试。本项目对标准 CRC 校验向量、
滤波响应、限幅行为和环形缓冲区回绕进行了测试。上板测试仍然必要，但主机测试
可以更早发现边界错误，并缩短烧录—观察—修改的循环。

## 六、如何选择

| 需求 | 推荐算法 | 运行代价 | 主要权衡 |
|---|---|---:|---|
| 平滑慢变化传感器 | EMA | O(1)，一个状态量 | 平滑越强，响应越慢 |
| 抑制随机噪声 | 滑动平均 | O(1)，N 个浮点数 | 窗口越大，延迟越大 |
| 去除偶发尖峰 | 三点中值 | 固定少量比较 | 需要相邻三个样本 |
| 低成本姿态融合 | 互补滤波 | O(1) | 依赖稳定采样周期 |
| 避免执行器突变 | 斜率限制器 | O(1) | 会延缓目标跟随 |
| 检查短帧损坏 | CRC‑8/CRC‑16 | O(8N) | 不是安全认证 |
| 单状态噪声估计 | 一维卡尔曼 | O(1) | 参数理解要求更高 |
| 回差开关控制 | Hysteresis | O(1) | 输出会在回差带内波动 |
| 二状态模型控制 | State Feedback | O(1) | 增益依赖系统模型 |
| 一阶抗扰控制 | LADRC | O(1) | b0、带宽和周期需谨慎整定 |
| 通用闭环控制 | PID | O(1) | 需要结合对象整定 |

轻量算法的价值不只是“运行得快”，而是状态少、边界清楚、容易验证。先用最简单
且足够的算法解决问题，只有当测量和实验表明它不够时，再增加模型复杂度，通常
能得到更可靠、更容易维护的嵌入式系统。
