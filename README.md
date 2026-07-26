# STM32 轻量级算法库

面向 STM32 和其他裸机/RTOS C 项目的轻量算法模块：

- 固定容量字节环形缓冲区
- 位置式与增量式 PID 控制器
- 一维标量卡尔曼滤波器
- EMA、滑动平均与三点中值滤波
- 陀螺仪/参考角互补滤波
- 执行器斜率限制器
- 无查表 CRC-8/SMBUS 与 CRC-16/MODBUS
- 回差开关、二状态反馈与一阶线性 ADRC 控制

算法模块只依赖 C 标准头文件，不再绑定特定 STM32 HAL，可直接加入
STM32CubeIDE、Keil、IAR 或普通 CMake 工程。

## 目录

```text
complementary_filter/  互补滤波
control/               回差、状态反馈与 LADRC 控制
crc/                   CRC-8 与 CRC-16
filters/               EMA、滑动平均、三点中值
kalman/                一维卡尔曼滤波
pid/                   PID 控制器
ring_buffer/           字节环形缓冲区
slew_rate_limiter/     斜率限制器
docs/                  项目文章与设计说明
tests/                 主机侧单元测试
```

## 快速使用

### 环形缓冲区

```c
#include "ring_buffer.h"

RingBuffer rx;
uint8_t byte;

RingBuffer_Init(&rx);
RingBuffer_Put(&rx, 0x55U);

if (RingBuffer_Get(&rx, &byte)) {
    /* process byte */
}
```

容量默认为 256 字节。可在编译选项或包含头文件前覆盖：

```c
#define RING_BUFFER_SIZE 128U
#include "ring_buffer.h"
```

批量读写会在缓冲区满或空时停止，并返回实际处理的字节数。

> 环形缓冲区本身不提供并发同步。如果中断和主循环会同时修改同一个
> 缓冲区，请在工程中使用临界区，或保证严格的单生产者/单消费者访问策略。

### PID

```c
#include "pid.h"

PID_Controller pid;

PID_Init(&pid, PID_POSITION,
         1.2f, 0.01f, 0.05f,
         100.0f, 50.0f);

float output = PID_Calc(&pid, target, feedback);
```

`PID_Calc()` 根据初始化时选择的模式计算，并始终返回限幅后的最终输出。
如需增量式 PID 的本周期增量，可直接调用 `PID_Increment_Calc()`；最终输出
保存在 `pid.out`。

位置式公式：

```text
u(k) = Kp*e(k) + Ki*Σe(k) + Kd*(e(k)-e(k-1))
```

增量式公式：

```text
Δu(k) = Kp*(e(k)-e(k-1))
      + Ki*e(k)
      + Kd*(e(k)-2e(k-1)+e(k-2))
```

### 卡尔曼滤波

```c
#include "kalman.h"

KalmanFilter filter;

Kalman_Init(&filter,
            0.01f, /* Q: 过程噪声方差 */
            0.10f, /* R: 测量噪声方差 */
            1.0f,  /* A: 状态转移系数 */
            1.0f,  /* H: 观测系数 */
            0.0f,  /* 初始状态 */
            1.0f); /* 初始协方差 */

float filtered = Kalman_Filter(&filter, measurement);
```

实现使用 Joseph 形式更新协方差，以减少浮点舍入导致协方差变负的风险；
当创新协方差退化为零时会跳过测量更新，避免除零。

## 控制算法

### 回差开关控制

适合加热器、水泵等只需要开/关或高/低档的对象。回差带可避免测量值在设定点
附近抖动时频繁切换。

```c
#include "control.h"

HysteresisController heater;
HysteresisController_Init(&heater,
                          35.0f,  /* 设定值 */
                          0.5f,   /* 上下各 0.5 的回差 */
                          0.0f, 100.0f, 0U);
float power = HysteresisController_Update(&heater, temperature);
```

### 二状态反馈

当系统能获得两个状态量并已有反馈增益时，可直接计算：
`u = Kr*r - K1*x1 - K2*x2`。

```c
StateFeedback2Controller feedback;
StateFeedback2_Init(&feedback, k1, k2, kr, max_output);
float output = StateFeedback2_Update(
    &feedback, reference, state1, state2);
```

### 一阶线性 ADRC

LADRC 使用扩张状态观测器估计总扰动，适合模型不精确但近似一阶的速度、流量、
温度等对象。`plant_gain` 的符号必须与控制方向一致，更新周期必须固定。

```c
FirstOrderLadrc ladrc;
FirstOrderLadrc_Init(&ladrc,
                     20.0f, /* 观测器带宽 */
                     5.0f,  /* 控制器带宽 */
                     b0, max_output, initial_measurement);
float output = FirstOrderLadrc_Update(
    &ladrc, reference, measurement, dt_seconds);
```

## 其他轻量算法

### 数字滤波

```c
#include "filters.h"

EmaFilter ema;
MovingAverageFilter average;

EmaFilter_Init(&ema, 0.1f, first_sample);
MovingAverage_Init(&average, 8U);

float smooth_a = EmaFilter_Update(&ema, sample);
float smooth_b = MovingAverage_Update(&average, sample);
float no_spike = Median3_Filter(previous, current, next);
```

### 互补滤波

```c
#include "complementary_filter.h"

ComplementaryFilter attitude;
ComplementaryFilter_Init(&attitude, 0.98f, initial_angle);
float angle = ComplementaryFilter_Update(
    &attitude, gyro_rate, reference_angle, dt_seconds);
```

### 斜率限制器

```c
#include "slew_rate_limiter.h"

SlewRateLimiter actuator;
SlewRateLimiter_Init(&actuator, 20.0f, 50.0f, 0.0f);
float safe_output = SlewRateLimiter_Update(
    &actuator, requested_output, dt_seconds);
```

### CRC

```c
#include "crc.h"

uint8_t crc8 = CRC8_Calculate(frame, frame_length);
uint16_t crc16 = CRC16_Modbus(frame, frame_length);
```

更完整的选型、参数和组合示例见
[《在 STM32 上搭一条轻量、可靠的传感器控制链》](docs/lightweight-stm32-algorithms.md)。
## 集成到 STM32 工程

将需要模块的 `.c` 文件加入工程，并把对应目录加入头文件搜索路径即可。
源码不调用动态内存、操作系统或 HAL API。

环形缓冲区若在中断与线程/主循环间共享，应由调用方根据平台实现临界区；
不要仅依赖 `volatile` 来保证复合读写的原子性。

## 构建与测试

使用主机 C 编译器：

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

也可以使用 ARM GCC 做只编译检查：

```sh
arm-none-eabi-gcc -std=c99 -Wall -Wextra -Wpedantic -Wconversion \
  -Icomplementary_filter -Icrc -Ifilters -Ikalman -Ipid \
  -Iring_buffer -Islew_rate_limiter -Icontrol \
  -c complementary_filter/complementary_filter.c control/control.c crc/crc.c \
  filters/filters.c kalman/kalman.c pid/pid.c \
  ring_buffer/ring_buffer.c slew_rate_limiter/slew_rate_limiter.c
```

## 参数建议

- PID：先调 `Kp`，再用较小的 `Ki` 消除稳态误差，最后增加 `Kd` 抑制超调。
- 卡尔曼：增大 Q 会更快跟随新状态；增大 R 会更信任模型预测。
- LADRC：先确认 `plant_gain` 方向，再从较低控制带宽开始，观测器带宽通常略高。
- 所有算法都应使用固定采样周期；当前 PID 系数默认已包含采样周期影响。