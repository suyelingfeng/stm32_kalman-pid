# STM32 轻量级算法库

面向 STM32 和其他裸机/RTOS C 项目的三个独立模块：

- 固定容量字节环形缓冲区
- 位置式与增量式 PID 控制器
- 一维标量卡尔曼滤波器

算法模块只依赖 C 标准头文件，不再绑定特定 STM32 HAL，可直接加入
STM32CubeIDE、Keil、IAR 或普通 CMake 工程。

## 目录

```text
kalman/       一维卡尔曼滤波
pid/          PID 控制器
ring_buffer/  字节环形缓冲区
tests/        主机侧单元测试
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
  -Ikalman -Ipid -Iring_buffer \
  -c kalman/kalman.c pid/pid.c ring_buffer/ring_buffer.c
```

## 参数建议

- PID：先调 `Kp`，再用较小的 `Ki` 消除稳态误差，最后增加 `Kd` 抑制超调。
- 卡尔曼：增大 `Q` 会更快跟随新状态；增大 `R` 会更信任模型预测。
- 所有算法都应使用固定采样周期；当前 PID 系数默认已包含采样周期影响。