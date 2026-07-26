#include "kalman.h"
#include "pid.h"
#include "ring_buffer.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static int NearlyEqual(float left, float right, float tolerance)
{
    return fabsf(left - right) <= tolerance;
}

static void TestRingBuffer(void)
{
    RingBuffer rb;
    uint8_t value;
    uint8_t input[RING_BUFFER_SIZE];
    uint8_t output[RING_BUFFER_SIZE];
    uint16_t index;

    RingBuffer_Init(&rb);
    assert(RingBuffer_IsEmpty(&rb));
    assert(RingBuffer_GetFreeSpace(&rb) == RING_BUFFER_SIZE);
    for (index = 0U; index < (uint16_t)RING_BUFFER_SIZE; ++index) {
        input[index] = (uint8_t)index;
    }
    assert(RingBuffer_Write(&rb, input, (uint16_t)RING_BUFFER_SIZE) ==
           (uint16_t)RING_BUFFER_SIZE);
    assert(RingBuffer_IsFull(&rb));
    assert(RingBuffer_Put(&rb, 0xFFU) == 0U);
    for (index = 0U; index < 32U; ++index) {
        assert(RingBuffer_Get(&rb, &value) == 1U);
        assert(value == (uint8_t)index);
    }
    assert(RingBuffer_Write(&rb, input, 32U) == 32U);
    assert(RingBuffer_Read(&rb, output, (uint16_t)RING_BUFFER_SIZE) ==
           (uint16_t)RING_BUFFER_SIZE);
    for (index = 0U; index < (uint16_t)(RING_BUFFER_SIZE - 32U); ++index) {
        assert(output[index] == input[index + 32U]);
    }
    for (; index < (uint16_t)RING_BUFFER_SIZE; ++index) {
        assert(output[index] == input[index - (RING_BUFFER_SIZE - 32U)]);
    }
    assert(RingBuffer_IsEmpty(&rb));
}

static void TestPid(void)
{
    PID_Controller position;
    PID_Controller increment;
    float delta;

    PID_Init(&position, PID_POSITION, 2.0f, 1.0f, 0.0f, -10.0f, -3.0f);
    assert(NearlyEqual(PID_Calc(&position, 10.0f, 0.0f), 10.0f, 0.0001f));
    assert(NearlyEqual(position.Iout, 3.0f, 0.0001f));
    PID_Reset(&position);
    assert(NearlyEqual(position.out, 0.0f, 0.0001f));

    PID_Init(&increment, PID_INCREMENT, 1.0f, 0.0f, 0.0f, 5.0f, 0.0f);
    delta = PID_Increment_Calc(&increment, 10.0f, 0.0f);
    assert(NearlyEqual(delta, 10.0f, 0.0001f));
    assert(NearlyEqual(increment.out, 5.0f, 0.0001f));
    PID_Reset(&increment);
    assert(NearlyEqual(PID_Calc(&increment, 10.0f, 0.0f),
                       5.0f, 0.0001f));
}

static void TestKalman(void)
{
    KalmanFilter filter;
    unsigned int index;
    float result = 0.0f;

    Kalman_Init(&filter, 0.01f, 0.1f, 1.0f, 1.0f, 0.0f, 1.0f);
    for (index = 0U; index < 50U; ++index) {
        result = Kalman_Filter(&filter, 10.0f);
    }
    assert(NearlyEqual(result, 10.0f, 0.01f));
    assert(filter.P >= 0.0f);
    Kalman_Init(&filter, -1.0f, -1.0f, 1.0f, 0.0f, 2.0f, -1.0f);
    assert(NearlyEqual(Kalman_Filter(&filter, 99.0f), 2.0f, 0.0001f));
    assert(NearlyEqual(filter.K, 0.0f, 0.0001f));
    assert(filter.P >= 0.0f);
}

int main(void)
{
    TestRingBuffer();
    TestPid();
    TestKalman();
    puts("All algorithm tests passed.");
    return 0;
}