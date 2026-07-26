#include "complementary_filter.h"
#include "control.h"
#include "crc.h"
#include "filters.h"
#include "kalman.h"
#include "pid.h"
#include "ring_buffer.h"
#include "slew_rate_limiter.h"

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

static void TestFilters(void)
{
    EmaFilter ema;
    MovingAverageFilter average;

    EmaFilter_Init(&ema, 0.25f, 0.0f);
    assert(NearlyEqual(EmaFilter_Update(&ema, 8.0f), 2.0f, 0.0001f));
    assert(NearlyEqual(EmaFilter_Update(&ema, 8.0f), 3.5f, 0.0001f));
    EmaFilter_Init(&ema, 2.0f, 0.0f);
    assert(NearlyEqual(EmaFilter_Update(&ema, 5.0f), 5.0f, 0.0001f));

    MovingAverage_Init(&average, 3U);
    assert(NearlyEqual(MovingAverage_Update(&average, 1.0f), 1.0f, 0.0001f));
    assert(NearlyEqual(MovingAverage_Update(&average, 2.0f), 1.5f, 0.0001f));
    assert(NearlyEqual(MovingAverage_Update(&average, 3.0f), 2.0f, 0.0001f));
    assert(NearlyEqual(MovingAverage_Update(&average, 7.0f), 4.0f, 0.0001f));

    assert(NearlyEqual(Median3_Filter(100.0f, 2.0f, 3.0f), 3.0f, 0.0001f));
    assert(NearlyEqual(Median3_Filter(-1.0f, -3.0f, -2.0f), -2.0f, 0.0001f));
}

static void TestComplementaryFilter(void)
{
    ComplementaryFilter filter;

    ComplementaryFilter_Init(&filter, 0.9f, 0.0f);
    assert(NearlyEqual(ComplementaryFilter_Update(
                           &filter, 10.0f, 0.0f, 0.1f),
                       0.9f, 0.0001f));
    assert(NearlyEqual(ComplementaryFilter_Update(
                           &filter, 0.0f, 10.0f, 0.1f),
                       1.81f, 0.0001f));
    ComplementaryFilter_Reset(&filter, 5.0f);
    assert(NearlyEqual(filter.angle, 5.0f, 0.0001f));
}

static void TestSlewRateLimiter(void)
{
    SlewRateLimiter limiter;

    SlewRateLimiter_Init(&limiter, 2.0f, 4.0f, 0.0f);
    assert(NearlyEqual(SlewRateLimiter_Update(&limiter, 10.0f, 0.5f),
                       1.0f, 0.0001f));
    assert(NearlyEqual(SlewRateLimiter_Update(&limiter, 10.0f, 0.5f),
                       2.0f, 0.0001f));
    assert(NearlyEqual(SlewRateLimiter_Update(&limiter, -10.0f, 0.5f),
                       0.0f, 0.0001f));
    assert(NearlyEqual(SlewRateLimiter_Update(&limiter, 5.0f, 0.0f),
                       0.0f, 0.0001f));
}

static void TestCrc(void)
{
    static const uint8_t check_data[] = {
        (uint8_t)'1', (uint8_t)'2', (uint8_t)'3',
        (uint8_t)'4', (uint8_t)'5', (uint8_t)'6',
        (uint8_t)'7', (uint8_t)'8', (uint8_t)'9'
    };

    assert(CRC8_Calculate(check_data, sizeof(check_data)) == 0xF4U);
    assert(CRC16_Modbus(check_data, sizeof(check_data)) == 0x4B37U);
    assert(CRC8_Calculate(NULL, 0U) == 0x00U);
    assert(CRC16_Modbus(NULL, 0U) == 0xFFFFU);
}

static void TestControlAlgorithms(void)
{
    HysteresisController hysteresis;
    StateFeedback2Controller state_feedback;
    FirstOrderLadrc ladrc;
    float plant_output = 0.0f;
    float control_output = 0.0f;
    unsigned int index;

    HysteresisController_Init(&hysteresis,
                              35.0f, 0.5f, 0.0f, 100.0f, 1U);
    assert(NearlyEqual(HysteresisController_Update(&hysteresis, 34.0f),
                       100.0f, 0.0001f));
    assert(NearlyEqual(HysteresisController_Update(&hysteresis, 35.2f),
                       100.0f, 0.0001f));
    assert(NearlyEqual(HysteresisController_Update(&hysteresis, 35.5f),
                       0.0f, 0.0001f));
    assert(NearlyEqual(HysteresisController_Update(&hysteresis, 34.8f),
                       0.0f, 0.0001f));
    assert(NearlyEqual(HysteresisController_Update(&hysteresis, 34.5f),
                       100.0f, 0.0001f));

    StateFeedback2_Init(&state_feedback, 2.0f, 0.5f, 3.0f, 10.0f);
    assert(NearlyEqual(StateFeedback2_Update(
                           &state_feedback, 4.0f, 1.0f, 2.0f),
                       9.0f, 0.0001f));
    assert(NearlyEqual(StateFeedback2_Update(
                           &state_feedback, 10.0f, 0.0f, 0.0f),
                       10.0f, 0.0001f));

    FirstOrderLadrc_Init(&ladrc, 20.0f, 5.0f, 1.0f, 10.0f, 0.0f);
    for (index = 0U; index < 500U; ++index) {
        control_output = FirstOrderLadrc_Update(
            &ladrc, 1.0f, plant_output, 0.01f);
        plant_output += control_output * 0.01f;
    }
    assert(NearlyEqual(plant_output, 1.0f, 0.02f));
    assert(control_output <= 10.0f);
    assert(control_output >= -10.0f);
}

int main(void)
{
    TestRingBuffer();
    TestPid();
    TestKalman();
    TestFilters();
    TestComplementaryFilter();
    TestSlewRateLimiter();
    TestCrc();
    TestControlAlgorithms();
    puts("All algorithm tests passed.");
    return 0;
}