#include "control.h"

#include <float.h>
#include <stddef.h>

static float Magnitude(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void HysteresisController_Init(HysteresisController *controller,
                               float setpoint,
                               float half_band,
                               float output_low,
                               float output_high,
                               uint8_t high_active)
{
    if (controller == NULL) {
        return;
    }

    controller->setpoint = setpoint;
    controller->half_band = Magnitude(half_band);
    controller->output_low = output_low;
    controller->output_high = output_high;
    controller->high_active = (uint8_t)(high_active != 0U);
}

float HysteresisController_Update(HysteresisController *controller,
                                  float measurement)
{
    if (controller == NULL) {
        return 0.0f;
    }

    if (controller->high_active != 0U) {
        if (measurement >= controller->setpoint + controller->half_band) {
            controller->high_active = 0U;
        }
    } else if (measurement <= controller->setpoint -
                              controller->half_band) {
        controller->high_active = 1U;
    }

    return (controller->high_active != 0U) ?
           controller->output_high : controller->output_low;
}

void HysteresisController_Setpoint(HysteresisController *controller,
                                   float setpoint)
{
    if (controller == NULL) {
        return;
    }
    controller->setpoint = setpoint;
}

void StateFeedback2_Init(StateFeedback2Controller *controller,
                         float k1,
                         float k2,
                         float reference_gain,
                         float max_output)
{
    if (controller == NULL) {
        return;
    }

    controller->k1 = k1;
    controller->k2 = k2;
    controller->reference_gain = reference_gain;
    controller->max_output = Magnitude(max_output);
    controller->output = 0.0f;
}

float StateFeedback2_Update(StateFeedback2Controller *controller,
                            float reference,
                            float state1,
                            float state2)
{
    if (controller == NULL) {
        return 0.0f;
    }

    controller->output =
        controller->reference_gain * reference -
        controller->k1 * state1 -
        controller->k2 * state2;
    controller->output = Clamp(controller->output, controller->max_output);
    return controller->output;
}

void StateFeedback2_Reset(StateFeedback2Controller *controller)
{
    if (controller == NULL) {
        return;
    }
    controller->output = 0.0f;
}

void FirstOrderLadrc_Init(FirstOrderLadrc *controller,
                          float observer_bandwidth,
                          float controller_bandwidth,
                          float plant_gain,
                          float max_output,
                          float initial_measurement)
{
    if (controller == NULL) {
        return;
    }

    controller->observer_bandwidth = Magnitude(observer_bandwidth);
    controller->controller_bandwidth = Magnitude(controller_bandwidth);
    if (Magnitude(plant_gain) <= FLT_EPSILON) {
        plant_gain = 1.0f;
    }
    controller->plant_gain = plant_gain;
    controller->max_output = Magnitude(max_output);
    FirstOrderLadrc_Reset(controller, initial_measurement);
}

float FirstOrderLadrc_Update(FirstOrderLadrc *controller,
                            float reference,
                            float measurement,
                            float dt_seconds)
{
    float observer_error;
    float beta1;
    float beta2;
    float z1_rate;
    float z2_rate;
    float virtual_output;

    if (controller == NULL) {
        return 0.0f;
    }
    if (dt_seconds <= 0.0f) {
        return controller->output;
    }

    observer_error = controller->z1 - measurement;
    beta1 = 2.0f * controller->observer_bandwidth;
    beta2 = controller->observer_bandwidth *
            controller->observer_bandwidth;

    z1_rate = controller->z2 - beta1 * observer_error +
              controller->plant_gain * controller->output;
    z2_rate = -beta2 * observer_error;
    controller->z1 += z1_rate * dt_seconds;
    controller->z2 += z2_rate * dt_seconds;

    virtual_output =
        controller->controller_bandwidth * (reference - controller->z1) -
        controller->z2;
    controller->output =
        Clamp(virtual_output / controller->plant_gain,
              controller->max_output);
    return controller->output;
}

void FirstOrderLadrc_Reset(FirstOrderLadrc *controller,
                           float measurement)
{
    if (controller == NULL) {
        return;
    }

    controller->z1 = measurement;
    controller->z2 = 0.0f;
    controller->output = 0.0f;
}
