#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float setpoint;
    float half_band;
    float output_low;
    float output_high;
    uint8_t high_active;
} HysteresisController;

typedef struct {
    float k1;
    float k2;
    float reference_gain;
    float max_output;
    float output;
} StateFeedback2Controller;

typedef struct {
    float observer_bandwidth;
    float controller_bandwidth;
    float plant_gain;
    float max_output;
    float z1;
    float z2;
    float output;
} FirstOrderLadrc;

/*
 * Direct-acting hysteresis controller: low measurements request high output.
 * half_band is the distance from setpoint to each switching threshold.
 */
void HysteresisController_Init(HysteresisController *controller,
                               float setpoint,
                               float half_band,
                               float output_low,
                               float output_high,
                               uint8_t high_active);
float HysteresisController_Update(HysteresisController *controller,
                                  float measurement);
void HysteresisController_Setpoint(HysteresisController *controller,
                                   float setpoint);

/* u = reference_gain * reference - k1 * state1 - k2 * state2 */
void StateFeedback2_Init(StateFeedback2Controller *controller,
                         float k1,
                         float k2,
                         float reference_gain,
                         float max_output);
float StateFeedback2_Update(StateFeedback2Controller *controller,
                            float reference,
                            float state1,
                            float state2);
void StateFeedback2_Reset(StateFeedback2Controller *controller);

/*
 * First-order linear ADRC with a second-order extended-state observer.
 * dt_seconds must be the fixed controller period in seconds.
 */
void FirstOrderLadrc_Init(FirstOrderLadrc *controller,
                          float observer_bandwidth,
                          float controller_bandwidth,
                          float plant_gain,
                          float max_output,
                          float initial_measurement);
float FirstOrderLadrc_Update(FirstOrderLadrc *controller,
                            float reference,
                            float measurement,
                            float dt_seconds);
void FirstOrderLadrc_Reset(FirstOrderLadrc *controller,
                           float measurement);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
