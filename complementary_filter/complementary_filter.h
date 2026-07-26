#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float alpha;
    float angle;
} ComplementaryFilter;

void ComplementaryFilter_Init(ComplementaryFilter *filter,
                              float alpha,
                              float initial_angle);
float ComplementaryFilter_Update(ComplementaryFilter *filter,
                                 float gyro_rate,
                                 float reference_angle,
                                 float dt_seconds);
void ComplementaryFilter_Reset(ComplementaryFilter *filter, float angle);

#ifdef __cplusplus
}
#endif

#endif /* COMPLEMENTARY_FILTER_H */
