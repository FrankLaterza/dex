
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement, int64_t delta);
void kalman_init();
void kalman_calc_angle();

#endif