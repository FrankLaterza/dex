/*
 * this is an implimentation of a 1d kalman filter. see https://github.com/CarbonAeronautics/Manual-Quadcopter-Drone
 * for a guide on how this filter works.
 * also see https://drive.google.com/drive/folders/1ojPEXYt8hd2MT1O8jXZDiqqSfAkgUkVf
 * for a guide on how to impliment the full euler matrix
 * credits to CarbonAeronautics
 */

#include "kalman.h"
#include "mpu6050.h"
#include "pico/stdlib.h"
#include "utils.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define ACC_X_CAL -0.0625
#define ACC_Y_CAL 0.015
#define ACC_Z_CAL -0.1

// init vars
static float roll_rate, pitch_rate, yaw_rate;
static float roll_rate_cal, pitch_rate_cal, yaw_rate_cal;
static int rate_cal_num;
static float acc_x, acc_y, acc_z;
static float roll_angle, pitch_angle;
static uint32_t loop_timer;
static float kalman_roll_angle, kalman_uncertainty_roll_angle;
static float kalman_pitch_angle, kalman_uncertainty_pitch_angle;
static float kalman_1d_out[] = {0, 0};
static int64_t delta_time;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement, int64_t delta) {
    float delta_seconds = (float)delta / 1000000;
    KalmanState = KalmanState + delta_seconds * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + delta_seconds * delta_seconds * 4 * 4;
    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    kalman_1d_out[0] = KalmanState;
    kalman_1d_out[1] = KalmanUncertainty;
}

void kalman_init() {

    // set unsertanty
    // TODO move to init file and change init angle
    kalman_roll_angle = 0;
    kalman_uncertainty_roll_angle = 2 * 2;
    kalman_pitch_angle = 0;
    kalman_uncertainty_pitch_angle = 2 * 2;

    // start calibration
    for (rate_cal_num = 0; rate_cal_num < 2000; rate_cal_num++) {
        mpu6050_read_raw();
        mpu6050_gyro_scale(&roll_rate, &pitch_rate, &yaw_rate);
        roll_rate_cal += roll_rate;
        pitch_rate_cal += pitch_rate;
        yaw_rate_cal += yaw_rate;
        sleep_ms(1);
    }
    roll_rate_cal /= 2000;
    pitch_rate_cal /= 2000;
    yaw_rate_cal /= 2000;
}

void kalman_calc_angle() {
    // roll_rate = (float)gyro[0] / 65.5;
    // pitch_rate = (float)gyro[1] / 65.5;
    // yaw_rate = (float)gyro[2] / 65.5;
    mpu6050_gyro_scale(&roll_rate, &pitch_rate, &yaw_rate);
    // acc_x = ((float)acceleration[0] / 4096) + ACC_X_CAL;
    // acc_y = ((float)acceleration[1] / 4096) + ACC_Y_CAL;
    // acc_z = ((float)acceleration[2] / 4096) + ACC_Z_CAL;
    mpu6050_accel_scale(&acc_x, &acc_y, &acc_z);
    mpu6050_get_delta(&delta_time);
    roll_angle = atan(acc_y / sqrt(acc_x * acc_x + acc_z * acc_z)) * 1 / (3.142 / 180);
    pitch_angle = -atan(acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * 1 / (3.142 / 180);

    roll_rate -= roll_rate_cal;
    pitch_rate -= pitch_rate_cal;
    yaw_rate -= yaw_rate_cal;
    kalman_1d(kalman_roll_angle, kalman_uncertainty_roll_angle, roll_rate, roll_angle, delta_time);
    kalman_roll_angle = kalman_1d_out[0];
    kalman_uncertainty_roll_angle = kalman_1d_out[1];
    kalman_1d(kalman_pitch_angle, kalman_uncertainty_pitch_angle, pitch_rate, pitch_angle, delta_time);
    kalman_pitch_angle = kalman_1d_out[0];
    kalman_uncertainty_pitch_angle = kalman_1d_out[1];
    g_current_angle_roll  = kalman_roll_angle;
    g_current_angle_pitch = kalman_pitch_angle;
    // sprintf(g_print_buf, "Roll Angle [°] ");
    // vGuardedPrint(g_print_buf);
    // sprintf(g_print_buf, "%f", kalman_roll_angle);
    // vGuardedPrint(g_print_buf);
    // sprintf(g_print_buf, " Pitch Angle [°] ");
    // vGuardedPrint(g_print_buf);
    // sprintf(g_print_buf, "%f", kalman_pitch_angle);
    // vGuardedPrint(g_print_buf);
    // sprintf(g_print_buf, "\n");
    // vGuardedPrint(g_print_buf);
}