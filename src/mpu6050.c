#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "kalman.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

// TODO move these to a pinouts header file
#define I2C_SDA 0
#define I2C_SCL 1
#define I2C i2c0
#define DEVICE_ADDRESS 0x68
#define ACC_X_CAL -0.0625
#define ACC_Y_CAL 0.015
#define ACC_Z_CAL -0.1

// global static variables (only live here)
static int16_t acceleration[3], gyro[3];
static absolute_time_t start_time, end_time;
static int64_t elapsed_time;

void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(I2C, DEVICE_ADDRESS, buf, 2, false);
}

void mpu6050_config() {
    // config mpu6050
    uint8_t config_filt[] = {0x1A, 0x05};
    i2c_write_blocking(I2C, DEVICE_ADDRESS, config_filt, 2, true);
    uint8_t config_accel[] = {0x1B, 0x8};
    i2c_write_blocking(I2C, DEVICE_ADDRESS, config_accel, 2, true);
    uint8_t config_gyro[] = {0x1C, 0x10};
    i2c_write_blocking(I2C, DEVICE_ADDRESS, config_gyro, 2, true);
}

void mpu6050_read_raw() {

    uint8_t buffer[6];
    // start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C, DEVICE_ADDRESS, &val, 1, true);
    i2c_read_blocking(I2C, DEVICE_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        acceleration[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // now gyro data from reg 0x43 for 6 bytes
    // the register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(I2C, DEVICE_ADDRESS, &val, 1, true);
    i2c_read_blocking(I2C, DEVICE_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

void mpu6050_gyro_scale(float *roll_rate, float *pitch_rate, float *yaw_rate) {
    *roll_rate = (float)gyro[0] / 65.5;
    *pitch_rate = (float)gyro[1] / 65.5;
    *yaw_rate = (float)gyro[2] / 65.5;
}

void mpu6050_accel_scale(float *acc_x, float *acc_y, float *acc_z) {
    *acc_x = ((float)acceleration[0] / 4096) + ACC_X_CAL;
    *acc_y = ((float)acceleration[1] / 4096) + ACC_Y_CAL;
    *acc_z = ((float)acceleration[2] / 4096) + ACC_Z_CAL;
}

void mpu6050_get_delta(int64_t *delta_time){
    *delta_time = elapsed_time;
}


void mpu6050_poll() {
    end_time = get_absolute_time();
    elapsed_time = absolute_time_diff_us(start_time, end_time);
    start_time = get_absolute_time();
    mpu6050_read_raw(acceleration, gyro);
    
    // printf("Time elapsed: %lld us\n", elapsed_time);
    // printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
    // printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
    kalman_calc_angle();
}
