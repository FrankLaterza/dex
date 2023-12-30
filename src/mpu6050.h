#ifndef MPU6050_I2C_H
#define MPU6050_I2C_H

void mpu6050_read_raw();
void mpu6050_poll();
void mpu6050_gyro_scale(float *roll_rate, float *pitch_rate, float *yaw_rate);
void mpu6050_accel_scale(float *acc_x, float *acc_y, float *acc_z);
void mpu6050_get_delta(int64_t *delta_time);

#endif