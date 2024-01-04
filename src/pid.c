#include "pid.h"
#include "kalman.h"
#include "math.h"
#include "pico/stdlib.h"
#include "stepper.h"
#include "utils.h"
#include <stdbool.h>
#include <stdio.h>

void pid_init(struct pid_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    pid->error = 0;
    pid->error_last = 0;
}

void balance(struct pid_t *pid_wheels) {
    // get target angle
    float rpm = pid_calc(pid_wheels, g_current_angle_roll, 0);
    // drive left and right the same for now
    drive_motors_rpm(rpm, rpm);
}

float pid_calc(struct pid_t *pid, float current, float target) {
    // record the last p
    pid->error_last = pid->error;
    // update the error
    pid->error = target - current;
    // direct feedback
    pid->p = pid->kp * pid->error;
    // integrate
    pid->i += (pid->error);
    // derive
    pid->d = pid->kd * (pid->error - pid->error_last);
    // the magic
    return pid->p + (pid->ki * pid->i) + pid->d;
}