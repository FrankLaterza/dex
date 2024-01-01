#ifndef PID_H
#define PID_H

#include <stdio.h>
#include "pico/stdlib.h"

struct pid_t {
    float kp;
    float ki;
    float kd;
    float p;
    float i;
    float d;
    float error;
    float error_last;
};

void pid_init(struct pid_t *pid, float kp, float ki, float kd);
void config_pid(struct pid_t *pid, float kp, float ki, float kd);
float pid_calc(struct pid_t *pid, float current, float target);

#endif