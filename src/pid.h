#include <stdio.h>
#include "pico/stdlib.h"

#ifndef PID_H
#define PID_H

struct pid_t {
    float kp;
    float ki;
    float kd;
    float p;
    float i;
    float d;
    float error;
    float error_last;
    float max_windup;
};

void pid_init(struct pid_t *pid, float kp, float ki, float kd, float max_windup);
void config_pid(struct pid_t *pid, float kp, float ki, float kd);
float pid_calc(struct pid_t *pid, float current, float target);

#endif