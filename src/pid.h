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
float pid_calc(struct pid_t *pid, float current, float target);
void pid_init_balance();
void balance();
void print_balance_stats();
void tune_center_up();
void tune_center_down();
void reset_balance();
void print_tweaker(uint8_t tune_select);
void tweaker(uint8_t tune_select, bool inc_dir);

#endif