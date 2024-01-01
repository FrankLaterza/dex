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

// golf this shit it looks awful
// TODO add micro stepping profile
void balance(struct pid_t *pid_wheels) {
    // get target angle
    float out = pid_calc(pid_wheels, g_current_angle_roll, 0);
    // set the dir
    if (out > 0) {
        set_direction(FORWARD);
    } else {
        set_direction(BACKWARD);
    }
    // step delay
    uint32_t delay = rpm_to_step_delay_us(fabs(out));

    // check min delay
    if (delay > MAX_STEP_TASK_DELAY) {
        delay = MAX_STEP_TASK_DELAY;
    }
    
    // sprintf(g_print_buf, "angle %f | pid %f | us period %u\n", g_current_angle_roll, out, delay);
    // vGuardedPrint(g_print_buf);

    g_step_delay_period_us_right = delay;
    g_step_delay_period_us_left = delay;
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