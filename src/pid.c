#include "pid.h"
#include "kalman.h"
#include "math.h"
#include "pico/stdlib.h"
#include "stepper.h"
#include "utils.h"
#include <stdbool.h>
#include <stdio.h>

#define MAX_TARGET_ANGLE_ABSOLUTE 40
#define MAX_OFFSET_ANGLE_ABSOLUTE 20
#define POSITION_LOCK_THRESHOLD 5

static struct pid_t pid_wheels;
static struct pid_t pid_rpm;
static struct pid_t pid_position;
/*
 * the average of the left and right rpm is taken
 * because we want to know the velocity vector
 * related to the center fo the robot. regarless
 * during turning the offset of rpm will be applied
 * to both left and right the same.
 */
static float center_balance = -4.5;
static float angle_offset = 0;
static float rpm_balance = 0;
static float average_rpm = 0;
static float rpm_left = 0;
static float rpm_right = 0;
static uint32_t step_count_avg = 0;
static bool position_lock = true;

void pid_init(struct pid_t *pid, float kp, float ki, float kd, float max_windup) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    pid->error = 0;
    pid->error_last = 0;
    pid->max_windup = max_windup;
}

float pid_calc(struct pid_t *pid, float current, float target) {
    // record the last p
    pid->error_last = pid->error;
    // update the error
    pid->error = target - current;
    // direct feedback
    pid->p = pid->kp * pid->error;
    // integrate with windup
    pid->i = convert_from_range_float(pid->i + pid->error, -pid->max_windup, pid->max_windup);
    // derive
    pid->d = pid->kd * (pid->error - pid->error_last);
    // the magic
    return pid->p + (pid->ki * pid->i) + pid->d;
}

void pid_init_balance() {
    // TODO tune
    pid_init(&pid_wheels, 0.3, 0.001, 6, 15);
    pid_init(&pid_rpm, 0.14, 0.02, 0.01, 100);
    pid_init(&pid_position, 0.1, 0.0, 0.0, 100);
}

// main balancing script
void balance() {
    if (fabs(g_current_angle_roll) >= 60) {
        disable_stepper();
        return;
    }
    // accelerate to target rpm
    if (g_target_rpm > 0) {
        // get target angle
        angle_offset = pid_calc(&pid_rpm, average_rpm, g_target_rpm);
        angle_offset = convert_from_absolute_range_float(angle_offset, MAX_TARGET_ANGLE_ABSOLUTE);
        position_lock = false; // we are moving
    }
    // lock onto position
    else if (position_lock) {
        step_count_avg = (step_count_left + step_count_right) / 2;
        angle_offset = pid_calc(&pid_position, step_count_avg, 0);
        angle_offset = convert_from_absolute_range_float(angle_offset , MAX_TARGET_ANGLE_ABSOLUTE);
    }
    // there is no target
    else if (average_rpm <= POSITION_LOCK_THRESHOLD) {
        // toggle position lock after its been set false by target rpm
        position_lock = true;
        step_count_left = 0;
        step_count_right = 0;
    }
    else {
        // enforce deceleration
        angle_offset = 0;
    }

    angle_offset = convert_from_absolute_range_float(angle_offset, MAX_OFFSET_ANGLE_ABSOLUTE);
    rpm_balance = pid_calc(&pid_wheels, g_current_angle_roll, center_balance - angle_offset);
    average_rpm += rpm_balance / 2;
    average_rpm = convert_from_absolute_range_float(average_rpm, MAX_RPM_ABSOLUTE);

    // drive left and right the same for now
    drive_motors_rpm(average_rpm + g_steer, average_rpm - g_steer);
}

void print_balance_stats() {
    sprintf(g_print_buf, "target rpm: %f, average rpm: %f, angle offset: %f, current angle: %f, balance rpm: %f",
            g_target_rpm, average_rpm, angle_offset, g_current_angle_roll, rpm_balance);
    vGuardedPrint(g_print_buf);
}

void tune_center_up() {
    center_balance += 0.05;
}

void tune_center_down() {
    center_balance -= 0.05;
}

void reset_balance() {
    average_rpm = 0;
    rpm_left = 0;
    rpm_right = 0;
}

void do_nothing() {
    return;
};

void print_star() {
    sprintf(g_print_buf, "*");
    vGuardedPrint(g_print_buf);
}

void print_tweaker(uint8_t tune_select) {
    // print by a page
    if (tune_select <= 3) {
        vGuardedPrint("| pid wheels: ");
        tune_select == 0 ? print_star() : do_nothing();
        sprintf(g_print_buf, "w-kp: %f, ", pid_wheels.kp);
        vGuardedPrint(g_print_buf);
        tune_select == 1 ? print_star() : do_nothing();
        sprintf(g_print_buf, "w-ki: %f, ", pid_wheels.ki);
        vGuardedPrint(g_print_buf);
        tune_select == 2 ? print_star() : do_nothing();
        sprintf(g_print_buf, "w-kd: %f, ", pid_wheels.kd);
        vGuardedPrint(g_print_buf);
        tune_select == 3 ? print_star() : do_nothing();
        sprintf(g_print_buf, "w-mw: %f | ", pid_wheels.max_windup);
        vGuardedPrint(g_print_buf);
    } else if (tune_select <= 7) {
        vGuardedPrint("| rpm wheels: ");
        tune_select == 4 ? print_star() : do_nothing();
        sprintf(g_print_buf, "r-kp: %f, ", pid_rpm.kp);
        vGuardedPrint(g_print_buf);
        tune_select == 5 ? print_star() : do_nothing();
        sprintf(g_print_buf, "r-ki: %f, ", pid_rpm.ki);
        vGuardedPrint(g_print_buf);
        tune_select == 6 ? print_star() : do_nothing();
        sprintf(g_print_buf, "r-kd: %f, ", pid_rpm.kd);
        vGuardedPrint(g_print_buf);
        tune_select == 7 ? print_star() : do_nothing();
        sprintf(g_print_buf, "r-mw: %f | ", pid_rpm.max_windup);
        vGuardedPrint(g_print_buf);
    } else {
        vGuardedPrint("| rpm position: ");
        tune_select == 8 ? print_star() : do_nothing();
        sprintf(g_print_buf, "p-kp: %f, ", pid_position.kp);
        vGuardedPrint(g_print_buf);
        tune_select == 9 ? print_star() : do_nothing();
        sprintf(g_print_buf, "p-ki: %f, ", pid_position.ki);
        vGuardedPrint(g_print_buf);
        tune_select == 10 ? print_star() : do_nothing();
        sprintf(g_print_buf, "p-kd: %f, ", pid_position.kd);
        vGuardedPrint(g_print_buf);
        tune_select == 11 ? print_star() : do_nothing();
        sprintf(g_print_buf, "p-mw: %f | ", pid_position.max_windup);
        vGuardedPrint(g_print_buf);
    }
}

/*
 * handles pid turn
 * turn select is as follows
 */
void tweaker(uint8_t tune_select, bool inc_dir) {
    switch (tune_select) {
    case 0:
        pid_wheels.kp += inc_dir * 0.01;
        break;
    case 1:
        pid_wheels.ki += inc_dir * 0.001;
        break;
    case 2:
        pid_wheels.kd += inc_dir * 0.5;
        break;
    case 3:
        pid_wheels.max_windup += inc_dir * 5;
        break;
    case 4:
        pid_rpm.kp += inc_dir * 0.01;
        break;
    case 5:
        pid_rpm.ki += inc_dir * 0.001;
        break;
    case 6:
        pid_rpm.kd += inc_dir * 0.01;
        break;
    case 7:
        pid_rpm.max_windup += inc_dir * 10;
        break;
    case 8:
        pid_position.kp += inc_dir * 0.1;
        break;
    case 9:
        pid_position.ki += inc_dir * 0.001;
        break;
    case 10:
        pid_position.kd += inc_dir * 0.1;
        break;
    case 11:
        pid_position.max_windup += inc_dir * 10;
        break;
    }
}
