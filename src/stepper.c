#include "stepper.h"
#include "pico/stdlib.h"
#include "pid.h"
#include "pinout.h"
#include "utils.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// static vars only live here
static uint8_t step_size_l = FULL;
static uint8_t step_size_r = FULL;

/*
 * the average of the left and right rpm is taken
 * because we want to know the velocity vector
 * related to the center fo the robot. regarless
 * during turning the offset of rpm will be applied
 * to both left and right the same.
 */
static float current_rpm_average = 0;
static float center_balance = -4.5;

void enable_stepper() {
    // enable the motor
    gpio_put(SLEEP, HIGH);
    gpio_put(RESET, HIGH);
}

void disable_stepper() {
    // disable the motor
    gpio_put(SLEEP, LOW);
    gpio_put(RESET, LOW);
}

void set_direction(bool stepper, bool direction) {
    // sets the direction of a specific stepper
    direction ? gpio_put(stepper ? DIR_L : DIR_R, LOW) : gpio_put(stepper ? DIR_L : DIR_R, HIGH);
}

uint8_t rpm_step_profile(float rpm) {
    uint8_t step_size;

    // if (rpm > MIN_RPM_FULL) {
    //     step_size = FULL;
    // }
    if (rpm > MIN_RPM_HALF) {
        step_size = HALF;
    } else if (rpm > MIN_RPM_FOURTH) {
        step_size = FOURTH;
    } else {
        step_size = EIGHTH;
    }
    // } else {
    //     step_size = SIXTEENTH;
    // }
    return step_size;
}

/*
 * converts rpm into steps per micro second
 * note this will be the perfect instantaneous delay
 */
uint32_t rpm_to_step_delay_us(bool stepper, float rpm) {
    uint8_t step_size = stepper ? step_size_l : step_size_r;

    if (rpm == 0) {
        return MAX_STEP_TASK_DELAY;
    }
    // revolutions per second
    float rps = rpm / 60;
    // degrees per second
    float dps = rps * 360;
    // steps per second
    float sps = dps / (STEP_ANGLE / step_size);
    // step delay period (div by 2 because delay twice)
    float delay_period_s = (1 / sps) / 2;
    // get the us delay
    int32_t delay_period_us = delay_period_s * 1000000;
    // return the goods
    return min((uint32_t)delay_period_us, MAX_STEP_TASK_DELAY);
}

uint32_t rpm_to_step_delay_us_fast(bool stepper, float rpm) {
    uint8_t step_size = stepper ? step_size_l : step_size_r;
    if (rpm == 0) {
        return MAX_STEP_TASK_DELAY;
    }
    float delay_period_us = ((RPM_US_CONVERSION_MAGIC * STEP_ANGLE) / (rpm * step_size));
    int32_t round = delay_period_us;
    return min((uint32_t)round, MAX_STEP_TASK_DELAY);
}

// drives both stepper motor based on an rpm
void drive_motors_rpm(float rpm_l, float rpm_r) {
    // drive the left and right motor in the appropriate direction
    rpm_l > 0 ? set_direction(LEFT, BACKWARD) : set_direction(LEFT, FORWARD);
    rpm_r > 0 ? set_direction(RIGHT, FORWARD) : set_direction(RIGHT, BACKWARD);

    rpm_l = convert_from_range_float(rpm_l, -MAX_RPM_ABSOLUTE, MAX_RPM_ABSOLUTE);
    rpm_r = convert_from_range_float(rpm_r, -MAX_RPM_ABSOLUTE, MAX_RPM_ABSOLUTE);

    // current tangent rpm
    current_rpm_average = (rpm_l + rpm_r) / 2;

    // store the step size (only changes in stepper task)
    step_size_l = rpm_step_profile(fabs(rpm_l));
    step_size_r = rpm_step_profile(fabs(rpm_r));

    // get the step delay
    uint32_t step_delay_l = rpm_to_step_delay_us_fast(LEFT, fabs(rpm_l));
    uint32_t step_delay_r = rpm_to_step_delay_us_fast(RIGHT, fabs(rpm_r));

    // sprintf(g_print_buf, "l rpm: %f, l step size: %u, l step delay: %u\n", rpm_l, step_size_l, step_delay_r);
    // vGuardedPrint(g_print_buf);

    // drive motor
    g_step_delay_period_us_left = step_delay_l;
    g_step_delay_period_us_right = step_delay_r;
}

// main balancing script
void balance(struct pid_t *pid_wheels, struct pid_t *pid_rpm) {
    if (fabs(g_current_angle_roll) >= 50) {
        disable_stepper();
        return;
    }
    // get target angle)
    // float angle_offset = -pid_calc(pid_rpm, current_rpm_average, g_target_rpm);
    // angle_offset = convert_from_range_float(angle_offset, -MAX_TARGET_ANGLE_ABSOLUTE, MAX_TARGET_ANGLE_ABSOLUTE);
    float rpm_balance = pid_calc(pid_wheels, g_current_angle_roll, center_balance + g_target_rpm);
    sprintf(g_print_buf, "current angle %f, steer %f, current rpm %f, target rpm %f\n",
            g_current_angle_roll, g_steer, current_rpm_average, g_target_rpm);
    vGuardedPrint(g_print_buf);
    // drive left and right the same for now
    drive_motors_rpm(rpm_balance + g_steer, rpm_balance - g_steer);
}

void tune_center_up() {
    center_balance += 0.05;
}

void tune_center_down() {
    center_balance -= 0.05;
}

void set_step_size(uint8_t step_config) {
    if (step_config == FULL) {
        gpio_put(MS1, LOW);
        gpio_put(MS2, LOW);
        gpio_put(MS3, LOW);
    } else if (step_config == HALF) {
        gpio_put(MS1, HIGH);
        gpio_put(MS2, LOW);
        gpio_put(MS3, LOW);
    } else if (step_config == FOURTH) {
        gpio_put(MS1, LOW);
        gpio_put(MS2, HIGH);
        gpio_put(MS3, LOW);
    } else if (step_config == EIGHTH) {
        gpio_put(MS1, HIGH);
        gpio_put(MS2, HIGH);
        gpio_put(MS3, LOW);
    } else if (step_config == SIXTEENTH) {
        gpio_put(MS1, HIGH);
        gpio_put(MS2, HIGH);
        gpio_put(MS3, HIGH);
    }
}

void set_stepper_step_size(uint8_t stepper) {
    set_step_size(stepper ? step_size_l : step_size_r);
}