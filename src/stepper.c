#include "stepper.h"
#include "pico/stdlib.h"
#include "pinout.h"
#include "utils.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// static vars only live here
static uint8_t step_size_l = FULL;
static uint8_t step_size_r = FULL;

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
        return MAX_STEP_RATE_FULL;
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
    return (uint32_t)delay_period_us;
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
    rpm_l > 0 ? set_direction(LEFT, FORWARD) : set_direction(LEFT, BACKWARD);
    rpm_r > 0 ? set_direction(RIGHT, BACKWARD) : set_direction(RIGHT, FORWARD);

    // store the step size (only changes in stepper task)
    step_size_l = rpm_step_profile(fabs(rpm_l));
    step_size_r = rpm_step_profile(fabs(rpm_r));

    // step_size_l = FOURTH;
    // step_size_r = EIGHTH;

    // get the step delay
    uint32_t step_delay_l = rpm_to_step_delay_us_fast(LEFT, fabs(rpm_l));
    uint32_t step_delay_r = rpm_to_step_delay_us_fast(RIGHT, fabs(rpm_r));

    sprintf(g_print_buf, "l rpm: %f, l step size: %u, l step delay: %u\n", rpm_l, step_size_l, step_delay_l);
    vGuardedPrint(g_print_buf);

    // drive motor
    g_step_delay_period_us_left = step_delay_l;
    g_step_delay_period_us_right = step_delay_r;
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