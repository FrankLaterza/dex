#include "stepper.h"
#include "pico/stdlib.h"
#include "pinout.h"
#include "utils.h"
#include <stdint.h>
#include <stdio.h>

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

/*
 * converts rpm into steps per micro second
 * note this will be the perfect instantaneous delay
 */
// TODO confirm this calculation
uint32_t rpm_to_step_delay_us(float rpm) {
    if (rpm == 0) { return 0; }
    // TODO float division is slow and ugly
    // revolutions per second
    float rps = rpm / 60;
    // degrees per second
    float dps = rps * 360;
    // steps per second
    float sps = dps / STEP_ANGLE;
    // step delay period (div by 2 because delay twice)
    float delay_period_s = (1 / sps) / 2;
    // get the us delay
    int32_t delay_period_us = delay_period_s * 1000000;
    // return the goods
    return (uint32_t)delay_period_us;
}

void config_step_size(uint8_t step_config) {
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
    } else if (step_config == EIGHT) {
        gpio_put(MS1, HIGH);
        gpio_put(MS2, HIGH);
        gpio_put(MS3, LOW);
    } else if (step_config == SIXTEENTH) {
        gpio_put(MS1, HIGH);
        gpio_put(MS2, HIGH);
        gpio_put(MS3, HIGH);
    }
}

void set_direction(bool direction) {
    // forward
    if (direction) {
        gpio_put(DIR_L, LOW);
        gpio_put(DIR_R, HIGH);
    }
    // backward
    else {
        gpio_put(DIR_L, HIGH);
        gpio_put(DIR_R, LOW);
    }
}