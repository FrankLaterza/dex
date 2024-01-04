#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>

#define STEP_ANGLE 1.80
#define FULL 1
#define HALF 2
#define FOURTH 4
#define EIGHTH 8
#define SIXTEENTH 16
#define FORWARD 1
#define BACKWARD 0
#define LEFT 0
#define RIGHT 1
#define MAX_RPM_FULL 208
#define MIN_RPM_FULL 45
#define MAX_STEP_RATE_FULL 3000
#define MIN_STEP_RATE_FULL 721
#define MAX_STEP_TASK_DELAY 20000
#define RPM_US_CONVERSION_MAGIC 83333.333 
#define MAX_STEP_RATE_FOURTH 8000
#define MAX_STEP_RATE_EIGHTH 16000
#define MIN_RPM_HALF 60
#define MIN_RPM_FOURTH 30
#define MIN_RPM_EIGHTH 5
#define MIN_RPM_SIXTEENTH 2

void enable_stepper();
void disable_stepper();
void set_direction(bool stepper, bool direction);
uint32_t rpm_to_step_delay_us(bool stepper, float rpm);
void drive_motors_rpm(float rpm_l, float rpm_r);
void set_stepper_step_size(uint8_t stepper);

#endif