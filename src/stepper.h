#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>

#define STEP_ANGLE 1.80
#define FULL 0
#define HALF 1
#define FOURTH 2
#define EIGHT 3
#define SIXTEENTH 4
#define MAX_RPM_FULL 208
#define MIN_RPM_FULL 45
#define MAX_STEP_RATE_FULL 3333
#define MIN_STEP_RATE_FULL 721

void enable_stepper();
void disable_stepper();
uint32_t rpm_to_step_delay_us(float rpm);

#endif