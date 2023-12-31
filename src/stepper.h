#ifndef STEPPER_H
#define STEPPER_H

#define STEP_ANGLE 1.80
#define FULL 0
#define HALF 1
#define FOURTH 2
#define EIGHT 3
#define SIXTEENTH 4

void enable_stepper();
void disable_stepper();
float rpm_to_step_delay_us(float rpm);

#endif