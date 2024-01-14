#include "bt_hid.h"
#include "pico/stdlib.h"
#include "pid.h"
#include "pinout.h"
#include "stepper.h"
#include "utils.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define DEADZONE 20
#define JOYSTICK_MAX 255
#define JOYSTICK_MAX_HALF 127
#define UP 1
#define DOWN -1
#define IS_BIT_SET(value, bit_position) (((value) >> (bit_position)) & 1)
// one action per button press
static uint16_t buttons = 0;
static uint16_t hat = 0;
static uint16_t button_lock = 0;
static uint16_t hat_lock = 0;
static float speed_filt = 0.0;
static float steer_filt = 0.0;
static float alpha = 0.5;
static uint8_t tune_select = 0;

bool check_button_lock(uint8_t position) {
    return (IS_BIT_SET(buttons, position) && !IS_BIT_SET(button_lock, position));
}

/*
 * converts the messy hat format to normal button bit map_int (easier for lock)
 * bit 0 - left
 * bit 1 - down
 * bit 2 - right
 * bit 3 - up
 * bit 4 - down + left
 * bit 5 - down + right
 * bit 6 - up + right
 * bit 7 - up + left
 */

uint8_t hat_convert(uint8_t hat) {
    uint8_t bin_hat = 0;
    if (hat == 6) {
        bin_hat |= 1 << 0;
    }
    if (hat == 4) {
        bin_hat |= 1 << 1;
    }
    if (hat == 2) {
        bin_hat |= 1 << 2;
    }
    if (hat == 0) {
        bin_hat |= 1 << 3;
    }
    if (hat == 5) {
        bin_hat |= 1 << 4;
    }
    if (hat == 3) {
        bin_hat |= 1 << 5;
    }
    if (hat == 1) {
        bin_hat |= 1 << 6;
    }
    if (hat == 7) {
        bin_hat |= 1 << 7;
    }
    return bin_hat;
}

int8_t convert_center_axis(uint8_t in, uint8_t min, uint8_t max) {
    uint8_t center = (max + min) / 2;
    return in - (int8_t)center - 1;
}

int8_t check_deadzone(int8_t input, uint8_t deadzone, uint8_t max) {
    // get abs deadzone and scale
    int8_t out = abs(input) > deadzone ? map_int(abs(input), deadzone, max, 0, max) : 0;
    // get sign back
    return input >= 0 ? out : -out;
}

bool check_hat_lock(uint8_t position) {
    return (IS_BIT_SET(hat, position) && !IS_BIT_SET(hat_lock, position));
}

void process_hid_controls(struct bt_hid_state controls) {
    buttons = controls.buttons;
    hat = hat_convert(controls.hat);

    // L1
    if (check_button_lock(0)) {
        tune_select = (tune_select - 1 + 12) % 12;
    }
    // R1
    if (check_button_lock(1)) {
        tune_select = (tune_select + 1) % 12;
    }
    // L2 button
    if (check_button_lock(2)) {
        tweaker(tune_select, UP);
    }
    // R2 button
    if (check_button_lock(3)) {
        tweaker(tune_select, DOWN);
    }
    // SHARE
    if (check_button_lock(4)) {
    }
    // OPTIONS
    if (check_button_lock(5)) {
    }
    // LEFT STICK PRESS
    if (check_button_lock(6)) {
    }
    // RIGHT STICK PRESS
    if (check_button_lock(7)) {
    }
    // HOME
    if (check_button_lock(8)) {
    }
    // TOUCH PAD PRESS
    if (check_button_lock(9)) {
    }
    if (check_button_lock(10)) {
    }
    if (check_button_lock(11)) {
    }
    // SQAURE
    if (check_button_lock(12)) {
    }
    // X
    if (check_button_lock(13)) {
        enable_stepper();
        vGuardedPrint("steppered enabled\n");
    }
    // CIRCLE
    if (check_button_lock(14)) {
        disable_stepper();
        vGuardedPrint("steppered disabled\n");
    }
    // TRIANGLE
    if (check_button_lock(15)) {
    }
    // LEFT HAT
    if (check_hat_lock(0)) {
    }
    // DOWN HAT
    if (check_hat_lock(1)) {
        // trim angle down
        tune_center_down();
    }
    // RIGHT HAT
    if (check_hat_lock(2)) {
    }
    // UP HAT
    if (check_hat_lock(3)) {
        // trim angle up
        tune_center_up();
    }
    // DOWN LEFT HAT
    if (check_hat_lock(4)) {
    }
    // DOWN RIGHT
    if (check_hat_lock(5)) {
    }
    // UP RIGHT HAT
    if (check_hat_lock(6)) {
    }
    // UP LEFT HAT
    if (check_hat_lock(7)) {
    }

    // set the step period
    // g_step_delay_period_us_left = rpm_to_step_delay_us(map_int(controls.ry, 0, 255, MIN_RPM_FULL, MAX_RPM_FULL));
    // g_step_delay_period_us_right = rpm_to_step_delay_us(map_int(controls.ry, 0, 255, MIN_RPM_FULL, MAX_RPM_FULL));

    // convert LY axis
    int8_t ly_axis = convert_center_axis(controls.ly, 0, JOYSTICK_MAX);
    ly_axis = check_deadzone(ly_axis, DEADZONE, JOYSTICK_MAX_HALF);
    // uint8_t ly_axis = controls.ly;
    // scale and filter throttle
    float scale1 = (float)ly_axis * 3; // max target is about 400 rpm
    
    speed_filt = alpha * scale1 + (1 - alpha) * speed_filt;
    g_target_rpm = speed_filt;

    // convert RX axis
    int8_t rx_axis = convert_center_axis(controls.rx, 0, JOYSTICK_MAX);
    rx_axis = check_deadzone(rx_axis, DEADZONE, JOYSTICK_MAX_HALF);
    // uint8_t ly_axis = controls.ly;
    // scale and filter throttle
    float scale2 = (float)rx_axis * 1.0;
    steer_filt = alpha * scale2 + (1 - alpha) * steer_filt;
    g_steer = steer_filt;

    // sprintf(g_print_buf, "speed: %f, steer %f\n", speed_filt, steer_filt);
    // vGuardedPrint(g_print_buf);

    // store the last state of the button
    button_lock = buttons;
    hat_lock = hat;

    print_tweaker(tune_select);
    print_balance_stats();
    vGuardedPrint("\n");
}
