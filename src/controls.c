#include "bt_hid.h"
#include "pico/stdlib.h"
#include "pinout.h"
#include "stepper.h"
#include "utils.h"
#include <stdint.h>
#include <stdio.h>

#define IS_BIT_SET(value, bit_position) (((value) >> (bit_position)) & 1)
// one action per button press
uint16_t buttons = 0;
uint16_t hat = 0;
uint16_t button_lock = 0;
uint16_t hat_lock = 0;

bool check_button_lock(uint8_t position) {
    return (IS_BIT_SET(buttons, position) && !IS_BIT_SET(button_lock, position));
}

/*
 * converts the messy hat format to normal button bit map (easier for lock)
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

bool check_hat_lock(uint8_t position) {
    return (IS_BIT_SET(hat, position) && !IS_BIT_SET(hat_lock, position));
}

void process_hid_controls(struct bt_hid_state controls) {
    buttons = controls.buttons;
    hat = hat_convert(controls.hat);
    // print_bin_8(hat);
    // L1
    if (check_button_lock(0)) {
        vGuardedPrint("button 1 pressed");
    }
    // R1
    if (check_button_lock(1)) {
        vGuardedPrint("button 2 pressed");
    }
    // L2 button
    if (check_button_lock(2)) {
        vGuardedPrint("button 3 pressed");
    }
    // R2 button
    if (check_button_lock(3)) {
        vGuardedPrint("button 4 pressed");
    }
    // SHARE
    if (check_button_lock(4)) {
        vGuardedPrint("button 5 pressed");
    }
    // OPTIONS
    if (check_button_lock(5)) {
        vGuardedPrint("button 6 pressed");
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
        // beep(2, 50);
        sprintf(g_print_buf, "%uus | ", rpm_to_step_delay_us(map(controls.ry, 0, 255, 5, 300)));
        vGuardedPrint(g_print_buf);
        sprintf(g_print_buf, "%urmp\n", (map(controls.ry, 0, 255, MIN_RPM_FULL, MAX_RPM_FULL)));
        vGuardedPrint(g_print_buf);
    }
    // LEFT HAT
    if (check_hat_lock(0)) {
    }
    // DOWN HAT
    if (check_hat_lock(1)) {
    }
    // RIGHT HAT
    if (check_hat_lock(2)) {
    }
    // UP HAT
    if (check_hat_lock(3)) {
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

    // put analog controls right here

    // im patching this by hand for now
    // g_step_delay_period_us = rpm_to_step_delay_us(60);
    // sprintf(g_print_buf, "step delay %fus\n", g_step_delay_period_us);
    // vGuardedPrint(g_print_buf);

    // sprintf(g_print_buf, "ry %u | ", controls.ry);
    // vGuardedPrint(g_print_buf);
    // sprintf(g_print_buf, "rmp map %u | ", map(controls.ry, 0, 255, 10, 200));
    // vGuardedPrint(g_print_buf);
    // sprintf(g_print_buf, "us %u\n", rpm_to_step_delay_us(map(controls.ry, 0, 255, 10, 200)));
    // vGuardedPrint(g_print_buf);

    // set the step period
    // g_step_delay_period_us_left = rpm_to_step_delay_us(map(controls.ry, 0, 255, MIN_RPM_FULL, MAX_RPM_FULL));
    // g_step_delay_period_us_right = rpm_to_step_delay_us(map(controls.ry, 0, 255, MIN_RPM_FULL, MAX_RPM_FULL));

    // store the last state of the button
    button_lock = buttons;
    hat_lock = hat;
}
