#include "bt_hid.h"
#include "pico/stdlib.h"
#include "pinout.h"
#include "stepper.h"
#include "utils.h"
#include <stdint.h>
#include <stdio.h>

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
    uint8_t center = (max - min) / 2;
    return center - in;
}

bool check_hat_lock(uint8_t position) {
    return (IS_BIT_SET(hat, position) && !IS_BIT_SET(hat_lock, position));
}

void process_hid_controls(struct bt_hid_state controls) {
    buttons = controls.buttons;
    hat = hat_convert(controls.hat);

    // L1
    if (check_button_lock(0)) {
        tune_select -= (tune_select >= 0) ? 1 : -7;
    }
    // R1
    if (check_button_lock(1)) {
        tune_select += (tune_select <= 7) ? 1 : -7;
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

    int8_t ly_axis = convert_center_axis(controls.ly, 0, 255);
    int8_t rx_axis = convert_center_axis(controls.rx, 0, 255);
    // sprintf(g_print_buf, "lx axis: %d\n", ly_axis);
    // vGuardedPrint(g_print_buf);

    float scale1 = ly_axis * 1;
    ;
    speed_filt = alpha * scale1 + (1 - alpha) * speed_filt;
    g_target_rpm = speed_filt;

    // filter steer input
    float scale2 = rx_axis * 0.40;
    steer_filt = alpha * scale2 + (1 - alpha) * steer_filt;
    g_steer = steer_filt;
    // drive_motors_rpm(ly_axis * 2, ly_axis * 2);

    // store the last state of the button
    button_lock = buttons;
    hat_lock = hat;

    print_tweaker();    
    print_balance_stats();


}

void do_nothing() {
    return;
};

void print_star() {
    sprintf(g_print_buf, "*");
    vGuardedPrint(g_print_buf);
}

/*
 * handles pid turn
 * turn select is as follows
 *
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
    }
}

void print_tweaker(){
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
    sprintf(g_print_buf, "w-mw: %f, ", pid_wheels.max_windup);
    vGuardedPrint(g_print_buf);
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
}
