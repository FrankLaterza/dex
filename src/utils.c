#include "utils.h"
#include "FreeRTOS.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "pinout.h"
#include "semphr.h"
#include "stepper.h"
#include "utils.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// * uncomment if you're a buzz kill
// #define BUZZKILL
// * uncomment if you don't want to print anything
// #define SHUT_UP

// ? not sure if threadsafe print is used for all prints
// thread safe print
void vGuardedPrint(char *s) {
#ifndef SHUT_UP
    xSemaphoreTake(g_mutex_print, portMAX_DELAY);
    printf(s);
    xSemaphoreGive(g_mutex_print);
#endif
}

// prints a uint16 in binary
void print_bin_16(uint16_t num) {
    for (int i = 15; i >= 0; i--) {
        uint16_t mask = 1 << i;
        sprintf(g_print_buf, "%d", (num & mask) ? 1 : 0);
        vGuardedPrint(g_print_buf);
        if (i % 4 == 0)
            vGuardedPrint(" ");
    }
    vGuardedPrint("\n");
}

// prints a uint8 in binary
void print_bin_8(uint8_t num) {
    for (int i = 7; i >= 0; i--) {
        uint16_t mask = 1 << i;
        sprintf(g_print_buf, "%d", (num & mask) ? 1 : 0);
        vGuardedPrint(g_print_buf);
        if (i % 4 == 0)
            vGuardedPrint(" ");
    }
    vGuardedPrint("\n");
}

// this maps a value given from range and to range
int map_int(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    assert(fromLow < fromHigh);
    assert(toLow < toHigh);
    // truncate value into range
    value = (value < fromLow) ? fromLow : ((value > fromHigh) ? fromHigh : value);
    // mappp
    return toLow + (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow);
}

// returns a value between given floor or ceilings
float convert_from_range_float(float num, float floor, float ceiling) {
    assert(floor < ceiling);
    num = min(num, ceiling);
    num = max(num, floor);
    return num;
}

// returns a value between plus or minus the given absolute range
float convert_from_absolute_range_float(float num, float absolute_range) {
    absolute_range = fabs(absolute_range);
    num = min(num, absolute_range);
    num = max(num, -absolute_range);
    return num;
}

// BUZZZZZZZ
void beep(uint8_t beep_count, uint8_t interval) {
#ifndef BUZZKILL
    for (int i = 0; i < beep_count; i++) {
        gpio_put(BUZZER, HIGH);
        sleep_ms(interval);
        gpio_put(BUZZER, LOW);
        sleep_ms(interval);
    }
#endif
}

int64_t elapsed_time(struct stopwatch_t stopwatch) {
    return absolute_time_diff_us(stopwatch.start_time, stopwatch.end_time);
}

// waits for bt_hid.c to set connected flag
void wait_for_bt_connect() {
    while (is_bt_connected == false) {
        // wait
        vTaskDelay(US_TO_RTOS_TICK(100000));
    }
    // wait for bluetooth to do its thing
    vTaskDelay(US_TO_RTOS_TICK(1000000));
}