#include "utils.h"
#include "FreeRTOS.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "pinout.h"
#include "semphr.h"
#include "stepper.h"
#include "utils.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// uncomment if lame
// #define BUZZKILL

// TODO remove my main code is not printg on any other thread.
// thread safe print
void vGuardedPrint(char *s) {
    xSemaphoreTake(g_mutex_print, portMAX_DELAY);
    printf(s);
    xSemaphoreGive(g_mutex_print);
}

void print_bin_16(uint16_t num) {
    for (int i = 15; i >= 0; i--) {
        // Use bitwise AND to check the value of the current bit
        uint16_t mask = 1 << i;
        sprintf(g_print_buf, "%d", (num & mask) ? 1 : 0);
        vGuardedPrint(g_print_buf);
        if (i % 4 == 0)
            vGuardedPrint(" ");
    }
    vGuardedPrint("\n");
}

void print_bin_8(uint8_t num) {
    for (int i = 7; i >= 0; i--) {
        // Use bitwise AND to check the value of the current bit
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

float convert_from_range_float(float num, float floor, float ceiling) {
    assert(floor < ceiling);
    num = min(num, ceiling);
    num = max(num, floor);
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
        vTaskDelay(US_TO_RTOS_TICK(500000));
    }
}