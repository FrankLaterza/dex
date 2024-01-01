#include "FreeRTOS.h"
#include "semphr.h"
#include "hardware/timer.h"


#ifndef UTILS_H
#define UTILS_H

// ALL GLOBALS
extern bool g_bt_packet_gaurd;
extern SemaphoreHandle_t g_mutex_print;
extern char g_print_buf[1024];
extern enum micro_step_t g_micro_step;
extern float g_current_angle_roll;
extern float g_current_angle_pitch;
extern uint32_t g_step_delay_period_us_left;
extern uint32_t g_step_delay_period_us_right;

struct stopwatch_t {
    absolute_time_t start_time;
    absolute_time_t end_time;
};

void vGuardedPrint(char *out);
void print_bin_16(uint16_t num);
void print_bin_8(uint8_t num);
int map(int value, int fromLow, int fromHigh, int toLow, int toHigh);
void beep(uint8_t beep_count, uint8_t interval);

#endif