#include "FreeRTOS.h"
#include "semphr.h"
#include "hardware/timer.h"


#ifndef UTILS_H
#define UTILS_H

// min and max helpers
#define min(a, b) ((__typeof__(a))((a) < (b) ? (a) : (b)))
#define max(a, b) ((__typeof__(a))((a) > (b) ? (a) : (b)))

/*
 * PUT TIME IN MILISECONDS AND DIVIDE BY THIS FACTOR!
 * freertos is set with a tick speed of 100000 which gives
 * it 10us accuracy, but seems to be battling resources and
 * causing timing issues. the steppers are outputing accurate
 * timeing while the "blink" task seems to be off my 40
 * milliseconds. knowing this i'm going to keep these settings
 */
#define US_TO_RTOS_TICK(us) (us / 10)

// ALL GLOBALS
extern bool g_bt_packet_gaurd;
extern SemaphoreHandle_t g_mutex_print;
extern char g_print_buf[1024];
extern enum micro_step_t g_micro_step;
extern float g_current_angle_roll;
extern float g_current_angle_pitch;
extern uint32_t g_step_delay_period_us_left;
extern uint32_t g_step_delay_period_us_right;
extern bool is_bt_connected;
extern float g_target_rpm;
extern float g_steer;


struct stopwatch_t {
    absolute_time_t start_time;
    absolute_time_t end_time;
};

void vGuardedPrint(char *out);
void print_bin_16(uint16_t num);
void print_bin_8(uint8_t num);
int map_int(int value, int fromLow, int fromHigh, int toLow, int toHigh);
float convert_from_range_float(float num, float floor, float ceiling);
void beep(uint8_t beep_count, uint8_t interval);
void wait_for_bt_connect();
 

#endif