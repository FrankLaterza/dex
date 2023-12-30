#include "FreeRTOS.h"
#include "semphr.h"

#ifndef UTILS_H
#define UTILS_H

extern bool g_bt_packet_gaurd;
extern SemaphoreHandle_t g_mutex_print;
extern char g_print_buf[1024];

void vGuardedPrint(char *out);
void beep(uint8_t beep_count, uint8_t interval);

#endif