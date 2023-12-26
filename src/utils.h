#include "FreeRTOS.h"
#include "semphr.h"

#ifndef UTILS_H
#define UTILS_H

extern bool g_packet_gaurd;
extern SemaphoreHandle_t g_mutex_print;

void vGuardedPrint(char *out);

#endif