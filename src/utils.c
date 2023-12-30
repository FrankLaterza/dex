#include "utils.h"
#include "FreeRTOS.h"
#include "hardware/timer.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

// TODO fix print
// thread safe print
void vGuardedPrint(char *s) {
    xSemaphoreTake(g_mutex_print, portMAX_DELAY);
    printf(s);
    xSemaphoreGive(g_mutex_print);
}
