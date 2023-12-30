#include "utils.h"
#include "FreeRTOS.h"
#include "hardware/timer.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pinout.h"

// TODO fix print
// thread safe print
void vGuardedPrint(char *s) {
    xSemaphoreTake(g_mutex_print, portMAX_DELAY);
    printf(s);
    xSemaphoreGive(g_mutex_print);
}

// BUZZZZZZZ
void beep(uint8_t beep_count, uint8_t interval) {
    for (int i = 0; i < beep_count; i++) {
        gpio_put(BUZZER, HIGH);
        sleep_ms(interval);
        gpio_put(BUZZER, LOW);
        sleep_ms(interval);
    }
}