#include "FreeRTOS.h"
#include "bt_hid.h"
#include "btstack.h"
#include "classic/sdp_server.h"
#include "hardware/flash.h"
#include "pico/flash.h"
#include "mpu6050.h"
#include "pico/async_context.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pinout.h"
#include "semphr.h"
#include "task.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

// globals
SemaphoreHandle_t g_mutex_print;
bool g_bt_packet_gaurd = false;
char g_print_buf[1024];

void print_inputs(void *pvParameters) {
    struct bt_hid_state state;
    while (true) {
        g_bt_packet_gaurd = true;
        state = bt_hid_get_latest_lazy();
        g_bt_packet_gaurd = false;
        // sprintf(g_print_buf, "buttons: %04x, l: %d,%d, r: %d,%d, l2,r2: %d,%d hat: %d\n", state.buttons, state.lx, state.ly,
        //         state.rx, state.ry, state.l2, state.r2, state.hat);
        // TODO make all prints point to this gaurded print
        // vGuardedPrint(g_print_buf);
        if (state.buttons > 0) {
            gpio_put(PWR_LED, HIGH);
        } else {
            gpio_put(PWR_LED, LOW);
        }
        vTaskDelay(20);
    }
}

void stat_led_handle(void *pvParameters) {
    int interval = 500;
    while (true) {
        gpio_put(STAT_LED_1, LOW);
        gpio_put(STAT_LED_2, HIGH);
        vTaskDelay(interval);
        gpio_put(STAT_LED_2, LOW);
        gpio_put(STAT_LED_1, HIGH);
        vTaskDelay(interval);
    }
}

void mpu6050_task(void *pvParameters) {
    while (true) {
        mpu6050_poll();
        vTaskDelay(20);
    }
}

void startMotor() {
    // enable the motor
    gpio_put(SLEEP, HIGH);
    gpio_put(RESET, HIGH);
    sleep_ms(2000);
    gpio_put(SLEEP, LOW);
    gpio_put(RESET, LOW);
}

void start_core1() {
    bt_hid_init();
    vTaskStartScheduler();
    bt_main();
}

int main() {
    // run the setup script
    pinout_init();
    // create a mutex
    g_mutex_print = xSemaphoreCreateMutex();

    TaskHandle_t stat_led;
    TaskHandle_t inputs;
    TaskHandle_t mpu6050;

    xTaskCreate(stat_led_handle, "stat_led_handle", 256, NULL, 1, &stat_led);
    xTaskCreate(print_inputs, "inputs", 256, NULL, 2, &inputs);
    // xTaskCreate(mpu6050_task, "mpu6050", 256, NULL, 2, &mpu6050);

    // beep after setup
    beep(3, 50);

    flash_safe_execute_core_init();
    multicore_launch_core1(start_core1);

    while (true) {
        /* do nothing */
    }

    return 1;
}