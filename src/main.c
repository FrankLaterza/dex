#include "FreeRTOS.h"
#include "bt_hid.h"
#include "btstack.h"
#include "classic/sdp_server.h"
#include "controls.h"
#include "hardware/flash.h"
#include "mpu6050.h"
#include "pico/async_context.h"
#include "pico/cyw43_arch.h"
#include "pico/flash.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pinout.h"
#include "semphr.h"
#include "stepper.h"
#include "task.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

// all extern globals
SemaphoreHandle_t g_mutex_print;
bool g_bt_packet_gaurd = false;
char g_print_buf[1024];
uint32_t g_step_delay_period_us_left = 3000;
uint32_t g_step_delay_period_us_right = 3000;

/*
 * idk if i want to keep the tasks here or put them in
 * another file called tasks.c or put them in their
 * related files???
 */

void bt_hid_inputs(void *pvParameters) {
    struct bt_hid_state controls;
    while (true) {
        g_bt_packet_gaurd = true;
        controls = bt_hid_get_latest_lazy();
        g_bt_packet_gaurd = false;
        process_hid_controls(controls);
        vTaskDelay(50 * 1000);
    }
}

void stat_led_handle(void *pvParameters) {
    int interval = 500 * 1000;
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
        vTaskDelay(20 * 1000);
    }
}

void stepper_left_task(void *pvParameters) {
    while (true) {
        // 800us
        gpio_put(STEP_L, HIGH);        
        vTaskDelay(g_step_delay_period_us_left);
        gpio_put(STEP_L, LOW);
        vTaskDelay(g_step_delay_period_us_left);
    }
}

void stepper_right_task(void *pvParameters) {
    while (true) {
        // 800us
        gpio_put(STEP_R, HIGH);
        vTaskDelay(g_step_delay_period_us_right);
        gpio_put(STEP_R, LOW);
        vTaskDelay(g_step_delay_period_us_right);
    }
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
    TaskHandle_t stepper_left;
    TaskHandle_t stepper_right;
    TaskHandle_t mpu6050;

    // TODO check task timing and align accordingly
    xTaskCreate(stat_led_handle, "stat_led_handle", 256, NULL, 5, &stat_led);
    // xTaskCreate(bt_hid_inputs, "inputs", 256, NULL, 3, &inputs);
    // xTaskCreate(stepper_left_task, "stepper_left", 256, NULL, 1, &stepper_left);
    // xTaskCreate(stepper_right_task, "stepper_right", 256, NULL, 1, &stepper_right);
    // xTaskCreate(mpu6050_task, "mpu6050", 256, NULL, 2, &mpu6050);

    // beep after setup
    beep(3, 50);

    // TODO check resources. what core is i2c running on?
    flash_safe_execute_core_init();
    multicore_launch_core1(start_core1);

    while (true) {
        /* do nothing */
    }

    return 1;
}