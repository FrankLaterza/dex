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
#include "pid.h"
#include "pinout.h"
#include "semphr.h"
#include "stepper.h"
#include "task.h"
#include "utils.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

// all GLOBALS (see utils)
SemaphoreHandle_t g_mutex_print;
bool g_bt_packet_gaurd = false;
char g_print_buf[1024];
uint32_t g_step_delay_period_us_left = 3000;
uint32_t g_step_delay_period_us_right = 3000;
float g_current_angle_roll;
float g_current_angle_pitch;
bool is_bt_connected = false;
float g_target_rpm = 0;
float g_steer = 0;
struct pid_t pid_wheels;
struct pid_t pid_rpm;

void bt_hid_inputs(void *pvParameters) {
    wait_for_bt_connect();
    struct bt_hid_state controls;
    while (true) {
        g_bt_packet_gaurd = true;
        controls = bt_hid_get_latest_lazy();
        g_bt_packet_gaurd = false;
        process_hid_controls(controls);
        vTaskDelay(US_TO_RTOS_TICK(20000));
    }
}

void stat_led_task(void *pvParameters) {
    wait_for_bt_connect();
    int interval = US_TO_RTOS_TICK(500000);
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
    wait_for_bt_connect();
    while (true) {
        mpu6050_poll();
        vTaskDelay(US_TO_RTOS_TICK(20000));
    }
}

void stepper_left_task(void *pvParameters) {
    wait_for_bt_connect();
    while (true) {
        set_stepper_step_size(LEFT);
        gpio_put(STEP_L, HIGH);
        vTaskDelay(US_TO_RTOS_TICK(g_step_delay_period_us_left));

        set_stepper_step_size(LEFT);
        gpio_put(STEP_L, LOW);
        vTaskDelay(US_TO_RTOS_TICK(g_step_delay_period_us_left));
    }
}

void stepper_right_task(void *pvParameters) {
    wait_for_bt_connect();
    while (true) {
        set_stepper_step_size(RIGHT);
        gpio_put(STEP_R, HIGH);
        vTaskDelay(US_TO_RTOS_TICK(g_step_delay_period_us_right));

        set_stepper_step_size(RIGHT);
        gpio_put(STEP_R, LOW);
        vTaskDelay(US_TO_RTOS_TICK(g_step_delay_period_us_right));
    }
}

void pid_task(void *pvParameters) {
    wait_for_bt_connect();
    pid_init(&pid_wheels, 0.3, 0.001, 6, 15);
    pid_init(&pid_rpm, 0.14 , 0.02, 0.01, 100);
    while (true) {
        balance(&pid_wheels, &pid_rpm);
        vTaskDelay(US_TO_RTOS_TICK(10000));
    }
}

void start_core1() {
    bt_hid_init();
    vTaskStartScheduler();
    bt_main(); // does not return
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
    TaskHandle_t pid;

    // TODO check task timing and align accordingly
    xTaskCreate(stat_led_task, "stat_led", 256, NULL, 5, &stat_led);
    xTaskCreate(bt_hid_inputs, "inputs", 256, NULL, 3, &inputs);
    xTaskCreate(stepper_left_task, "stepper_left", 256, NULL, 1, &stepper_left);
    xTaskCreate(stepper_right_task, "stepper_right", 256, NULL, 1, &stepper_right);
    xTaskCreate(mpu6050_task, "mpu6050", 256, NULL, 2, &mpu6050);
    xTaskCreate(pid_task, "pid", 256, NULL, 2, &pid);

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