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
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pid.h"

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

/*
 * PUT TIME IN MILISECONDS AND DIVIDE BY THIS FACTOR!
 * freertos is set with a tick speed of 100000 which gives
 * it 10us accuracy, but seems to be battling resources and
 * causing timing issues. the steppers are outputing accurate
 * timeing while the "blink" task seems to be off my 40
 * milliseconds. knowing this i'm going to keep these settings
 */
#define US_TO_RTOS_TICK(us) (us / 10)

// all GLOBALS (see utils)
SemaphoreHandle_t g_mutex_print;
bool g_bt_packet_gaurd = false;
char g_print_buf[1024];
uint32_t g_step_delay_period_us_left = 3000;
uint32_t g_step_delay_period_us_right = 3000;
float g_current_angle_roll;
float g_current_angle_pitch;

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
        vTaskDelay(US_TO_RTOS_TICK(50000));
    }
}

void stat_led_handle(void *pvParameters) {
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
    while (true) {
        mpu6050_poll();
        vTaskDelay(US_TO_RTOS_TICK(20000));
    }
}
// static struct stopwatch_t stopwatch1;
void stepper_left_task(void *pvParameters) {
    while (true) {
        // start time
        // stopwatch1.start_time = get_absolute_time();
        gpio_put(STEP_L, HIGH);
        vTaskDelay(US_TO_RTOS_TICK(g_step_delay_period_us_left));
        // end time
        // stopwatch1.end_time = get_absolute_time();
        // print elapsed
        // sprintf(g_print_buf, "elpased time %u | step %u\n", elapsed_time(stopwatch1), g_step_delay_period_us_left);
        // vGuardedPrint(g_print_buf);
        gpio_put(STEP_L, LOW);
        vTaskDelay(US_TO_RTOS_TICK(g_step_delay_period_us_left));
    }
}

void stepper_right_task(void *pvParameters) {
    while (true) {
        gpio_put(STEP_R, HIGH);
        vTaskDelay(US_TO_RTOS_TICK(g_step_delay_period_us_right));
        gpio_put(STEP_R, LOW);
        vTaskDelay(US_TO_RTOS_TICK(g_step_delay_period_us_right));
    }
}

void process_pid(void *pvParameters) {
    
    struct pid_t pid_wheels;
    pid_init(&pid_wheels, 5, 0.0000, 0.2);
    while (true) {
        balance(&pid_wheels);
        vTaskDelay(US_TO_RTOS_TICK(100000));
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
    TaskHandle_t pid;

    // TODO check task timing and align accordingly
    xTaskCreate(stat_led_handle, "stat_led_handle", 256, NULL, 5, &stat_led);
    xTaskCreate(bt_hid_inputs, "inputs", 256, NULL, 3, &inputs);
    xTaskCreate(stepper_left_task, "stepper_left", 256, NULL, 1, &stepper_left);
    xTaskCreate(stepper_right_task, "stepper_right", 256, NULL, 1, &stepper_right);
    xTaskCreate(mpu6050_task, "mpu6050", 256, NULL, 2, &mpu6050);
    xTaskCreate(process_pid, "pid", 256, NULL, 2, &pid);

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