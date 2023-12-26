#include "FreeRTOS.h"
#include "bt_hid.h"
#include "btstack.h"
#include "classic/sdp_server.h"
#include "hardware/flash.h"
#include "pico/async_context.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "semphr.h"
#include "task.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>

#define STAT_LED_0 18 // bt led
#define STAT_LED_1 19 // stat 1
#define STAT_LED_2 17 // stat 2
#define PWR_LED 20
#define BUZZER 16 // BUZZZZZZZ
#define STEP_L 7
#define STEP_R 9
#define DIR_L 6
#define DIR_R 8
#define MS1 2
#define MS2 3
#define MS3 4
#define ENABLE 5
#define SLEEP 10
#define RESET 11

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)
#define HIGH 1
#define LOW 0

// globals
SemaphoreHandle_t g_mutex_print;
bool g_packet_gaurd = false;

// say hi
void print_test(void *pvParameters) {
    while (true) {
        vGuardedPrint("hello world \0");
        vTaskDelay(500);
    }

    return;
}

// say hi
void print_test_2(void *pvParameters) {
    while (true) {
        vGuardedPrint("hello world \0");
        vTaskDelay(500);
    }

    return;
}

void get_inputs(void *pvParameters) {
    struct bt_hid_state state;
    while (true) {
        char buf[512];
        g_packet_gaurd = true;
        state = bt_hid_get_latest_lazy();
        g_packet_gaurd = false;
        sprintf(buf, "buttons: %04x, l: %d,%d, r: %d,%d, l2,r2: %d,%d hat: %d\n", state.buttons, state.lx, state.ly,
                state.rx, state.ry, state.l2, state.r2, state.hat);
        vGuardedPrint(buf);
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

        gpio_put(STAT_LED_0, HIGH);
        gpio_put(STAT_LED_1, LOW);
        vTaskDelay(interval);
        gpio_put(STAT_LED_0, LOW);
        gpio_put(STAT_LED_1, HIGH);
        vTaskDelay(interval);
        gpio_put(STAT_LED_1, LOW);
        gpio_put(STAT_LED_2, HIGH);
        vTaskDelay(interval);
        gpio_put(STAT_LED_2, LOW);
        gpio_put(STAT_LED_1, HIGH);
        vTaskDelay(interval);
    }
}

void beep2() {
    for (int i = 0; i < 2; i++) {
        gpio_put(BUZZER, HIGH);
        sleep_ms(50);
        gpio_put(BUZZER, LOW);
        sleep_ms(50);
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

// TODO put in another file
void setup() {

    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        panic("Wi-Fi init failed\n");
    }

    // TODO put overclocking here with a flag
    gpio_init(PWR_LED);
    gpio_init(STAT_LED_0);
    gpio_init(STAT_LED_1);
    gpio_init(STAT_LED_2);
    gpio_init(BUZZER);
    gpio_init(PWR_LED);
    gpio_init(STEP_L);
    gpio_init(STEP_R);
    gpio_init(DIR_L);
    gpio_init(DIR_R);
    gpio_init(MS1);
    gpio_init(MS2);
    gpio_init(MS3);
    gpio_init(ENABLE);
    gpio_init(SLEEP);
    gpio_init(RESET);
    gpio_set_dir(PWR_LED, GPIO_OUT);
    gpio_set_dir(STAT_LED_0, GPIO_OUT);
    gpio_set_dir(STAT_LED_1, GPIO_OUT);
    gpio_set_dir(STAT_LED_2, GPIO_OUT);
    gpio_set_dir(BUZZER, GPIO_OUT);
    gpio_set_dir(STEP_L, GPIO_OUT);
    gpio_set_dir(STEP_R, GPIO_OUT);
    gpio_set_dir(DIR_L, GPIO_OUT);
    gpio_set_dir(DIR_R, GPIO_OUT);
    gpio_set_dir(MS1, GPIO_OUT);
    gpio_set_dir(MS2, GPIO_OUT);
    gpio_set_dir(MS3, GPIO_OUT);
    gpio_set_dir(ENABLE, GPIO_OUT);
    gpio_set_dir(SLEEP, GPIO_OUT);
    gpio_set_dir(RESET, GPIO_OUT);

    // TODO change based on speed and accuracy
    gpio_put(MS1, HIGH);
    gpio_put(MS2, LOW);
    gpio_put(MS3, LOW);

    // turn on the power led
    gpio_put(PWR_LED, HIGH);

    // TODO change with debug
    uart_init(uart0, 9600);
    gpio_set_function(12, GPIO_FUNC_UART);
    gpio_set_function(13, GPIO_FUNC_UART);

    // beep2();
    // startMotor();
}

void start_core1() {
    bt_hid_init();
    vTaskStartScheduler();
    bt_main();
}


int main() {
    // run the setup script
    setup();

    // create a mutex
    g_mutex_print = xSemaphoreCreateMutex();

    TaskHandle_t print;
    TaskHandle_t print_2;
    TaskHandle_t stat_led;
    TaskHandle_t blink;
    TaskHandle_t inputs;

    // xTaskCreate(print_test, "print", 256, NULL, 1, &print);
    // xTaskCreate(print_test_2, "print2", 256, NULL, 1, &print);
    // xTaskCreate(stat_led_handle, "stat_led_handle", 256, NULL, 1, &stat_led);
    // xTaskCreate(blink_2, "blink", 256, NULL, 1, &blink);
    xTaskCreate(get_inputs, "inputs", 256, NULL, 2, &inputs);

    flash_safe_execute_core_init();
    multicore_launch_core1(start_core1);

    while (true) {
        /* do nothing */
    }

    return 1;
}