#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "bt_hid.h"
#include "btstack.h"
#include "classic/sdp_server.h"
#include "pico/async_context.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "semphr.h"
#include "task.h"

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

struct Data {
    // protected data
    bool state;
    SemaphoreHandle_t mutex;
};
struct Data data;

SemaphoreHandle_t mutex_print;
// vTaskCoreAffinitySet(inputs, CORE_1);
// vTaskCoreAffinitySet(bt, CORE_0);
// blink a light and fill mutex
void led_task(void *pvParameters) {
    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

        data.state = true;
        vTaskDelay(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LOW);

        data.state = false;
        vTaskDelay(500);
    }
}

// say hi
void print_from_second_thread(void *pvParameters) {
    bool last_state = false;
    while (true) {
        if (data.state != last_state) {
            printf("updated state is %s\r\n", data.state ? "true" : "false");
            last_state = data.state;
        }
        vTaskDelay(50);
    }

    return;
}

void get_inputs(void *pvParameters) {

    struct bt_hid_state state;
    while (true) {
        bt_hid_get_latest(&state);
        printf("buttons: %04x, l: %d,%d, r: %d,%d, l2,r2: %d,%d hat: %d\n", state.buttons, state.lx, state.ly, state.rx,
               state.ry, state.l2, state.r2, state.hat);
        vTaskDelay(20);
    }
}

void stat_led_handle(void *pvParameters) {
    int interval = 400;
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

void beep5() {
    for (int i = 0; i < 5; i++) {
        gpio_put(BUZZER, HIGH);
        sleep_ms(100);
        gpio_put(BUZZER, LOW);
        sleep_ms(100);
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

void setup() {

    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }

    // init gpio
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

    // configure half stepping TODO change based on speed and accuracy
    gpio_put(MS1, HIGH);
    gpio_put(MS2, LOW);
    gpio_put(MS3, LOW);

    // turn on the power led
    gpio_put(PWR_LED, HIGH);

    beep5();
    startMotor();
}

int main() {

    // run the setup script
    setup();

    // create a mutex
    data.mutex = xSemaphoreCreateMutex();
    data.state = false;

    TaskHandle_t blink;
    TaskHandle_t print;
    TaskHandle_t stat_led;
    // TaskHandle_t inputs;
    TaskHandle_t bt;

    xTaskCreate(led_task, "led", 256, NULL, 1, &blink);
    xTaskCreate(print_from_second_thread, "print", 256, NULL, 1, &print);
    xTaskCreate(stat_led_handle, "stat_led_handle", 256, NULL, 1, &stat_led);
    // xTaskCreate(get_inputs, "inputs", 256, NULL, 2, &inputs);
    // xTaskCreate(bt_main, "bt", 256, NULL, 2, &bt);

    vTaskCoreAffinitySet(blink, CORE_1);
    vTaskCoreAffinitySet(print, CORE_1);
    vTaskCoreAffinitySet(stat_led, CORE_1);
    // vTaskCoreAffinitySet(inputs, CORE_1);
    // vTaskCoreAffinitySet(bt, CORE_0);

    vTaskStartScheduler();

    while (1) {
        ;
    }

    return 1;
}