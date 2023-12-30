#include "pinout.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "mpu6050.h"

void pinout_init() {

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

    // init mpu 6050 with fastest speed
    i2c_init(I2C_INST, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));

    // setup the mpu
    mpu6050_reset();
    mpu6050_config();

    // TODO change based on speed and accuracy
    gpio_put(MS1, HIGH);
    gpio_put(MS2, LOW);
    gpio_put(MS3, LOW);

    // turn on the power led
    gpio_put(PWR_LED, HIGH);

    // TODO change with debug
    // uart_init(uart0, 9600);
    // gpio_set_function(12, GPIO_FUNC_UART);
    // gpio_set_function(13, GPIO_FUNC_UART);
}
