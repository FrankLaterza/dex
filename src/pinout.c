#include "pinout.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "stepper.h"
#include "utils.h"
#include <stdio.h>
#include "hardware/vreg.h"

void pinout_init() {
    
    /*
     * note if you wish to overclock your pi pico you'll have to change the
     * spi clock speed in your pico-sdk. This will allow you to communicate 
     * with the CYW43439 chip by further dividing the spi clock. You'll need
     * to change definition in your sdk located at
     * YOUR_SDK_LOCATION/src/rp2_common/pico_cyw43_driver/cyw43_bus_pio_spi.c
     * TODO find a way to change this in the config in the cmake
    */
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(1000);
    // OVERCLOCK 🔥🔥🔥
    set_sys_clock_khz(400000, true);

    stdio_init_all();
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        panic("Wi-Fi init failed\n");
    }

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

    // start the steper at full step
    config_step_size(HALF);
    set_direction(FORWARD);

    // turn on the power led
    gpio_put(PWR_LED, HIGH);

    // TODO change with debug
    // uart_init(uart0, 9600);
    // gpio_set_function(12, GPIO_FUNC_UART);
    // gpio_set_function(13, GPIO_FUNC_UART);
}
