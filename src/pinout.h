#ifndef PINOUT_H
#define PINOUT_H

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
#define I2C_SDA 0
#define I2C_SCL 1
#define I2C_INST i2c0
#define HIGH 1
#define LOW 0

void pinout_init();


#endif