#pragma once

#include <driver/gpio.h>

static gpio_num_t GPIO_SENSORS_SDA;
static gpio_num_t GPIO_RTC_SDA;
static gpio_num_t GPIO_HVDRIVER_DATA;
static gpio_num_t GPIO_HVDRIVER_CLK;

static const gpio_num_t GPIO_DS18B20 = GPIO_NUM_32;
static const gpio_num_t GPIO_SENSORS_SCL = GPIO_NUM_4;

static const gpio_num_t GPIO_RTC_SCL = GPIO_NUM_22;

static const gpio_num_t GPIO_SD_MISO = GPIO_NUM_12;
static const gpio_num_t GPIO_SD_MOSI = GPIO_NUM_26;
static const gpio_num_t GPIO_SD_CLK = GPIO_NUM_14;
static const gpio_num_t GPIO_SD_CS = GPIO_NUM_27;
static const gpio_num_t GPIO_SD_DET = GPIO_NUM_25;

static const gpio_num_t GPIO_BUZZER = GPIO_NUM_33;
static const gpio_num_t GPIO_LED = GPIO_NUM_13;

static const gpio_num_t GPIO_BUTTON_MODE = GPIO_NUM_34;
static const gpio_num_t GPIO_BUTTON_UP = GPIO_NUM_36;
static const gpio_num_t GPIO_BUTTON_DOWN = GPIO_NUM_39;

static const gpio_num_t GPIO_HVDRIVER_LE = GPIO_NUM_19;
static const gpio_num_t GPIO_HVDRIVER_BL = GPIO_NUM_21;

void pin_config(void);
