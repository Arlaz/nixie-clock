#pragma once

#include <driver/gpio.h>
#include <driver/spi_master.h>

typedef struct SHIFT_REGISTERS {
    int num_channels;
    int cascade_size;
    spi_device_handle_t spi_reg;
} sr_handle_t;

esp_err_t init_shift_registers(sr_handle_t* sr, gpio_num_t data, gpio_num_t clk, gpio_num_t latch);

esp_err_t send_data(const sr_handle_t* sr, const uint32_t* data);
