/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 *
 * This module provides functionality that is common to all Sensirion drivers
 */


#include <stdio.h>
#include "driver/i2c.h"
#include "iot_i2c_bus.h"
#include "esp_types.h"
#include "sensirion_common.h"
#include "freertos/task.h"
#include "esp32/rom/ets_sys.h"
#define WRITE_BIT  I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN   0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0               /*!< I2C ack value */
#define NACK_VAL   0x1               /*!< I2C nack value */


/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_sleep_usec(uint32_t us) {
    const int us_per_tick = portTICK_PERIOD_MS * 1000;
    if (us < us_per_tick) {
        ets_delay_us((uint32_t)us);
    }
    else {
        /* since vTaskDelay(1) blocks for anywhere between 0 and portTICK_PERIOD_MS,
         * round up to compensate.
         */
        vTaskDelay((us + us_per_tick - 1) / us_per_tick);
    }
}

uint8_t sensirion_common_generate_crc(const uint8_t* data, uint16_t count) {
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

int8_t sensirion_common_check_crc(const uint8_t* data, uint16_t count,
                                  uint8_t checksum) {
    if (sensirion_common_generate_crc(data, count) != checksum)
        return STATUS_FAIL;
    return STATUS_OK;
}

uint16_t sensirion_fill_cmd_send_buf(uint8_t* buf, uint16_t cmd,
                                     const uint16_t* args, uint8_t num_args) {
    uint8_t crc;
    uint8_t i;
    uint16_t idx = 0;

    buf[idx++] = (uint8_t)((cmd & 0xFF00) >> 8);
    buf[idx++] = (uint8_t)((cmd & 0x00FF) >> 0);

    for (i = 0; i < num_args; ++i) {
        buf[idx++] = (uint8_t)((args[i] & 0xFF00) >> 8);
        buf[idx++] = (uint8_t)((args[i] & 0x00FF) >> 0);

        crc = sensirion_common_generate_crc((uint8_t*)&buf[idx - 2],
                                            SENSIRION_WORD_SIZE);
        buf[idx++] = crc;
    }
    return idx;
}

int16_t sensirion_i2c_read_words_as_bytes(i2c_bus_handle_t bus, uint8_t address, uint8_t* data,
                                          uint16_t num_words) {
    int32_t ret;
    uint16_t i, j;
    uint16_t size = num_words * (SENSIRION_WORD_SIZE + CRC8_LEN);
    uint16_t word_buf[SENSIRION_MAX_BUFFER_WORDS];
    uint8_t* const buf8 = (uint8_t*)word_buf;

#if 1
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, buf8, size,I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ESP_FAIL;
    }
#else
    ret = sensirion_i2c_read(address, buf8, size);
    if (ret != NO_ERROR)
        return ret;
#endif
    /* check the CRC for each word */
    for (i = 0, j = 0; i < size; i += SENSIRION_WORD_SIZE + CRC8_LEN) {

        ret = sensirion_common_check_crc(&buf8[i], SENSIRION_WORD_SIZE,
                                         buf8[i + SENSIRION_WORD_SIZE]);
        if (ret != STATUS_OK)
            return ret;

        data[j++] = buf8[i];
        data[j++] = buf8[i + 1];
    }

    return STATUS_OK;
}

int16_t sensirion_i2c_read_words(i2c_bus_handle_t bus, uint8_t address, uint16_t* data_words,
                                 uint16_t num_words) {
    int16_t ret;
    uint8_t i;
    const uint8_t* word_bytes;

    ret = sensirion_i2c_read_words_as_bytes(bus, address, (uint8_t*)data_words, num_words);
    if (ret != STATUS_OK)
        return ret;

    for (i = 0; i < num_words; ++i) {
        word_bytes = (uint8_t*)&data_words[i];
        data_words[i] = ((uint16_t)word_bytes[0] << 8) | word_bytes[1];
    }

    return STATUS_OK;
}

int16_t sensirion_i2c_write_cmd(i2c_bus_handle_t bus, uint8_t address, uint16_t command) {
    uint8_t buf[SENSIRION_COMMAND_SIZE];

    sensirion_fill_cmd_send_buf(buf, command, NULL, 0);
#if 1
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, buf, SENSIRION_COMMAND_SIZE, true);
    i2c_master_stop(cmd);
    int ret = iot_i2c_bus_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
#else
    return sensirion_i2c_write(address, buf, SENSIRION_COMMAND_SIZE);
#endif
}

int16_t sensirion_i2c_write_cmd_with_args(i2c_bus_handle_t bus, uint8_t address, uint16_t command,
                                          const uint16_t* data_words,
                                          uint16_t num_words) {
    uint8_t buf[SENSIRION_MAX_BUFFER_WORDS];
    uint16_t buf_size;

    buf_size = sensirion_fill_cmd_send_buf(buf, command, data_words, num_words);
#if 1
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, buf, buf_size, true);
    i2c_master_stop(cmd);
    int ret = iot_i2c_bus_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
#else
    return sensirion_i2c_write(address, buf, buf_size);
#endif
}

int16_t sensirion_i2c_delayed_read_cmd(i2c_bus_handle_t bus, uint8_t address, uint16_t user_cmd,
                                       uint32_t delay_us, uint16_t* data_words,
                                       uint16_t num_words) {
    int32_t ret;
    uint8_t buf[SENSIRION_COMMAND_SIZE];

    sensirion_fill_cmd_send_buf(buf, user_cmd, NULL, 0);
#if 1
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, buf, SENSIRION_COMMAND_SIZE, true);
    i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
#else
    ret = sensirion_i2c_write(address, buf, SENSIRION_COMMAND_SIZE);
#endif
    if (ret != STATUS_OK)
        return ret;

    if (delay_us)
        sensirion_sleep_usec(delay_us);

    return sensirion_i2c_read_words(bus, address, data_words, num_words);
}

int16_t sensirion_i2c_read_cmd(i2c_bus_handle_t bus, uint8_t address, uint16_t cmd,
                               uint16_t* data_words, uint16_t num_words) {
    return sensirion_i2c_delayed_read_cmd(bus, address, cmd, 0, data_words,
                                          num_words);
}
