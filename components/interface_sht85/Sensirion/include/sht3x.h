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
 * \brief Sensirion SHT driver interface
 *
 * This module provides access to the SHT functionality over a generic I2C
 * interface. It supports measurements without clock stretching only.
 */

#ifndef SHT3X_H
#define SHT3X_H

#include "driver/i2c.h"
#include "iot_i2c_bus.h"
#include "esp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STATUS_OK 0
#define STATUS_ERR_BAD_DATA (-1)
#define STATUS_CRC_FAIL (-2)
#define STATUS_UNKNOWN_DEVICE (-3)
#define SHT3X_MEASUREMENT_DURATION_USEC 30000


typedef void* sht3x_handle_t;

sht3x_handle_t iot_sht3x_create(i2c_bus_handle_t bus, uint16_t dev_addr);
esp_err_t iot_sht3x_delete(i2c_bus_handle_t sensor, bool del_bus);

/**
 * Detects if a sensor is connected by reading out the ID register.
 * If the sensor does not answer or if the answer is not the expected value,
 * the test fails.
 *
 * @return 0 if a sensor was detected
 */
int16_t sht3x_probe(sht3x_handle_t sensor);

/**
 * Starts a measurement and then reads out the results. This function blocks
 * while the measurement is in progress. The duration of the measurement depends
 * on the sensor in use, please consult the datasheet.
 * Temperature is returned in [degree Celsius], multiplied by 1000,
 * and relative humidity in [percent relative humidity], multiplied by 1000.
 *
 * @param temperature   the address for the result of the temperature
 * measurement
 * @param humidity      the address for the result of the relative humidity
 * measurement
 * @return              0 if the command was successful, else an error code.
 */
int16_t sht3x_measure_blocking_read(sht3x_handle_t sensor, int32_t* temperature, int32_t* humidity);

/**
 * Starts a measurement in high precision mode. Use sht3x_read() to read out the
 * values, once the measurement is done. The duration of the measurement depends
 * on the sensor in use, please consult the datasheet.
 *
 * @return     0 if the command was successful, else an error code.
 */
int16_t sht3x_measure(sht3x_handle_t sensor);

/**
 * Reads out the results of a measurement that was previously started by
 * sht3x_measure(). If the measurement is still in progress, this function
 * returns an error.
 * Temperature is returned in [degree Celsius], multiplied by 1000,
 * and relative humidity in [percent relative humidity], multiplied by 1000.
 *
 * @param temperature   the address for the result of the temperature
 * measurement
 * @param humidity      the address for the result of the relative humidity
 * measurement
 * @return              0 if the command was successful, else an error code.
 */
int16_t sht3x_read(sht3x_handle_t sensor, int32_t* temperature, int32_t* humidity);

/**
 * Enable or disable the SHT's low power mode
 *
 * @param enable_low_power_mode 1 to enable low power mode, 0 to disable
 */
void sht3x_enable_low_power_mode(uint8_t enable_low_power_mode);

/**
 * Read out the serial number
 *
 * @param serial    the address for the result of the serial number
 * @return          0 if the command was successful, else an error code.
 */
int16_t sht3x_read_serial(sht3x_handle_t sensor, uint32_t* serial);

/**
 * Return the driver version
 *
 * @return Driver version string
 */
const char* sht3x_get_driver_version(void);

/**
 * Returns the configured SHT3x address.
 *
 * @return SHT3x_ADDRESS
 */
uint8_t sht3x_get_configured_address(void);

#ifdef __cplusplus
}
#endif

#endif /* SHT3X_H */
