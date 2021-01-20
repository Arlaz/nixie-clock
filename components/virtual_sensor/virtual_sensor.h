#pragma once

#define BME680_RW 4 //RW for "relative weight"
#define DS18B20_RW 3
#define SHT85_RW 4
#define TEMP_TOTAL_WEIGHT (BME680_RW+DS18B20_RW)
#define HUM_TOTAL_WEIGHT (BME680_RW)

#include <esp_err.h>
#include <interface_bme680.h>
#include <interface_ds18b20.h>
#include <interface_sht85.h>
#include <stdbool.h>
#include <string.h>

typedef struct SENSORS_DATA {
    float* ds18b20_temp;
    bme680characteristics* bme680_data;
    sht85characteristics* sht85_data;
} SensorsData;

typedef struct VIRTUAL_SENSOR_DATA {
    float* temperature;
    float* humidity;
    float* pressure;
    float* static_iaq;
    float* co2_equivalent;
    float* breath_voc_equivalent;
} VirtualSensorData;

esp_err_t initialize_virtual_sensor(void);
SensorsData* get_sensors(void);
VirtualSensorData* get_v_sensor(void);
