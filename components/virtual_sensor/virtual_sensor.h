#pragma once

#include <esp_err.h>

#include <interface_bme680.h>
#include <interface_ds18b20.h>
#include <interface_sht85.h>

#define NO_DATA_AVAILABLE -256

typedef struct SENSORS_DATA {
    float* ds18b20_temp;
    bme680characteristics* bme680_data;
    sht85characteristics* sht85_data;
} SensorsData;

typedef struct VIRTUAL_SENSOR_DATA {
    struct TEMPERATURE {
        bool data_available;
        float* value;
    } temperature;
    struct HUMIDITY {
        bool data_available;
        float* value;
    } humidity;
    struct PRESSURE {
        bool data_available;
        float* value;
    } pressure;
    struct STATIC_IAQ {
        bool data_available;
        float* value;
        uint8_t* accuracy;
    } static_iaq;
    struct CO2_EQUIVALENT {
        bool data_available;
        float* value;
    } co2_equivalent;
    struct BREATH_VOC_EQUIVALENT {
        bool data_available;
        float* value;
    } breath_voc_equivalent;
} VirtualSensorData;

esp_err_t initialize_virtual_sensor(void);

SensorsData* get_sensors_data(void);

VirtualSensorData* get_virtual_sensor_data(void);
