#pragma once

#define BME680_RW 4 //RW for "relative weight"
#define DS18B20_RW 3
#define TEMP_TOTAL_WEIGHT (BME680_RW+DS18B20_RW)
#define HUM_TOTAL_WEIGHT (BME680_RW)

#include <stdbool.h>
#include <string.h>
#include "interface_bme680.h"
#include "interface_ds18b20.h"

typedef struct SENSORS_DATA {
    bool initialized;
    float* ds18b20_temp;
    bme680characteristics* bme680_data;
} SensorsData;

typedef struct VIRTUAL_SENSOR_DATA {
    float* temp;
} VSensorData;

void initialize_virtual_sensor(void);
SensorsData* get_sensors_data(void);
