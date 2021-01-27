#pragma once

#include <stdbool.h>
#include <string.h>
#include "interface_ds18b20.h"

typedef struct SENSORS_DATA {
    bool initialized;
    float* ds18b20_temp;
} SensorsData;

typedef struct VIRTUAL_SENSOR_DATA {
    float* temp;
} VSensorData;

void initialize_virtual_sensor(void);
SensorsData* get_sensors_data(void);
