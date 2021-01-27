#include "virtual_sensor.h"
#include <esp_log.h>

static const char* TAG = "virtual_sensor";
static SensorsData environmental_data = {false,NULL};

void initialize_virtual_sensor(void) {
    initialize_ds18b20_sensor();
    ESP_LOGI(TAG, "DS18B20 sensor initialized");
    environmental_data.initialized = true;
}

SensorsData* get_sensors_data(void) {
    return &environmental_data;
}
