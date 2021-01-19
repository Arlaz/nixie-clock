#include "virtual_sensor.h"
#include <esp_log.h>

static const char* TAG = "virtual_sensor";
static SensorsData environmental_data = {false,NULL,NULL};

void initialize_virtual_sensor(void) {

    initialize_ds18b20_sensor();
    ESP_LOGI(TAG, "DS18B20 sensor initialized");

    initialize_bme680_sensor();
    ESP_LOGI(TAG, "BME680 sensor initialized");

    environmental_data.ds18b20_temp = get_ds18b20_temp();
    environmental_data.bme680_data = get_bme680_data();

    environmental_data.initialized = true;
}

SensorsData* get_sensors_data(void) {
    return &environmental_data;
}
