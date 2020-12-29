#include "virtual_sensor.h"
#include <esp_log.h>

static const char* TAG = "virtual_sensor";

static float vs_temperature = -273.0f;
static float vs_humidity = 0.f;

static SensorsData sensors_data = {false,NULL,NULL};
static VirtualSensorData vs_data = {&vs_temperature, &vs_humidity,
                                    NULL, NULL, NULL, NULL};

void initialize_virtual_sensor(void) {

    initialize_ds18b20_sensor();
    ESP_LOGI(TAG, "DS18B20 sensor initialized");

    initialize_bme680_sensor();
    ESP_LOGI(TAG, "BME680 sensor initialized");

    sensors_data.ds18b20_temp = get_ds18b20_temp();
    sensors_data.bme680_data = get_bme680_data();

    sensors_data.initialized = true;

    vs_data.pressure = &(sensors_data.bme680_data->pressure);
    vs_data.static_iaq = &(sensors_data.bme680_data->static_iaq);
    vs_data.co2_equivalent = &(sensors_data.bme680_data->co2_equivalent);
    vs_data.breath_voc_equivalent = &(sensors_data.bme680_data->breath_voc_equivalent);
}

void update_virtual_sensor_cal_data(void) {
    vs_temperature = *(sensors_data.ds18b20_temp)*DS18B20_RW +
                   (sensors_data.bme680_data->temperature)*BME680_RW;
    vs_humidity = sensors_data.bme680_data->humidity;
}

SensorsData* get_sensors_data(void) {
    return &sensors_data;
}

VirtualSensorData* get_sensor_data(void) {
    return &vs_data;
}
