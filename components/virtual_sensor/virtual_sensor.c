#include "virtual_sensor.h"
#include <esp_err.h>
#include <esp_log.h>

static const char* TAG = "virtual_sensor";

static float vs_temperature = -273.0f;
static float vs_humidity = 0.f;

static SensorsData sensors_data = {NULL,NULL,NULL};
static VirtualSensorData vs_data = {&vs_temperature, &vs_humidity,
                                    NULL, NULL, NULL, NULL};

esp_err_t initialize_virtual_sensor(void) {
    esp_err_t ret;

    ret = initialize_ds18b20_sensor();
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "DS18B20 sensor initialization error : %d", ret);
    } else {
        ESP_LOGI(TAG, "DS18B20 sensor initialized");
        sensors_data.ds18b20_temp = get_ds18b20_temp();
    }

    ret = initialize_bme680_sensor();
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "BME680 sensor initialization error : %d", ret);
    } else {
        ESP_LOGI(TAG, "BME680 sensor initialized");
        sensors_data.bme680_data = get_bme680_data();
        vs_data.pressure = &(sensors_data.bme680_data->pressure);
        vs_data.static_iaq = &(sensors_data.bme680_data->static_iaq);
        vs_data.co2_equivalent = &(sensors_data.bme680_data->co2_equivalent);
        vs_data.breath_voc_equivalent = &(sensors_data.bme680_data->breath_voc_equivalent);
    }

    ret = initialize_sht85_sensor();
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "SHT85 sensor initialization error : %d", ret);
    } else {
        ESP_LOGI(TAG, "SHT85 sensor initialized");
        sensors_data.sht85_data = get_sht85_data();
    }

    return ESP_OK;
}

void update_virtual_sensor_cal_data(void) {
    vs_temperature =    *(sensors_data.ds18b20_temp)*DS18B20_RW +
                        (sensors_data.bme680_data->temperature)*BME680_RW+
                        (sensors_data.sht85_data->temperature)*SHT85_RW;

    vs_humidity = sensors_data.bme680_data->humidity*SHT85_RW + sensors_data.bme680_data->humidity*SHT85_RW;

    vs_temperature /= TEMP_TOTAL_WEIGHT;
    vs_humidity /= HUM_TOTAL_WEIGHT;
}

SensorsData* get_sensors_data(void) {
    return &sensors_data;
}

VirtualSensorData* get_sensor_data(void) {
    return &vs_data;
}
