#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include <esp_log.h>
#include <sht3x.h>

#include "interface_sht85.h"

static const char* TAG = "sht85_sensor";

static sht85characteristics sht85_current_data;

sht85characteristics* get_sht85_data(void) {
    return &sht85_current_data;
}

_Noreturn void sht85_loop(void) {
    int32_t temperature, humidity;
	while (1) {
		int8_t ret = sht3x_measure_blocking_read(&temperature, &humidity);
		if (ret == STATUS_OK) {
            sht85_current_data.temperature = temperature / 1000.0f;
            sht85_current_data.humidity = humidity / 1000.0f;
			ESP_LOGI(TAG, "measured temperature: %0.2f degree Celsius", sht85_current_data.temperature);
		    ESP_LOGI(TAG, "measured humidity: %0.2f %% RH", sht85_current_data.humidity);
        }
		else {
			ESP_LOGE(TAG, "Error reading measurement, code : %d", ret);
		}
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

esp_err_t initialize_sht85_sensor(void) {
    sensirion_i2c_init();
    xTaskCreatePinnedToCore((TaskFunction_t)sht85_loop, "SHT85 loop", 1024, NULL, 3, NULL, 0);
    return ESP_OK;
}