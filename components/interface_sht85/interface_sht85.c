#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sdkconfig.h>
#include <esp_log.h>

#include <memory.h>
#include <sht85.h>

#include "interface_sht85.h"

#define SHT85_FETCH_PERIOD 3000

static const char* TAG = "sht85_sensor";

static sht85_t sht85_dev;
static sht85characteristics sht85_current_data;

sht85characteristics* get_sht85_data(void) {
    return &sht85_current_data;
}

_Noreturn void sht85_loop(void) {
    esp_err_t res;

    // Start periodic measurements with 1 measurement per second and high repeatability
    ESP_ERROR_CHECK(sht85_start_measurement(&sht85_dev, SHT85_PERIODIC_1MPS, SHT85_HIGH));

    // Wait until first measurement is ready (constant time of at least 30 ms
    // or the duration returned from *sht85_get_measurement_duration*).
    vTaskDelay(sht85_get_measurement_duration(SHT85_HIGH));

    TickType_t last_wakeup = xTaskGetTickCount();

    while (1) {
        // Get the values and do something with them.
        res = sht85_get_results(&sht85_dev, &(sht85_current_data.temperature), &(sht85_current_data.humidity));
        if (res == ESP_OK)
            ESP_LOGI(TAG, "SHT85 Sensor: %.2f °C, %.2f %%\n", sht85_current_data.temperature, sht85_current_data.humidity);
        else
            ESP_LOGI(TAG, "Could not get results: %d (%s)", res, esp_err_to_name(res));

        // Wait until SHT85_FETCH_PERIOD ms are over.
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(SHT85_FETCH_PERIOD));
    }
}

esp_err_t initialize_sht85_sensor(i2c_port_t port, i2c_config_t* cfg) {
    ESP_LOGI(TAG, "Initialization started");
    memset(&sht85_dev, 0, sizeof(sht85_t));
    sht85_init_desc(&sht85_dev, port, SHT85_I2C_ADDR, cfg);

    esp_err_t ret = ESP_ERROR_CHECK_WITHOUT_ABORT(sht85_init(&sht85_dev));

    if (ret !=  ESP_OK) {
        sht85_free_desc(&sht85_dev);
        return ESP_ERR_NOT_FOUND;
    }

    xTaskCreatePinnedToCore((TaskFunction_t)sht85_loop, "SHT85 loop", configMINIMAL_STACK_SIZE * 3, NULL, 3, NULL, PRO_CPU_NUM);
    return ESP_OK;
}
