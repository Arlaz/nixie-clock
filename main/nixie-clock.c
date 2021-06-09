#include <esp_log.h>
#include <nvs_flash.h>

#include <wiring.h>
#include <leds_light.h>
#include <nixie_display.h>
#include <virtual_sensor.h>
#include <app_wifi.h>
#include <server_homekit.h>

static const char* TAG = "Main";

void app_main(void) {

    /* Initialize NVS partition */
    esp_err_t err_nvs_flash = ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_init());
    if (err_nvs_flash == ESP_ERR_NVS_NO_FREE_PAGES || err_nvs_flash == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());
        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    pin_config();

    init_leds();
    init_display();

    initialize_virtual_sensor();

    /* Initialize Wi-Fi */
    app_wifi_init();

    ESP_LOGI(TAG, "Wifi initialized");

    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);

    ESP_LOGI(TAG, "Wifi started");

    homekit_server_start();
    ESP_LOGI(TAG, "stack remaining free in main %d", uxTaskGetStackHighWaterMark(NULL));
}
