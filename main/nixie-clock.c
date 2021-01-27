#include <nvs_flash.h>
#include <wiring.h>
#include <server_homekit.h>

void app_main(void) {
    // Initialize NVS
    esp_err_t err_nvs_flash = nvs_flash_init();
    if (err_nvs_flash == ESP_ERR_NVS_NO_FREE_PAGES || err_nvs_flash == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated or updated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err_nvs_flash = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err_nvs_flash);

    pin_config();
    homekit_server_start();
}