#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "wiring.h"
#include "ds18b20.h"
#include "owb.h"
#include "owb_rmt.h"

#define DS18B20_RESOLUTION (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD (3000)  // milliseconds

static const char* TAG = "ds18b20_sensor";

static owb_rmt_driver_info rmt_driver_info;
static OneWireBus* owb;
static bool device_is_present = false;
static DS18B20_Info* ds18b20_info;
static float ds18b20_current_temperature = -273.0f;

_Noreturn static void ds18b20_update_temperature_task(void* args) {
    while (1) {
        TickType_t last_wake_time_ds18b20 = xTaskGetTickCount();
        ESP_LOGI(TAG, "Reading DS18B20 temperature");

        if (!device_is_present || owb == NULL) {
            ESP_LOGW(TAG, "Trying to read sensor which is not present or undetected");
        }
        DS18B20_ERROR error = ds18b20_convert_and_read_temp(ds18b20_info, &ds18b20_current_temperature);
        if (error != DS18B20_OK) {
            ESP_LOGE(TAG, "Error Type %d", error);
        }

        vTaskDelayUntil(&last_wake_time_ds18b20, SAMPLE_PERIOD / portTICK_PERIOD_MS);
    }
}

float* get_ds18b20_temp(void) {
    return &ds18b20_current_temperature;
}

int initialize_ds18b20_sensor(void) {
    // Override global log level
    esp_log_level_set("*", ESP_LOG_INFO);

    // To debug, use 'make menuconfig' to set default Log level to DEBUG, then uncomment:
    // esp_log_level_set("owb", ESP_LOG_DEBUG);
    // esp_log_level_set("ds18b20", ESP_LOG_DEBUG);

    // Stable readings require a brief period before communication
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Create a 1-Wire bus, using the RMT timeslot driver
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    // Find all connected devices
    ESP_LOGI(TAG, "Finding device");
    OneWireBus_SearchState search_state = {0};
    owb_search_first(owb, &search_state, &device_is_present);
    char rom_code_s[17];
    if (device_is_present) {
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        ESP_LOGI(TAG, "Device found");
    } else {
        ESP_LOGI(TAG, "Device not found");
        return 1;
    }
    ESP_LOGI(TAG, "Device ROM Code: %s", rom_code_s);

    // Create DS18B20 devices on the 1-Wire bus
    ds18b20_info = ds18b20_malloc();  // heap allocation
    ESP_LOGI(TAG, "Single device optimisations enabled");
    ds18b20_init_solo(ds18b20_info, owb);  // only one device on bus

    ds18b20_use_crc(ds18b20_info, true);  // enable CRC check on all reads
    ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);

    xTaskCreatePinnedToCore(ds18b20_update_temperature_task, "Update ds18b20 temperature", 4096, NULL, 2, NULL, 0);

    return 0;
}