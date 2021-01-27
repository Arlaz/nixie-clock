#include <esp_event.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <math.h>

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <homekit/characteristics.h>
#include <homekit/homekit.h>
#include "wifi.h"

#include <wiring.h>

#include "server_homekit.h"

static const char* TAG = "HomeKit_Server";

homekit_characteristic_t temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t relative_humidity = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);
homekit_characteristic_t air_quality = HOMEKIT_CHARACTERISTIC_(AIR_QUALITY, 0);
homekit_characteristic_t carbon_dioxide_equivalent = HOMEKIT_CHARACTERISTIC_(CARBON_DIOXIDE_LEVEL, 0);
homekit_characteristic_t voc_equivalent = HOMEKIT_CHARACTERISTIC_(VOC_DENSITY, 0);

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && (event_id == WIFI_EVENT_STA_START || event_id == WIFI_EVENT_STA_DISCONNECTED)) {
        ESP_LOGI(TAG,"STA start");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG,"WiFI ready");
        on_wifi_ready();
    }
}

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT() ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = WIFI_SSID,
                .password = WIFI_PASSWORD,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static bool s_led_on = false;

void led_write(bool on) {
    gpio_set_level(GPIO_LED, on ? 1 : 0);
}

void led_init(void) {
    PIN_FUNC_SELECT(GPIO_PIN_REG_13, PIN_FUNC_GPIO);
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
    led_write(s_led_on);
}

void led_identify_task(void* _args) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            led_write(true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            led_write(false);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    led_write(s_led_on);

    vTaskDelete(NULL);
}

void clock_identify(homekit_value_t _value) {
    ESP_LOGI(TAG,"Clock identify");
    xTaskCreate(led_identify_task, "LED identify", 512, NULL, 2, NULL);
}

homekit_value_t led_on_get(void) {
    return HOMEKIT_BOOL(s_led_on);
}

void led_on_set(homekit_value_t value) {
    if (value.format != homekit_format_bool) {
        ESP_LOGI(TAG,"Invalid value format: %d", value.format);
        return;
    }

    s_led_on = value.bool_value;
    led_write(s_led_on);
}

homekit_accessory_t* accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_thermostat, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "LED and TEMP Sensor"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "MathisK"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "037A2BA5BF19D"),
            HOMEKIT_CHARACTERISTIC(MODEL, "MyTemperatureSensor"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, clock_identify),
            NULL}),
                      HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
                          HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"), &temperature,
                          NULL}),
                      HOMEKIT_SERVICE(HUMIDITY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
                          HOMEKIT_CHARACTERISTIC(NAME, "Humidity Sensor"), &relative_humidity,
                          NULL}),
                      HOMEKIT_SERVICE(AIR_QUALITY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
                          HOMEKIT_CHARACTERISTIC(NAME, "Air Quality Sensor"), &air_quality, &carbon_dioxide_equivalent, &voc_equivalent,
                          NULL}),
                      NULL}),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "121-14-181",
};

void on_wifi_ready(void) {
    homekit_server_init(&config);
}

void homekit_server_start(void) {
    wifi_init();
    led_init();
}
