#include <esp_err.h>
#include <esp_log.h>

#include <i2cdev.h>
#include <wiring.h>

#include "virtual_sensor.h"

#define ACTIVE_I2C      I2C_NUM_1
#define I2C_FREQUENCY   500000

#define BME68X_RW 4 //RW for "relative weight"
#define DS18B20_RW 3
#define SHT85_RW 4

#define VS_DATA_TASK_CALC_INTERVAL 4000

static const char* TAG = "virtual_sensor";

static float vs_temperature = NO_DATA_AVAILABLE;
static float vs_humidity = NO_DATA_AVAILABLE;

static uint8_t vs_temp_w = 0;
static uint8_t vs_hum_w = 0;

static SensorsData sensors_data = {NULL,NULL,NULL};
static VirtualSensorData vs_data = {{false, &vs_temperature},
                                    {false, &vs_humidity},
                                    {false, NULL},
                                    {false, NULL, NULL},
                                    {false, NULL},
                                    {false, NULL}};

void update_virtual_sensor_data(void) {

    esp_err_t status;
    if (sensors_data.ds18b20_temp == NULL && sensors_data.bme680_data == NULL && sensors_data.sht85_data == NULL) {
        status = ESP_FAIL;
    } else {
        status = ESP_OK;
    }
    ESP_ERROR_CHECK(status);

    while (1) {

        vs_temperature = 0;
        vs_humidity = 0;

        if (sensors_data.ds18b20_temp) {
            vs_temperature += *(sensors_data.ds18b20_temp);
        }

        if (sensors_data.bme680_data) {
            vs_temperature += sensors_data.bme680_data->temperature*BME68X_RW;
            vs_humidity += sensors_data.bme680_data->humidity*BME68X_RW;
        }

        if (sensors_data.sht85_data) {
            vs_temperature += sensors_data.sht85_data->temperature*SHT85_RW;
            vs_humidity += sensors_data.sht85_data->humidity*SHT85_RW;
        }

        vs_temperature /= vs_temp_w;
        vs_humidity /= vs_hum_w;

        ESP_LOGD(TAG, "New virtual sensor values %.2f °C, %.2f %%\n", vs_temperature, vs_humidity);
        vTaskDelay(pdMS_TO_TICKS(VS_DATA_TASK_CALC_INTERVAL));
    }
}

esp_err_t initialize_virtual_sensor(void) {
    esp_err_t ret;

    ret = initialize_ds18b20_sensor(GPIO_DS18B20);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DS18B20 sensor initialization error : %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "DS18B20 sensor initialized");
        sensors_data.ds18b20_temp = get_ds18b20_temp();
        vs_temp_w += DS18B20_RW;
        vs_data.temperature.data_available = true;
    }

    ESP_ERROR_CHECK(i2cdev_init());

    // Use same i2c_config for bme680 and sht85 to avoid driver reinstallation between each communication
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_SENSORS_SDA,
        .scl_io_num = GPIO_SENSORS_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_FREQUENCY,
    };

    ret = initialize_bme68x_sensor(ACTIVE_I2C, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BME680 sensor initialization error : %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "BME680 sensor initialized");
        sensors_data.bme680_data = get_bme68x_data();
        vs_temp_w += BME68X_RW;
        vs_hum_w += BME68X_RW;

        vs_data.pressure.value = &(sensors_data.bme680_data->pressure);
        vs_data.static_iaq.value = &(sensors_data.bme680_data->static_iaq);
        vs_data.static_iaq.accuracy = &(sensors_data.bme680_data->static_iaq_accuracy);
        vs_data.co2_equivalent.value = &(sensors_data.bme680_data->co2_equivalent);
        vs_data.breath_voc_equivalent.value = &(sensors_data.bme680_data->breath_voc_equivalent);

        vs_data.temperature.data_available = true;
        vs_data.humidity.data_available = true;
        vs_data.pressure.data_available = true;
        vs_data.static_iaq.data_available = true;
        vs_data.co2_equivalent.data_available = true;
        vs_data.breath_voc_equivalent.data_available = true;
    }

    ret = initialize_sht85_sensor(ACTIVE_I2C, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT85 sensor initialization error : %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SHT85 sensor initialized");
        sensors_data.sht85_data = get_sht85_data();
        vs_temp_w += SHT85_RW;
        vs_hum_w += SHT85_RW;

        vs_data.temperature.data_available = true;
        vs_data.humidity.data_available = true;
    }

    xTaskCreatePinnedToCore((TaskFunction_t)update_virtual_sensor_data, "Virtual Sensor data updater", configMINIMAL_STACK_SIZE * 3, NULL, 3, NULL, PRO_CPU_NUM);

    return ESP_OK;
}

SensorsData* get_sensors_data(void) {
    return &sensors_data;
}

VirtualSensorData* get_virtual_sensor_data(void) {
    return &vs_data;
}
