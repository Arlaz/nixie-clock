#pragma once

#include <driver/i2c.h>

typedef struct BME680_CHARACTERISTICS {
    float temperature;
    float humidity;
    float pressure;
    float static_iaq;
    uint8_t static_iaq_accuracy;
    float co2_equivalent;
    float breath_voc_equivalent;
} bme680characteristics;

esp_err_t initialize_bme680_sensor(i2c_port_t port, i2c_config_t* cfg);

bme680characteristics* get_bme680_data(void);
