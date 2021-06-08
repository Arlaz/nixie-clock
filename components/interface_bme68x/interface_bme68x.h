#pragma once

#include <driver/i2c.h>

typedef struct BME68X_CHARACTERISTICS {
    float temperature;
    float humidity;
    float pressure;
    float static_iaq;
    uint8_t static_iaq_accuracy;
    float co2_equivalent;
    float breath_voc_equivalent;
} bme68xcharacteristics;

esp_err_t initialize_bme68x_sensor(i2c_port_t port, i2c_config_t* cfg);

bme68xcharacteristics* get_bme68x_data(void);
