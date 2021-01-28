#pragma once

#include <bsec_integration.h>
#include <bsec_serialized_configurations_iaq.h>

typedef struct PARAMETERS_FOR_THE_BME680_TASK {
    sleep_fct sleep_function;
    get_timestamp_us_fct get_timestamp_us_function;
    output_ready_fct output_ready_function;
    state_save_fct state_save_function;
    uint32_t save_intvl;
} ParametersForBME680;

typedef struct BME680_CHARACTERISTICS {
    float temperature;
    float humidity;
    float pressure;
    float static_iaq;
    uint8_t iaq_accuracy;
    float co2_equivalent;
    float breath_voc_equivalent;
} bme680characteristics;

int initialize_bme680_sensor(void);
bme680characteristics* get_bme680_data(void);
