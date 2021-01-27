#pragma once

typedef struct SHT85_CHARACTERISTICS {
    float temperature;
    float humidity;
} sht85characteristics;

esp_err_t initialize_sht85_sensor(void);
sht85characteristics* get_sht85_data(void);
