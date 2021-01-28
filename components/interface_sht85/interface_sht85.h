#pragma once

typedef struct SHT85_CHARACTERISTICS {
    float temperature;
    float humidity;
} sht85characteristics;

esp_err_t initialize_sht85_sensor(i2c_port_t port, i2c_config_t* cfg);

sht85characteristics* get_sht85_data(void);
