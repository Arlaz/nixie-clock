#pragma once

int initialize_ds18b20_sensor(gpio_num_t gpio);

float* get_ds18b20_temp(void);
