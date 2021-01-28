#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include <iot_i2c_bus.h>
#include "sht3x.h"

#define CONFIG_I2C_MASTER_SCL 19
#define CONFIG_I2C_MASTER_SDA 18

#define CONFIG_I2C_MASTER_FREQUENCY 100000

static i2c_bus_handle_t i2c_bus = NULL;
static sht3x_handle_t sht3x_sens = NULL;

/**
 * @brief i2c master initialization
 */
void i2c_bus_init() {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = CONFIG_I2C_MASTER_SDA;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = CONFIG_I2C_MASTER_SCL;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = CONFIG_I2C_MASTER_FREQUENCY;
	i2c_bus = iot_i2c_bus_create(I2C_NUM_1, &conf);
}

void sht85_loop(void) {

	i2c_bus_init();
	sht3x_sens = iot_sht3x_create(i2c_bus, 0x44);
	int32_t temperature, humidity;
	while (1)
	{
		int8_t ret = sht3x_measure_blocking_read(sht3x_sens, &temperature, &humidity);
		if (ret == STATUS_OK) {
			printf("measured temperature: %0.2f degreeCelsius, "
				   "measured humidity: %0.2f percentRH\n",
				   temperature / 1000.0f,
				   humidity / 1000.0f);
		}
		else {
			printf("error reading measurement\n");
		}
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}
