#include <esp_log.h>
#include <soc/efuse_periph.h>

#include "wiring.h"

static const char* TAG = "wiring";

void pin_config(void) {
    uint32_t chip_ver = REG_GET_FIELD(EFUSE_BLK0_RDATA3_REG, EFUSE_RD_CHIP_VER_PKG);
    uint32_t pkg_ver = chip_ver & 0x7;
    if ((pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32D0WDQ6) || (pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32D0WDQ5)) {
        ESP_LOGI(TAG, "This chip is ESP32-D0WD");
        GPIO_SENSORS_SDA = GPIO_NUM_16;
        GPIO_RTC_SDA = GPIO_NUM_23;
        GPIO_HVDRIVER_DATA = GPIO_NUM_18;
        GPIO_HVDRIVER_CLK = GPIO_NUM_17;
    } else {
        GPIO_SENSORS_SDA = GPIO_NUM_9;
        GPIO_HVDRIVER_CLK = GPIO_NUM_10;
        if ((pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32PICOD2) || (pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32PICOD4)) {
            ESP_LOGI(TAG, "This chip is ESP32-PICO");
            GPIO_RTC_SDA = GPIO_NUM_23;
            GPIO_HVDRIVER_DATA = GPIO_NUM_18;
        } else if (pkg_ver == EFUSE_RD_CHIP_VER_PKG_ESP32PICOV302) {
            ESP_LOGI(TAG, "This chip is ESP32-PICO-V3-02");
            GPIO_RTC_SDA = GPIO_NUM_8;
            GPIO_HVDRIVER_DATA = GPIO_NUM_7;
        }
    }
}
