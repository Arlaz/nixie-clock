#include <driver/gpio.h>
#include <driver/spi_master.h>

#include "shift_registers.h"

#define USED_SPI_HOST HSPI_HOST
#define DMA_CHAN 1

static const char* TAG = "shift registers";

esp_err_t init_shift_registers(sr_handle_t* sr, gpio_num_t data, gpio_num_t clk, gpio_num_t latch) {
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .mosi_io_num = data,
        .miso_io_num = -1,
        .sclk_io_num = clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = (sr->cascade_size * sr->num_channels) / 8,
        .flags = SPICOMMON_BUSFLAG_MASTER |
                 SPICOMMON_BUSFLAG_GPIO_PINS |
                 SPICOMMON_BUSFLAG_MOSI |
                 SPICOMMON_BUSFLAG_SCLK,
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(USED_SPI_HOST, &buscfg, DMA_CHAN);
    if (ret != ESP_OK) {
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .mode = 1U,                            // SPI mode 1
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = sr->cascade_size,  // Time to propagate through the chained registers
        .clock_speed_hz = SPI_MASTER_FREQ_8M,  // Clock out at 8 MHz
        .spics_io_num = latch,                 // Use CS as LE line
        .flags = SPI_DEVICE_NO_DUMMY,          // Device only write data
        .pre_cb = NULL,                        // Specify pre-transfer callback
        .post_cb = NULL,                       // Specify post-transfer callback
    };

    // Attach the drivers to the SPI bus
    ret = spi_bus_add_device(USED_SPI_HOST, &devcfg, &sr->spi_reg);
    if (ret != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

esp_err_t send_data(const sr_handle_t* sr, const uint32_t* data) {
    spi_transaction_t trans = {
        .length = sr->cascade_size * sr->num_channels,
        .tx_buffer = data,
    };
    return spi_device_queue_trans(sr->spi_reg, &trans, pdMS_TO_TICKS(5));
}
