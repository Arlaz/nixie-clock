#include "sht3x.h"
#include "sensirion_common.h"
#include "driver/i2c.h"
#include "iot_i2c_bus.h"

/* all measurement commands return T (CRC) RH (CRC) */
#if USE_SENSIRION_CLOCK_STRETCHING
#define SHT3X_CMD_MEASURE_HPM 0x2C06
#define SHT3X_CMD_MEASURE_LPM 0x2C10
#else /* USE_SENSIRION_CLOCK_STRETCHING */
#define SHT3X_CMD_MEASURE_HPM 0x2400
#define SHT3X_CMD_MEASURE_LPM 0x2416
#endif /* USE_SENSIRION_CLOCK_STRETCHING */
static const uint16_t SHT3X_CMD_READ_STATUS_REG = 0xF32D;
static const uint16_t SHT3X_CMD_READ_SERIAL_ID = 0x3780;
static const uint16_t SHT3X_CMD_DURATION_USEC = 1000;
#ifdef SHT_ADDRESS
static const uint8_t SHT3X_ADDRESS = SHT_ADDRESS;
#else
static const uint8_t SHT3X_ADDRESS = 0x44;
#endif

typedef struct {
    i2c_bus_handle_t bus;
    uint16_t dev_addr;
} sht3x_dev_t;

static uint16_t sht3x_cmd_measure = SHT3X_CMD_MEASURE_HPM;

sht3x_handle_t iot_sht3x_create(i2c_bus_handle_t bus, uint16_t dev_addr) {
    sht3x_dev_t *sensor = (sht3x_dev_t *)calloc(1, sizeof(sht3x_dev_t));
    sensor->bus = bus;
    sensor->dev_addr = dev_addr;
    return (sht3x_handle_t)sensor;
}


esp_err_t iot_sht3x_delete(i2c_bus_handle_t sensor, bool del_bus) {
    sht3x_dev_t* sens = (sht3x_dev_t*) sensor;
    if(del_bus) {
        iot_i2c_bus_delete(sens->bus);
        sens->bus = NULL;
    }
    free(sens);
    return ESP_OK;
}


int16_t sht3x_measure_blocking_read(sht3x_handle_t sensor,int32_t *temperature, int32_t *humidity) {
    int16_t ret = sht3x_measure(sensor);
    if (ret == STATUS_OK) {
#if !defined(USE_SENSIRION_CLOCK_STRETCHING) || !USE_SENSIRION_CLOCK_STRETCHING
        sensirion_sleep_usec(SHT3X_MEASUREMENT_DURATION_USEC);
#endif /* USE_SENSIRION_CLOCK_STRETCHING */
        ret = sht3x_read(sensor, temperature, humidity);
    }
    return ret;
}

int16_t sht3x_measure(sht3x_handle_t sensor) {
    sht3x_dev_t *sens = (sht3x_dev_t *)sensor;
    return sensirion_i2c_write_cmd(sens->bus, sens->dev_addr, sht3x_cmd_measure);
}

int16_t sht3x_read(sht3x_handle_t sensor, int32_t *temperature, int32_t *humidity) {
    uint16_t words[2];
    sht3x_dev_t* sens = (sht3x_dev_t*) sensor;
    int16_t ret = sensirion_i2c_read_words(sens->bus,sens->dev_addr, words,
                                           SENSIRION_NUM_WORDS(words));
    /**
     * formulas for conversion of the sensor signals, optimized for fixed point
     * algebra: Temperature = 175 * S_T / 2^16 - 45
     * Relative Humidity = * 100 * S_RH / 2^16
     */
    *temperature = ((21875 * (int32_t)words[0]) >> 13) - 45000;
    *humidity = ((12500 * (int32_t)words[1]) >> 13);

    return ret;
}

int16_t sht3x_probe(sht3x_handle_t sensor) {
    uint16_t status;
    sht3x_dev_t* sens = (sht3x_dev_t*) sensor;
    return sensirion_i2c_delayed_read_cmd(sens->bus,sens->dev_addr,
                                          SHT3X_CMD_READ_STATUS_REG,
                                          SHT3X_CMD_DURATION_USEC, &status, 1);
}

void sht3x_enable_low_power_mode(uint8_t enable_low_power_mode) {
    sht3x_cmd_measure =
        enable_low_power_mode ? SHT3X_CMD_MEASURE_LPM : SHT3X_CMD_MEASURE_HPM;
}

int16_t sht3x_read_serial(sht3x_handle_t sensor, uint32_t* serial) {
    int16_t ret;
    sht3x_dev_t* sens = (sht3x_dev_t*) sensor;
    union {
        uint16_t words[SENSIRION_NUM_WORDS(*serial)];
        uint32_t u32_value;
    } buffer;

    ret = sensirion_i2c_delayed_read_cmd(sens->bus, sens->dev_addr, SHT3X_CMD_READ_SERIAL_ID, SHT3X_CMD_DURATION_USEC,
                                         buffer.words, SENSIRION_NUM_WORDS(buffer.words));
    SENSIRION_WORDS_TO_BYTES(buffer.words, SENSIRION_NUM_WORDS(buffer.words));
    *serial = be32_to_cpu(buffer.u32_value);
    return ret;
}

const char* sht3x_get_driver_version(void) {
    return "SHT_DRV_VERSION_STR";
}

uint8_t sht3x_get_configured_address(void) {
    return SHT3X_ADDRESS;
}
