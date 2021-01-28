#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs.h>
#include <driver/i2c.h>

#include "wiring.h"
#include "interface_bme680.h"

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_FREQUENCY   100000
#define ACTIVE_I2C      I2C_NUM_1
#define STATE_SAVING_SAMPLES_INTERVAL 10000

static const char* TAG = "bme680_sensor";
static const char* sensor_binary = "sensor_blob";

static bme680characteristics bme680_current_data =
    {-273.0f, 0.0f, 0.0f,
     0.0f, 0, 0.0f, 0.0f};

/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

/*!
 * @brief           Write operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[in]        reg_data_ptr    pointer to the data to be written
 * param[in]        data_len        number of bytes to be written
 *
 * @return          result of the bus communication function
 */
int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len) {
    // ...
    // Please insert system specific function to write to the bus where BME680 is connected
    // ...
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    assert(data_len > 0 && reg_data_ptr != NULL); // Safeguarding the assumptions
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data_ptr, data_len, true);
    i2c_master_stop(cmd);
    esp_err_t err_write = i2c_master_cmd_begin(ACTIVE_I2C, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    // ESP_OK matches with the function success code (0)
    return (int8_t)err_write;
}

/*!
 * @brief           Read operation in either I2C or SPI
 *
 * param[in]        dev_addr        I2C or SPI device address
 * param[in]        reg_addr        register address
 * param[out]       reg_data_ptr    pointer to the memory to be used to store the read data
 * param[in]        data_len        number of bytes to be read
 *
 * @return          result of the bus communication function
 */
int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len) {
    // ...
    // Please insert system specific function to read from bus where BME680 is connected
    // ...
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    assert(data_len > 0 && reg_data_ptr != NULL); // Safeguarding the assumptions
    // Feeding the command in
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    //bme680_sleep(150);
    // Reading data back
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (data_len > 1) {
        i2c_master_read(cmd, reg_data_ptr, data_len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data_ptr + data_len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ACTIVE_I2C, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    // ESP_OK matches with the function success code (0)
    return (int8_t)ret;
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       t_ms    time in milliseconds
 *
 * @return          none
 */
static void bme680_sleep(uint32_t t_ms) {
    // ...
    // Please insert system specific function sleep or delay for t_ms milliseconds
    // ...
    vTaskDelay(pdMS_TO_TICKS(t_ms));
}

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp       time in nanoseconds
 * @param[in]       iaq             IAQ signal
 * @param[in]       iaq_accuracy    accuracy of IAQ signal
 * @param[in]       temperature     temperature signal
 * @param[in]       humidity        humidity signal
 * @param[in]       pressure        pressure signal
 * @param[in]       raw_temperature raw temperature signal
 * @param[in]       raw_humidity    raw humidity signal
 * @param[in]       gas             raw gas sensor signal
 * @param[in]       bsec_status     value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
                  float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
                  float static_iaq, float co2_equivalent, float breath_voc_equivalent)
{
    // ...
    // Please insert system specific code to further process or display the BSEC outputs
    // ...
    bme680_current_data.temperature = temperature;
    bme680_current_data.humidity = humidity;
    bme680_current_data.pressure = pressure;
    bme680_current_data.static_iaq = static_iaq;
    bme680_current_data.iaq_accuracy = iaq_accuracy;
    bme680_current_data.co2_equivalent = co2_equivalent;
    bme680_current_data.breath_voc_equivalent = breath_voc_equivalent;

    ESP_LOGI(TAG, "iaq : %f | static iaq : %f | iaq accuracy : %hhu "
                  "temperature : %f | humidity : %f | pressure %f "
                  "co2_equivalent %f | breath voc %f",
             iaq, static_iaq, iaq_accuracy, temperature, humidity, pressure, co2_equivalent, breath_voc_equivalent);
}

/*!
 *
 * @return          pointer to the struct holding current bme680 sensor data
 */
bme680characteristics* get_bme680_data(void) {
    return &bme680_current_data;
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer) {
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available,
    // otherwise return length of loaded state string.
    // ...
    nvs_handle_t my_handle;
    esp_err_t err_nvs = nvs_open("state", NVS_READONLY, &my_handle);
    ESP_ERROR_CHECK(err_nvs);

    err_nvs = nvs_get_blob(my_handle, sensor_binary, state_buffer, &n_buffer);
    // We close this anyway even if the operation didn't succeed.
    nvs_close(my_handle);
    if (err_nvs == ESP_OK){
        return n_buffer;
    }
    ESP_LOGW(TAG, "loading sensor binary blob failed with code %d", err_nvs);
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length) {
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
    nvs_handle_t my_handle;
    esp_err_t err_nvs = nvs_open("state", NVS_READWRITE, &my_handle);
    ESP_ERROR_CHECK(err_nvs);

    err_nvs = nvs_set_blob(my_handle, sensor_binary, state_buffer, length);
    ESP_ERROR_CHECK(err_nvs);
    err_nvs = nvs_commit(my_handle);
    ESP_ERROR_CHECK(err_nvs);
    nvs_close(my_handle);
}

/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer) {
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available,
    // otherwise return length of loaded config string.
    // ...
    ESP_LOGI(TAG, "Loading configuration: buffer-size %d config size %zu", n_buffer, sizeof(bsec_config_iaq));
    assert(n_buffer >= sizeof(bsec_config_iaq));
    memcpy(config_buffer, bsec_config_iaq, sizeof(bsec_config_iaq));

    return sizeof(bsec_config_iaq);
}

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_SENSORS_SDA,
        .scl_io_num = GPIO_SENSORS_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_FREQUENCY,
    };
    esp_err_t err_i2c = i2c_param_config(ACTIVE_I2C, &conf);
    ESP_ERROR_CHECK(err_i2c);
    return i2c_driver_install(ACTIVE_I2C, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/*!
 * @brief           Pass the struct of the loop in arguments
 *
 * @param[in]       parameters    pointer to struct holding the parameters
 *
 * @return          none
 */
void bme680_loop(ParametersForBME680* parameters) {
    bsec_iot_loop(parameters->sleep_function, parameters->get_timestamp_us_function,
                  parameters->output_ready_function, parameters->state_save_function, parameters->save_intvl);
}

/*!
 * @brief           Main function which configures BSEC library and then reads and processes the data from sensor based
 *                  on timer ticks
 *
 * @return          result of the processing
 */
int initialize_bme680_sensor(void)  {
    return_values_init ret;
    ESP_ERROR_CHECK(i2c_master_init());

    ESP_LOGI(TAG, "I2C initialized");
    /* Call to the function which initializes the BSEC library
     * Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_CONTINUOUS, 0.0f, bus_write, bus_read, bme680_sleep, state_load, config_load);
    if (ret.bme680_status) {
        /* Could not initialize BME680 */
        ESP_LOGE(TAG, "initializing BME680 failed %d", ret.bme680_status);
        return (int)ret.bme680_status;
    }
    else if (ret.bsec_status) {
        /* Could not initialize BSEC library */
        ESP_LOGE(TAG, "initializing BSEC failed %d", ret.bsec_status);
        return (int)ret.bsec_status;
    }

    ESP_LOGI(TAG, "Entering into the loop");

    // Structure to initialize BME680 task because xTaskCreate can only pass one argument to the task function
    ParametersForBME680 arguments = {bme680_sleep, esp_timer_get_time, output_ready, state_save, STATE_SAVING_SAMPLES_INTERVAL};

    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every STATE_SAVING_SAMPLES_INTERVAL samples, by default every 10.000 * 3 secs = 500 minutes  */
    xTaskCreatePinnedToCore((TaskFunction_t)bme680_loop, "Update BME680 characteristics", 256, &arguments, 2, NULL, 0);

    return 0;
}