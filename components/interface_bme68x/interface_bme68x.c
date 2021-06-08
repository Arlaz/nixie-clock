#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs.h>
#include <driver/i2c.h>

#include <i2cdev.h>

#include <bsec_integration.h>
#include <bsec_serialized_configurations_selectivity.h>
#include <nvs_flash.h>

#include "interface_bme68x.h"

#define STATE_SAVING_SAMPLES_INTERVAL 10000

static const char* TAG = "bme68x_sensor";
static const char* sensor_binary = "bme68x_sensor_blob";

typedef struct PARAMETERS_FOR_THE_BME68X_TASK {
    bme68x_delay_us_fptr_t sleep_function;
    get_timestamp_us_fct get_timestamp_us_function;
    output_ready_fct output_ready_function;
    state_save_fct state_save_function;
    uint32_t save_intvl;
} ParametersForBME68x;

static i2c_dev_t i2c_bme68x;

static bme68xcharacteristics bme68x_current_data =
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
int8_t bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    // ...
    // Please insert system specific function to write to the bus where BME68x is connected
    // ...
    return i2c_dev_write_reg(&i2c_bme68x, reg_addr, reg_data, (size_t)length);

    /** Following is an implementation without i2c_dev

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
     **/
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
int8_t bus_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t length, void* intf_ptr) {
    // ...
    // Please insert system specific function to read from bus where BME68X is connected
    // ...
    return i2c_dev_read_reg(&i2c_bme68x, reg_addr, reg_data, (size_t)length);
}

/*!
 * @brief           System specific implementation of sleep function
 *
 * @param[in]       period_us    time in microseconds
 *
 * @return          none
 */
static void bme68x_sleep(uint32_t period_us, void* intf_ptr) {
    // ...
    // Please insert system specific function sleep or delay for t_us microseconds
    // ...
    vTaskDelay(pdMS_TO_TICKS((uint32_t) period_us / 1000.0f));
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
void output_ready(int64_t timestamp,
                  float iaq,
                  uint8_t iaq_accuracy,
                  float temp,
                  float raw_temp,
                  float raw_pressure,
                  float humidity,
                  float raw_humidity,
                  float raw_gas,
                  float static_iaq,
                  uint8_t static_iaq_accuracy,
                  float co2_equivalent,
                  uint8_t co2_accuracy,
                  float breath_voc_equivalent,
                  uint8_t breath_voc_accuracy,
                  float comp_gas_value,
                  uint8_t comp_gas_accuracy,
                  float gas_percentage,
                  uint8_t gas_percentage_acccuracy,
                  bsec_library_return_t bsec_status)
{
    // ...
    // Please insert system specific code to further process or display the BSEC outputs
    // ...
    bme68x_current_data.temperature = temp;
    bme68x_current_data.humidity = humidity;
    bme68x_current_data.pressure = raw_pressure;
    bme68x_current_data.static_iaq = static_iaq;
    bme68x_current_data.static_iaq_accuracy = iaq_accuracy;
    bme68x_current_data.co2_equivalent = co2_equivalent;
    bme68x_current_data.breath_voc_equivalent = breath_voc_equivalent;

    ESP_LOGI(TAG, "timestamp : %lld", timestamp);
    ESP_LOGI(TAG, "iaq : %f", iaq);
    ESP_LOGI(TAG, "iaq accuracy : %hhu", iaq_accuracy);
    ESP_LOGI(TAG, "temperature : %f", temp);
    ESP_LOGI(TAG, "raw temperature : %f", raw_temp);
    ESP_LOGI(TAG, "pressure %f", raw_pressure);
    ESP_LOGI(TAG, "humidity : %f", humidity);
    ESP_LOGI(TAG, "raw humidity : %f", raw_humidity);
    ESP_LOGI(TAG, "raw gas : %f", raw_gas);
    ESP_LOGI(TAG, "static iaq : %f", static_iaq);
    ESP_LOGI(TAG, "static iaq accuracy : %hhu", static_iaq_accuracy);
    ESP_LOGI(TAG, "co2_equivalent : %f", co2_equivalent);
    ESP_LOGI(TAG, "co2_equivalent accuracy : %hhu", co2_accuracy);
    ESP_LOGI(TAG, "breath voc equivalent : %f", breath_voc_equivalent);
    ESP_LOGI(TAG, "breath voc equivalent accuracy : %hhu", breath_voc_accuracy);
    ESP_LOGI(TAG, "compensated gas value : %f", comp_gas_value);
    ESP_LOGI(TAG, "compensated gas accuracy : %hhu", comp_gas_accuracy);
    ESP_LOGI(TAG, "gas percentage : %f", gas_percentage);
    ESP_LOGI(TAG, "gas percentage accuracy %hhu", gas_percentage_acccuracy);
    ESP_LOGI(TAG, "status %d", bsec_status);

}

/*!
 *
 * @return          pointer to the struct holding current bme68x sensor data
 */
bme68xcharacteristics* get_bme68x_data(void) {
    return &bme68x_current_data;
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
    esp_err_t err_nvs = nvs_open_from_partition(CONFIG_BSEC_DEF_NVS_RUNTIME_PARTITION, "bsec_state", NVS_READONLY, &my_handle);
    if (err_nvs == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "opening partition failed with code %d (no state available ?)", err_nvs);
        return ESP_OK;
    } else if (err_nvs != ESP_OK) {
        ESP_LOGW(TAG, "opening partition failed with code %d", err_nvs);
        return 0;
    }

    err_nvs = nvs_get_blob(my_handle, sensor_binary, state_buffer, &n_buffer);
    // We close this anyway even if the operation didn't succeed.
    nvs_close(my_handle);
    if (err_nvs == ESP_OK){
        return n_buffer;
    }
    ESP_LOGW(TAG, "loading sensor binary blob failed with code %d", err_nvs);
    return ESP_OK;
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
    esp_err_t err_nvs = nvs_open_from_partition(CONFIG_BSEC_DEF_NVS_RUNTIME_PARTITION, "bsec_state", NVS_READWRITE, &my_handle);
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
    ESP_LOGI(TAG, "Loading configuration: buffer-size %d config size %zu", n_buffer, sizeof(bsec_config_selectivity));
    assert(n_buffer >= sizeof(bsec_config_selectivity));
    memcpy(config_buffer, bsec_config_selectivity, sizeof(bsec_config_selectivity));

    return sizeof(bsec_config_selectivity);
}

/**
 * Have to be used if i2c bus is used without i2cdev
 *
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_SENSORS_SDA,
        .scl_io_num = GPIO_SENSORS_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_FREQUENCY,
    };
    i2c_param_config(ACTIVE_I2C, &conf);
    return i2c_driver_install(ACTIVE_I2C, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0)
}
*/

/*!
 * @brief           Pass the struct of the loop in arguments
 *
 * @param[in]       parameters    pointer to struct holding the parameters
 *
 * @return          none
 */
void bme68x_loop(ParametersForBME68x* parameters) {
    bsec_iot_loop(parameters->sleep_function, parameters->get_timestamp_us_function,
                  parameters->output_ready_function, parameters->state_save_function, parameters->save_intvl);
}

/*!
 * @brief           Main function which configures BSEC library and then reads and processes the data from sensor based
 *                  on timer ticks
 *
 * @return          result of the processing
 */
esp_err_t initialize_bme68x_sensor(i2c_port_t port, i2c_config_t* cfg)  {
    ESP_LOGI(TAG, "Initialization started");

    i2c_bme68x.port = port;
    // addr have to match the address defined in bsec_iot_init func
    // redefinition is required for i2cdev component
    i2c_bme68x.addr = BME68X_I2C_ADDR_LOW;
    i2c_bme68x.cfg = *cfg;
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_dev_create_mutex(&i2c_bme68x));

    // Init BSEC partition
    esp_err_t err;
    err = nvs_flash_init_partition(CONFIG_BSEC_DEF_NVS_RUNTIME_PARTITION);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase_partition(CONFIG_BSEC_DEF_NVS_RUNTIME_PARTITION));
        err = nvs_flash_init_partition(CONFIG_BSEC_DEF_NVS_RUNTIME_PARTITION);
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    /* Call to the function which initializes the BSEC library
     * Switch on low-power mode and provide no temperature offset */
    return_values_init init_result = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, bme68x_sleep, state_load, config_load);

    if (init_result.bme68x_status != BME68X_OK) {
        /* Could not initialize BME68X */
        ESP_LOGE(TAG, "initializing BME68X failed %d", init_result.bme68x_status);
        ESP_ERROR_CHECK(i2c_dev_delete_mutex(&i2c_bme68x));
        return ESP_ERR_NOT_FOUND;
    }

    if (init_result.bsec_status != BSEC_OK) {
        /* Could not initialize BSEC library */
        ESP_LOGE(TAG, "initializing BSEC failed %d", init_result.bsec_status);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Entering into the loop");

    // Structure to initialize BME68x task because xTaskCreate can only pass one argument to the task function
    ParametersForBME68x arguments = {bme68x_sleep, esp_timer_get_time, output_ready, state_save, STATE_SAVING_SAMPLES_INTERVAL};

    /* Call to endless loop function which reads and processes data based on sensor settings */
    /* State is saved every STATE_SAVING_SAMPLES_INTERVAL samples, by default every 10.000 * 3 secs = 500 minutes  */
    xTaskCreatePinnedToCore((TaskFunction_t)bme68x_loop, "Update BME68X", configMINIMAL_STACK_SIZE * 10, &arguments, 2, NULL, PRO_CPU_NUM);

    return ESP_OK;
}
