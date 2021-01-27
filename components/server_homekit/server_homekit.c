#include <string.h>

#include <esp_event.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <math.h>

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>
#include <hap_fw_upgrade.h>
#include <app_hap_setup_payload.h>

#include <virtual_sensor.h>
#include <leds_light.h>

#include "server_homekit.h"

static const char* TAG = "HomeKit_Server";

static VirtualSensorData* environmental_data;

/* Callback for handling writes on the Leds Service */
static int leds_write(hap_write_data_t* write_data, int count, void *serv_priv, void *write_priv) {
    int i, ret = HAP_SUCCESS;
    hap_write_data_t *write;
    for (i = 0; i < count; i++) {
        write = &write_data[i];
        /* Setting a default error value */
        *(write->status) = HAP_STATUS_VAL_INVALID;
        if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ON)) {

            ESP_LOGI(TAG, "Received Write for Leds %s", write->val.b ? "On" : "Off");
            if (leds_set_on(write->val.b) == 0) {
                *(write->status) = HAP_STATUS_SUCCESS;
            }
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_BRIGHTNESS)) {
            ESP_LOGI(TAG, "Received Write for Leds Brightness %d", write->val.i);
            if (leds_set_brightness(write->val.i) == 0) {
                *(write->status) = HAP_STATUS_SUCCESS;
            }
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_HUE)) {
            ESP_LOGI(TAG, "Received Write for Leds Hue %f", write->val.f);
            if (leds_set_hue(write->val.f) == 0) {
                *(write->status) = HAP_STATUS_SUCCESS;
            }
        } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_SATURATION)) {
            ESP_LOGI(TAG, "Received Write for Leds Saturation %f", write->val.f);
            if (leds_set_saturation(write->val.f) == 0) {
                *(write->status) = HAP_STATUS_SUCCESS;
            }
        } else {
            *(write->status) = HAP_STATUS_RES_ABSENT;
        }
        /* If the characteristic write was successful, update it in hap core */
        if (*(write->status) == HAP_STATUS_SUCCESS) {
            hap_char_update_val(write->hc, &(write->val));
        } else {
            /* Else, set the return value appropriately to report error */
            ret = HAP_FAIL;
        }
    }
    return ret;
}

static int temperature_sensor_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv) {
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_TEMPERATURE)) {
        if (environmental_data->temperature.data_available) {
            hap_val_t new_val;
            new_val.f = *environmental_data->temperature.value;
            hap_char_update_val(hc, &new_val);
            *status_code = HAP_STATUS_SUCCESS;
            return HAP_SUCCESS;
        } else {
            ESP_LOGW(TAG, "No temperature data available");
            *status_code = HAP_STATUS_RES_ABSENT;
        }
    } else {
        ESP_LOGW(TAG, "Not a temperature characteristic");
        *status_code = HAP_STATUS_RES_ABSENT;
    }
    return HAP_FAIL;
}

static int humidity_sensor_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv) {
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CURRENT_RELATIVE_HUMIDITY)) {
        if (environmental_data->humidity.data_available) {
            hap_val_t new_val;
            new_val.f = *environmental_data->humidity.value;
            hap_char_update_val(hc, &new_val);
            *status_code = HAP_STATUS_SUCCESS;
            return HAP_SUCCESS;
        } else {
            ESP_LOGW(TAG, "No humidity data available");
            *status_code = HAP_STATUS_RES_ABSENT;
        }
    } else {
        ESP_LOGW(TAG, "Not a relative humidity characteristic");
        *status_code = HAP_STATUS_RES_ABSENT;
    }
    return HAP_FAIL;
}

static int air_quality_sensor_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv) {
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_AIR_QUALITY)) {
        if (environmental_data->static_iaq.data_available) {
            hap_val_t new_val;
            if (*environmental_data->static_iaq.accuracy == 0) {
                new_val.u = 0;
            } else {
                new_val.u = round(0.5 + *environmental_data->static_iaq.value / 100.f);
            }
            hap_char_update_val(hc, &new_val);
            *status_code = HAP_STATUS_SUCCESS;
            return HAP_SUCCESS;
        } else {
            ESP_LOGW(TAG, "No air quality data available");
            *status_code = HAP_STATUS_RES_ABSENT;
        }
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_VOC_DENSITY)) {
        if (environmental_data->breath_voc_equivalent.data_available) {
            hap_val_t new_val;
            new_val.f = *environmental_data->breath_voc_equivalent.value;
            hap_char_update_val(hc, &new_val);
            *status_code = HAP_STATUS_SUCCESS;
            return HAP_SUCCESS;
        } else {
            ESP_LOGW(TAG, "No voc data available");
            *status_code = HAP_STATUS_RES_ABSENT;
        }
    } else {
            ESP_LOGW(TAG, "Not a air quality / voc characteristic");
            *status_code = HAP_STATUS_RES_ABSENT;
        }
    return HAP_FAIL;
}

static int carbon_dioxide_sensor_read(hap_char_t *hc, hap_status_t *status_code, void *serv_priv, void *read_priv) {
    if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CARBON_DIOXIDE_DETECTED)) {
        if (environmental_data->co2_equivalent.data_available) {
            hap_val_t new_val;
            new_val.u = *environmental_data->co2_equivalent.value > 1600;
            hap_char_update_val(hc, &new_val);
            *status_code = HAP_STATUS_SUCCESS;
            return HAP_SUCCESS;
        } else {
            ESP_LOGW(TAG, "No carbon detection data available");
            *status_code = HAP_STATUS_RES_ABSENT;
        }
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CARBON_DIOXIDE_LEVEL)) {
        if (environmental_data->co2_equivalent.data_available) {
            hap_val_t new_val;
            new_val.f = *environmental_data->co2_equivalent.value;
            hap_char_update_val(hc, &new_val);
            *status_code = HAP_STATUS_SUCCESS;
            return HAP_SUCCESS;
        } else {
            ESP_LOGW(TAG, "No carbon dioxide level data available");
            *status_code = HAP_STATUS_RES_ABSENT;
        }
    } else if (!strcmp(hap_char_get_type_uuid(hc), HAP_CHAR_UUID_CARBON_DIOXIDE_PEAK_LEVEL)) {
        if (environmental_data->co2_equivalent.data_available) {
            const hap_val_t *cur_val = hap_char_get_val(hc);
            hap_val_t new_val;
            new_val.f = cur_val->f > *environmental_data->co2_equivalent.value ? cur_val->f : *environmental_data->co2_equivalent.value;
            hap_char_update_val(hc, &new_val);
            *status_code = HAP_STATUS_SUCCESS;
            return HAP_SUCCESS;
        } else {
            ESP_LOGW(TAG, "No carbon dioxide peak level data available");
            *status_code = HAP_STATUS_RES_ABSENT;
        }
    } else {
        ESP_LOGW(TAG, "Not a carbon dioxide related characteristic");
        *status_code = HAP_STATUS_RES_ABSENT;
    }
    return HAP_FAIL;
}

static int nixie_identify(hap_acc_t* ha) {
    ESP_LOGI(TAG, "Accessory identified");
    return HAP_SUCCESS;
}

/*The main thread for handling the Accessory */
static void homekit_init(void *arg) {
    hap_acc_t* accessory;
    hap_serv_t* leds_service;
    hap_serv_t* temperature_service;
    hap_serv_t* humidity_service;
    hap_serv_t* air_quality_service;
    hap_serv_t* carbon_dioxide_detection_service;

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally */
    hap_acc_cfg_t cfg = {
        .name = "Nixie Clock",
        .manufacturer = "Arlaz",
        .model = "Cool Clock",
        .serial_num = "423971",
        .fw_rev = "0.9.0",
        .hw_rev = "1.0",
        .pv = "1.1.0",
        .identify_routine = nixie_identify,
        .cid = HAP_CID_OTHER,
    };

    /* Create accessory object */
    accessory = hap_acc_create(&cfg);
    if (!accessory) {
        ESP_LOGE(TAG, "Failed to create accessory");
        goto err;
    }

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* Create the Leds Service. Include the "name" since this is a user visible service  */
    leds_service = hap_serv_lightbulb_create(true);
    if (!leds_service) {
        ESP_LOGE(TAG, "Failed to create leds service");
        goto err;
    }

    /* Get environmental data */
    environmental_data = get_virtual_sensor_data();

    /* Create the Temperature Sensor Service. Include the "name" since this is a user visible service  */
    temperature_service = hap_serv_temperature_sensor_create(10);
    if (!temperature_service) {
        ESP_LOGE(TAG, "Failed to create temperature sensor service");
        goto err;
    }

    /* Create the Humidity Sensor Service. Include the "name" since this is a user visible service  */
    humidity_service = hap_serv_humidity_sensor_create(50);
    if (!humidity_service) {
        ESP_LOGE(TAG, "Failed to create humidity sensor service");
        goto err;
    }

    /* Create the Air Quality Sensor Service. Include the "name" since this is a user visible service  */
    air_quality_service = hap_serv_air_quality_sensor_create(3);
    if (!air_quality_service) {
        ESP_LOGE(TAG, "Failed to create air quality sensor service");
        goto err;
    }

    /* Create the Carbon Dioxide Sensor Service. Include the "name" since this is a user visible service  */
    carbon_dioxide_detection_service = hap_serv_carbon_dioxide_sensor_create(0);
    if (!carbon_dioxide_detection_service) {
        ESP_LOGE(TAG, "Failed to create carbon dioxide sensor service");
        goto err;
    }

    /* Add the optional characteristics to the Leds Service */
    int ret = hap_serv_add_char(leds_service, hap_char_name_create("Leds"));
    ret |= hap_serv_add_char(leds_service, hap_char_brightness_create(50));
    ret |= hap_serv_add_char(leds_service, hap_char_hue_create(180));
    ret |= hap_serv_add_char(leds_service, hap_char_saturation_create(100));
    if (ret != HAP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add optional characteristics to leds service");
        goto err;
    }

    /* Add the name characteristic to the Temperature Sensor Service */
    ret = hap_serv_add_char(temperature_service, hap_char_name_create("Temperature Sensor"));
    if (ret != HAP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add name characteristic to temperature sensor service");
        goto err;
    }

    /* Add the name optional characteristic to the Humidity Sensor Service */
    ret = hap_serv_add_char(humidity_service, hap_char_name_create("Humidity Sensor"));
    if (ret != HAP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add name characteristic to humidity sensor service");
        goto err;
    }

    /* Add the optional characteristics to the Air Quality Sensor Service */
    ret = hap_serv_add_char(air_quality_service, hap_char_name_create("Air Quality Sensor"));
    ret |= hap_serv_add_char(air_quality_service, hap_char_voc_density_create(500));
    if (ret != HAP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add optional characteristics to air quality sensor service");
        goto err;
    }

    /* Add the optional characteristics to the Carbon Dioxide Sensor Service */
    ret = hap_serv_add_char(carbon_dioxide_detection_service, hap_char_name_create("My Carbon Dioxide Sensor"));
    ret |= hap_serv_add_char(carbon_dioxide_detection_service, hap_char_carbon_dioxide_level_create(0));
    ret |= hap_serv_add_char(carbon_dioxide_detection_service, hap_char_carbon_dioxide_peak_level_create(0));
    if (ret != HAP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add optional characteristics to carbon dioxide sensor service");
        goto err;
    }

    /* Set the write callback for the leds service */
    hap_serv_set_write_cb(leds_service, leds_write);
    /* Set the read callback for the Temperature Sensor Service */
    hap_serv_set_read_cb(temperature_service, temperature_sensor_read);
    /* Set the read callback for the Humidity Sensor Service */
    hap_serv_set_read_cb(humidity_service, humidity_sensor_read);
    /* Set the read callback for the Air Quality Sensor Service */
    hap_serv_set_read_cb(air_quality_service, air_quality_sensor_read);
    /* Set the read callback for the Carbon Dioxide Sensor Service */
    hap_serv_set_read_cb(carbon_dioxide_detection_service, carbon_dioxide_sensor_read);

    /* Add the Leds Service to the Accessory Object */
    hap_acc_add_serv(accessory, leds_service);
    /* Add the Temperature Sensor Service to the Accessory Object */
    hap_acc_add_serv(accessory, temperature_service);
    /* Add the Humidity Sensor Service to the Accessory Object */
    hap_acc_add_serv(accessory, humidity_service);
    /* Add the Air Quality Sensor Service to the Accessory Object */
    hap_acc_add_serv(accessory, air_quality_service);
    /* Add the Carbon Dioxide Sensor Service to the Accessory Object */
    hap_acc_add_serv(accessory, carbon_dioxide_detection_service);


    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);
#ifdef CONFIG_EXAMPLE_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_EXAMPLE_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */
    hap_enable_mfi_auth(HAP_MFI_AUTH_HW);

    /* After all the initializations are done, start the HAP core */
    hap_start();

    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);

err:
    hap_acc_delete(accessory);
    vTaskDelete(NULL);
}

void homekit_server_start(void) {
    xTaskCreatePinnedToCore(homekit_init, "HomeKit task", configMINIMAL_STACK_SIZE*8, NULL, 3, NULL, PRO_CPU_NUM);
}
