idf_component_register(
        SRCS
            "server_homekit.c"
        INCLUDE_DIRS
            .
        PRIV_REQUIRES
            esp_hap_core
            esp_hap_apple_profiles
            esp_hap_extras
            app_hap_setup_payload
            nvs_flash
            virtual_sensor
            leds_light
)
