idf_component_register(SRCS "app_wifi.c"
                    INCLUDE_DIRS "."
                    REQUIRES wifi_provisioning qrcode nvs_flash)
