idf_component_register(

        SRCS
            "nixie-clock.c"

        REQUIRES
            wiring
            app_wifi
            leds_light
            nixie_display
            server_homekit
            virtual_sensor
)
