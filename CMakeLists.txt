idf_component_register(SRCS "src/blink.c"
                            "src/u8g2_esp32_hal.c"
                    INCLUDE_DIRS "." "include"
                    REQUIRES driver esp_driver_spi u8g2)
