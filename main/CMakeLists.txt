idf_component_register(SRCS "esp32-humidity.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_driver_spi u8g2 u8g2-hal-esp-idf)
