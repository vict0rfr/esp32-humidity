idf_component_register(SRCS "main.c" "wifi_driver.c"
                    INCLUDE_DIRS "."
                   REQUIRES driver
                             esp_driver_spi
                             u8g2 
                             u8g2-hal-esp-idf 
                             esp_wifi
                             nvs_flash
                             )