#include <stdio.h>
#include <string.h>
#include "main.h"

u8g2_t u8g2;

// Function to initialize I2C
static void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

//init screen and u8g2
static void u8g2_init(void) {    
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.clk   = PIN_CLK;
    u8g2_esp32_hal.mosi  = PIN_MOSI;
    u8g2_esp32_hal.cs    = PIN_CS;
    u8g2_esp32_hal.dc    = PIN_DC;
    u8g2_esp32_hal.reset = PIN_RESET;

    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_Setup_ssd1309_128x64_noname2_f(&u8g2, U8G2_R0, u8g2_esp32_spi_byte_cb, u8g2_esp32_gpio_and_delay_cb);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
}

void update_screen(void *message, const char *type) {
    char buffer[64]; // Buffer to hold the formatted string

    if (strcmp(type, "int") == 0) {
        snprintf(buffer, sizeof(buffer), "%u", *(unsigned int *)message); // Format as integer
    } else if (strcmp(type, "float") == 0) {
        snprintf(buffer, sizeof(buffer), "%.2f", *(float *)message); // Format as float
    } else if (strcmp(type, "string") == 0) {
        snprintf(buffer, sizeof(buffer), "%s", (char *)message); // Format as string
    } else {
        snprintf(buffer, sizeof(buffer), "Unknown Type");
    }

    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tr);
    u8g2_DrawStr(&u8g2, 0, 15, "Message:");
    u8g2_DrawStr(&u8g2, 0, 35, buffer); // Draw the formatted string
    u8g2_SendBuffer(&u8g2);
}

void app_main(void) {
    i2c_master_init();
    u8g2_init();
    esp_err_t status = WIFI_FAILURE;
    
    //initialize storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // ble_init();
    
    // connect to wireless AP
	status = connect_wifi();
	if (WIFI_SUCCESS != status)
	{
		ESP_LOGI(WIFI_TAG, "Failed to associate to AP, dying...");
        update_screen("WiFi Failed", "string");
		// return;
	}
    
	status = connect_tcp_server(update_screen);
	if (TCP_SUCCESS != status)
	{
		ESP_LOGI(WIFI_TAG, "Failed to connect to remote server, dying...");
        update_screen("TCP Failed", "string");
		// return;
	}

    while (1) {
        dht20_display(&u8g2);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 2 seconds before next read
    }
}