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
    gpio_reset_pin(PIN_RESET);
    gpio_set_direction(PIN_RESET, GPIO_MODE_OUTPUT);
    
    // Perform manual reset sequence
    gpio_set_level(PIN_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(20)); // Keep low for 20ms
    gpio_set_level(PIN_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait 100ms after reset before init
    // --- End Manual Reset ---
    
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

void update_screen(const char *message) {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tr);
    u8g2_DrawStr(&u8g2, 0, 15, "Message:");
    u8g2_DrawStr(&u8g2, 0, 35, message);
    u8g2_SendBuffer(&u8g2);
}

// Function to read temperature and humidity from DHT20
esp_err_t dht20_read(float *temperature, float *humidity) {
    uint8_t data[7];
    uint8_t cmd = 0xAC; // Command to trigger measurement

    // Send measurement command
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (DHT20_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, cmd, true);
    i2c_master_write_byte(cmd_handle, 0x33, true);
    i2c_master_write_byte(cmd_handle, 0x00, true);
    i2c_master_stop(cmd_handle);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(DHT20_TAG, "Failed to send measurement command");
        return ret;
    }

    // Wait for measurement to complete
    vTaskDelay(pdMS_TO_TICKS(80));

    // Read data
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (DHT20_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd_handle, data, sizeof(data) - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd_handle, data + 6, I2C_MASTER_NACK);
    i2c_master_stop(cmd_handle);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(DHT20_TAG, "Failed to read data");
        return ret;
    }

    // Parse temperature and humidity
    uint32_t raw_humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4;
    uint32_t raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
    *humidity = (raw_humidity * 100.0) / 1048576.0;
    *temperature = (raw_temperature * 200.0) / 1048576.0 - 50.0;

    return ESP_OK;
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

    // connect to wireless AP
	status = connect_wifi();
	if (WIFI_SUCCESS != status)
	{
		ESP_LOGI(WIFI_TAG, "Failed to associate to AP, dying...");
        update_screen("WiFi Failed");
		return;
	}
    
	esp_wifi_set_ps(WIFI_PS_NONE);
	status = connect_tcp_server(update_screen);
	if (TCP_SUCCESS != status)
	{
		ESP_LOGI(WIFI_TAG, "Failed to connect to remote server, dying...");
        update_screen("TCP Failed");
		return;
	}

    // float temperature, humidity;

    // while (1) {
    //     if (dht20_read(&temperature, &humidity) == ESP_OK) {
    //         u8g2_ClearBuffer(&u8g2);
    //         char temp_str[16], hum_str[16];
    //         snprintf(temp_str, sizeof(temp_str), "Temp: %.2fC", temperature);
    //         snprintf(hum_str, sizeof(hum_str), "Hum: %.2f%%", humidity);
    //         u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tr);
    //         u8g2_DrawStr(&u8g2, 0, 15, temp_str);
    //         u8g2_DrawStr(&u8g2, 0, 35, hum_str);
    //         u8g2_SendBuffer(&u8g2);
    //     } else {
    //         ESP_LOGE(DHT20_TAG, "Failed to read from DHT20");
    //     }

    //     vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 2 seconds before next read
    // }
}