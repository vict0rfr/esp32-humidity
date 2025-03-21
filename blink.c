#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#define I2C_PORT I2C_NUM_0
#define SDA_PIN 6
#define SCL_PIN 7
#define DHT20_ADDR 0x38
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 68
#define OLED_DC 10
#define OLED_RESET 11
#define OLED_ADDR 0x3C

#define SSD1309_CMD_DISPLAY_OFF        0xAE
#define SSD1309_CMD_DISPLAY_ON         0xAF
#define SSD1309_CMD_SET_DISPLAY_CLOCK  0xD5
#define SSD1309_CMD_SET_MUX_RATIO      0xA8
#define SSD1309_CMD_SET_OFFSET         0xD3
#define SSD1309_CMD_SET_START_LINE     0x40
#define SSD1309_CMD_SET_CONTRAST       0x81
#define SSD1309_CMD_DISPLAY_NORMAL     0xA6
#define SSD1309_CMD_SET_SEGMENT_REMAP  0xA1
#define SSD1309_CMD_SET_COM_SCAN_DIR   0xC8



static const char *TAG = "DHT20";
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

void i2c_init() {
    i2c_master_bus_config_t i2c_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .glitch_ignore_cnt = 7,  // Optional noise filtering
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_conf, &bus_handle));

    i2c_device_config_t dht20_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DHT20_ADDR,
        .scl_speed_hz = 100000
    };
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dht20_cfg, &dev_handle));

    i2c_device_config_t ssd1309_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_ADDR,
        .scl_speed_hz = 100000
    };
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &ssd1309_cfg, &dev_handle));
}

esp_err_t ssd1309_send_command(uint8_t command) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDR << 1), true);  // Send I2C address
    i2c_master_write_byte(cmd, 0x00, true);  // Command mode (0x00 means command)
    i2c_master_write_byte(cmd, command, true);  // Send the command
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t ssd1309_send_data(uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDR << 1), true);  // Send I2C address
    i2c_master_write_byte(cmd, 0x40, true);  // Data mode (0x40 means data)
    i2c_master_write_byte(cmd, data, true);  // Send the data
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void ssd1309_init() {
    ESP_LOGI("SSD1309", "Initializing OLED display");

    // Reset the display (optional, you can use GPIO to reset if needed)
    gpio_set_level(OLED_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for 100ms
    gpio_set_level(OLED_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for 100ms

    // Send initialization commands to SSD1309
    ssd1309_send_command(SSD1309_CMD_DISPLAY_OFF);  // Display OFF
    ssd1309_send_command(SSD1309_CMD_SET_DISPLAY_CLOCK);
    ssd1309_send_command(0x80);  // Set display clock (default value)
    ssd1309_send_command(SSD1309_CMD_SET_MUX_RATIO);
    ssd1309_send_command(0x3F);  // Set mux ratio (64 lines)
    ssd1309_send_command(SSD1309_CMD_SET_OFFSET);
    ssd1309_send_command(0x00);  // No offset
    ssd1309_send_command(SSD1309_CMD_SET_START_LINE);  // Set start line
    ssd1309_send_command(SSD1309_CMD_SET_SEGMENT_REMAP);  // Set segment remap
    ssd1309_send_command(SSD1309_CMD_SET_COM_SCAN_DIR);  // Set COM scan direction
    ssd1309_send_command(SSD1309_CMD_SET_CONTRAST);
    ssd1309_send_command(0x7F);  // Contrast setting (default)
    ssd1309_send_command(SSD1309_CMD_DISPLAY_NORMAL);  // Set normal display mode
    ssd1309_send_command(SSD1309_CMD_DISPLAY_ON);  // Turn on the display
}

void ssd1309_clear_screen() {
    // Set column address (from 0 to 127)
    ssd1309_send_command(0x21);  // Set column address
    ssd1309_send_command(0x00);  // Start column
    ssd1309_send_command(0x7F);  // End column

    // Set page address (from 0 to 7)
    ssd1309_send_command(0x22);  // Set page address
    ssd1309_send_command(0x00);  // Start page
    ssd1309_send_command(0x07);  // End page

    // Send 0x00 for each pixel (clear screen)
    for (int i = 0; i < (SCREEN_WIDTH * SCREEN_HEIGHT / 8); i++) {
        ssd1309_send_data(0x00);  // Send 0 (clear)
    }
}

esp_err_t read_dht20(float *temperature, float *humidity) {
    esp_err_t err;
    uint8_t command[] = {0xAC, 0x33, 0x00};  // Start measurement
    err = i2c_master_transmit(dev_handle, command, sizeof(command), 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command to DHT20");
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(80));  // Wait for measurement to complete

    uint8_t data[6] = {0};
    err = i2c_master_receive(dev_handle, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from DHT20");
        return err;
    }

    // Convert raw data to temperature and humidity
    uint32_t raw_humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4;
    uint32_t raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];

    *humidity = (raw_humidity * 100.0) / 1048576.0;
    *temperature = (raw_temperature * 200.0 / 1048576.0) - 50.0;

    ESP_LOGI(TAG, "Temperature: %.2f°C, Humidity: %.2f%%", *temperature, *humidity);
    return ESP_OK;
}

void app_main(void)
{
    char* taskName = pcTaskGetName(NULL);

    ESP_LOGI(taskName, "Hello, starting up\n");

    i2c_init();

    ssd1309_init();

    ssd1309_clear_screen();
    
    float temperature, humidity;
    while (1) {
        if (read_dht20(&temperature, &humidity) == ESP_OK) {
            ESP_LOGI(TAG, "Temp: %.2f°C, Humidity: %.2f%%", temperature, humidity);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));  // Read every 2 seconds
    }
}
