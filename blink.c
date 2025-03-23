#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#define I2C_PORT I2C_NUM_0
#define SDA_PIN 10
#define SCL_PIN 11
#define DHT20_ADDR 0x38
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 68

#define SPI_CLK 6
#define SPI_MOSI 7
#define SPI_CS 18
#define SPI_DC 19
#define SPI_RESET 20

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

static const char *TAGDHT20 = "DHT20";
static const char *TAGSSD1309 = "SSD1309";
spi_device_handle_t spi;

void i2c_init() {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}

void spi_init() {
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = -1, // Not used
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SCREEN_WIDTH * SCREEN_HEIGHT / 8 + 16,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = SPI_CS,             // CS pin
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    // Configure DC and RESET pins
    gpio_set_direction(SPI_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(SPI_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(SPI_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void i2c_scanner() {
    printf("Scanning I2C bus...\n");
    for (uint8_t address = 1; address < 127; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            printf("I2C device found at address 0x%02X\n", address);
        }
    }
    printf("Scan complete.\n");
}

void ssd1309_send_command(uint8_t command) {
    gpio_set_level(SPI_DC, 0); // Command mode
    spi_transaction_t t = {
        .length = 8, // Command is 1 byte
        .tx_buffer = &command,
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

void ssd1309_send_data(uint8_t data) {
    gpio_set_level(SPI_DC, 1); // Data mode
    spi_transaction_t t = {
        .length = 8, // Data is 1 byte
        .tx_buffer = &data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

void ssd1309_init() {
    ESP_LOGI(TAGSSD1309, "Initializing OLED display");

    // Reset the display
    gpio_set_level(SPI_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(SPI_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Send initialization commands
    ssd1309_send_command(SSD1309_CMD_DISPLAY_OFF);
    ssd1309_send_command(SSD1309_CMD_SET_DISPLAY_CLOCK);
    ssd1309_send_command(0x80);
    ssd1309_send_command(SSD1309_CMD_SET_MUX_RATIO);
    ssd1309_send_command(0x3F);
    ssd1309_send_command(SSD1309_CMD_SET_OFFSET);
    ssd1309_send_command(0x00);
    ssd1309_send_command(SSD1309_CMD_SET_START_LINE);
    ssd1309_send_command(SSD1309_CMD_SET_SEGMENT_REMAP);
    ssd1309_send_command(SSD1309_CMD_SET_COM_SCAN_DIR);
    ssd1309_send_command(SSD1309_CMD_SET_CONTRAST);
    ssd1309_send_command(0x7F);
    ssd1309_send_command(SSD1309_CMD_DISPLAY_NORMAL);
    ssd1309_send_command(SSD1309_CMD_DISPLAY_ON);
}

// Clear the Screen
void ssd1309_clear_screen() {
    ssd1309_send_command(0x21); // Set column address
    ssd1309_send_command(0x00);
    ssd1309_send_command(0x7F);

    ssd1309_send_command(0x22); // Set page address
    ssd1309_send_command(0x00);
    ssd1309_send_command(0x07);

    for (int i = 0; i < (SCREEN_WIDTH * SCREEN_HEIGHT / 8); i++) {
        ssd1309_send_data(0x00);
    }
}

esp_err_t read_dht20(float *temperature, float *humidity) {
    uint8_t command[] = {0xAC, 0x33, 0x00};
    esp_err_t err = i2c_master_write_to_device(I2C_PORT, DHT20_ADDR, command, sizeof(command), pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAGDHT20, "Failed to send command to DHT20");
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(80));

    uint8_t data[6] = {0};
    err = i2c_master_read_from_device(I2C_PORT, DHT20_ADDR, data, sizeof(data), pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE(TAGDHT20, "Failed to read data from DHT20");
        return err;
    }

    uint32_t raw_humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4;
    uint32_t raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];

    *humidity = (raw_humidity * 100.0) / 1048576.0;
    *temperature = (raw_temperature * 200.0 / 1048576.0) - 50.0;

    return ESP_OK;
}

void draw_horizontal_line() {
    ssd1309_send_command(0x21); // Set column address
    ssd1309_send_command(0x00); // Start column
    ssd1309_send_command(0x7F); // End column

    ssd1309_send_command(0x22); // Set page address
    ssd1309_send_command(0x00); // Start page
    ssd1309_send_command(0x00); // End page

    for (int i = 0; i < SCREEN_WIDTH; i++) {
        ssd1309_send_data(0xFF); // Fill the row with pixels
    }
}

void app_main(void) {
    char* taskName = pcTaskGetName(NULL);

    ESP_LOGI(taskName, "Hello, starting up\n");

    i2c_init();

    spi_init();
    ssd1309_init();

    ssd1309_clear_screen();

    draw_horizontal_line();

    float temperature, humidity;
    while (1) {
        if (read_dht20(&temperature, &humidity) == ESP_OK) {
            ESP_LOGI(TAGDHT20, "Temp: %.2f°C, Humidity: %.2f%%", temperature, humidity);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}