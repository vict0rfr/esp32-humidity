#ifndef MAIN
#define MAIN

#define PIN_CLK 6
#define PIN_MOSI 7
#define PIN_CS 18
#define PIN_DC 19
#define PIN_RESET 20

#define SDA_PIN 10
#define SCL_PIN 11
#define DHT20_ADDR 0x38

#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS 1000


#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>

#include "sdkconfig.h"

#include <driver/i2c.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include <u8g2.h>
#include "wifi_driver.h"

#include "u8g2_esp32_hal.h"

static const char *DHT20_TAG = "DHT20";
static const char *BT_TAG = "BT_PAIRING";

static void i2c_master_init(void);
static void u8g2_init(void);
void update_screen(const char *message);
static esp_err_t dht20_read(float *temperature, float *humidity);
void app_main(void);

#endif /* MAIN */

