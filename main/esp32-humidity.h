#ifndef ESP32_HUMIDITY
#define ESP32_HUMIDITY

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

#define WIFI_SUCCESS 1 << 0
#define WIFI_FAILURE 1 << 1
#define TCP_SUCCESS 1 << 0
#define TCP_FAILURE 1 << 1
#define MAX_FAILURES 10

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "sdkconfig.h"

#include <driver/i2c.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include <u8g2.h>

#include "u8g2_esp32_hal.h"

typedef struct {
    uint32_t chunk_id;       // "RIFF"
    uint32_t chunk_size;
    uint32_t format;         // "WAVE"
    uint32_t subchunk1_id;   // "fmt"
    uint32_t subchunk1_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    uint32_t subchunk2_id;   // "data"
    uint32_t subchunk2_size;
} wav_header_t;

// static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);


#endif /* ESP32_HUMIDITY */

