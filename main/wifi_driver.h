#ifndef WIFI_DRIVER
#define WIFI_DRIVER

#include "wifi_config.h"
#include "wifi_driver.h"          // Custom header for Wi-Fi driver declarations
#include <string.h>               // For functions like `bzero`
#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>
#include <esp_wifi.h>             // For Wi-Fi functions and configurations
#include <esp_event.h>            // For event handling
#include <esp_log.h>              // For logging
#include <freertos/FreeRTOS.h>    // For FreeRTOS functions
#include <freertos/event_groups.h>// For event group handling
#include <freertos/task.h>        // For task delay (`vTaskDelay`)
#include "u8g2_esp32_hal.h" 

static const char *WIFI_TAG = "WIFI";
typedef void (*screen_update_callback_t)(const char *message);

typedef enum {
    WIFI_SUCCESS = 1 << 0,
    WIFI_FAILURE = 1 << 1,
    TCP_SUCCESS = 1 << 0,
    TCP_FAILURE = 1 << 1,
    MAX_FAILURES = 10
} wifi_status_t;

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

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
esp_err_t connect_wifi(void);
esp_err_t connect_tcp_server(screen_update_callback_t update_screen);
static void handle_server_data(int sock, screen_update_callback_t update_screen);

#endif /* WIFI_DRIVER */
