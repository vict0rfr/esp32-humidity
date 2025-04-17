#include <stdio.h>
#include <string.h>

#include "esp32-humidity.h"

static EventGroupHandle_t wifi_event_group;

static uint8_t tries = 0;

static const char *DHT20_TAG = "DHT20";
static const char *WIFI_TAG = "WIFI";
static const char *BT_TAG = "BT_PAIRING";

u8g2_t u8g2;

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START){
        ESP_LOGI(WIFI_TAG, "Connecting to AP...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED){
        if (tries < MAX_FAILURES){
            ESP_LOGI(WIFI_TAG, "Reconnecting to AP...");
            esp_wifi_connect();
            tries++;
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAILURE);
        }
    }
}

//event handler for ip events
static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP){
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI_TAG, "STA IP: " IPSTR, IP2STR(&event->ip_info.ip));
        tries = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_SUCCESS);
    }
}

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

bool parse_wav_header(const uint8_t *data, wav_header_t *header) {
    memcpy(header, data, sizeof(wav_header_t));
    if (header->chunk_id != 0x46464952 || header->format != 0x45564157) { // "RIFF" and "WAVE"
        ESP_LOGE(WIFI_TAG, "Invalid WAV file");
        return false;
    }
    ESP_LOGI(WIFI_TAG, "Sample Rate: %u, Channels: %u, Bits per Sample: %u",
        (unsigned int)header->sample_rate,
        (unsigned int)header->num_channels,
        (unsigned int)header->bits_per_sample);
    return true;
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

//use ret and esp_loge to gracefeully handle errors, wifi errors are not fatal.
esp_err_t connect_wifi(){
	int status = WIFI_FAILURE;

	/** INITIALIZE ALL THE THINGS **/
	//initialize the esp network interface
	ESP_ERROR_CHECK(esp_netif_init());

	//initialize default esp event loop
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	//create wifi station in the wifi driver
	esp_netif_create_default_wifi_sta();

	//setup wifi station with the default wifi configuration
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /** EVENT LOOP **/
	wifi_event_group = xEventGroupCreate();

    esp_event_handler_instance_t wifi_handler_event_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &wifi_handler_event_instance));

    esp_event_handler_instance_t got_ip_event_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &ip_event_handler,
                                                        NULL,
                                                        &got_ip_event_instance));

    /** START THE WIFI DRIVER **/
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Victornet",
            .password = "1",
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    // set the wifi controller to be a station
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // set the wifi config
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // start the wifi driver
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(WIFI_TAG, "STA initialization complete");

    /** NOW WE WAIT **/
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
            WIFI_SUCCESS | WIFI_FAILURE,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_SUCCESS) {
        ESP_LOGI(WIFI_TAG, "Connected to ap");
        status = WIFI_SUCCESS;
    } else if (bits & WIFI_FAILURE) {
        ESP_LOGI(WIFI_TAG, "Failed to connect to ap");
        status = WIFI_FAILURE;
    } else {
        ESP_LOGE(WIFI_TAG, "UNEXPECTED EVENT");
        status = WIFI_FAILURE;
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip_event_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler_event_instance));
    vEventGroupDelete(wifi_event_group);

    return status;
}

esp_err_t connect_tcp_server(void){
	struct sockaddr_in serverInfo = {0};
	static char readBuffer[8096] = {0};

	serverInfo.sin_family = AF_INET;
	serverInfo.sin_addr.s_addr = inet_addr("192.168.1.249");
	serverInfo.sin_port = htons(12345);

	int sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0){
		ESP_LOGE(WIFI_TAG, "Failed to create a socket..?");
		return TCP_FAILURE;
	}

	if (connect(sock, (struct sockaddr *)&serverInfo, sizeof(serverInfo)) != 0){
		ESP_LOGE(WIFI_TAG, "Failed to connect to %s!", inet_ntoa(serverInfo.sin_addr.s_addr));
		close(sock);
		return TCP_FAILURE;
	}

	ESP_LOGI(WIFI_TAG, "Connected to TCP server.");
	while (1) {
        // Clear the buffer and read data from the server
        bzero(readBuffer, sizeof(readBuffer));
        int r = read(sock, readBuffer, sizeof(readBuffer));
        if (r > 0) {

            ESP_LOGI(WIFI_TAG, "Received %d bytes from server", r);
            for (int i = 0; i < r; i++) {
                ESP_LOGI(WIFI_TAG, "Byte %d: 0x%02X", i, (uint8_t)readBuffer[i]);
            }
            static bool header_parsed = false;
            static wav_header_t wav_header;

            if (!header_parsed) {
                if (!parse_wav_header((uint8_t *)readBuffer, &wav_header)) {
                    ESP_LOGE(WIFI_TAG, "Failed to parse WAV header");
                    break;
                }
                header_parsed = true;
                continue; // Skip the header
            }
                
            // Display the received message on the screen
            u8g2_ClearBuffer(&u8g2);
            u8g2_SetFont(&u8g2, u8g2_font_ncenB12_tr);
            u8g2_DrawStr(&u8g2, 0, 15, "Message:");
            u8g2_DrawStr(&u8g2, 0, 35, readBuffer);
            u8g2_SendBuffer(&u8g2);
        } else {
            ESP_LOGE(WIFI_TAG, "Failed to read from server");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second before reading again
    }

    close(sock);
    return TCP_SUCCESS;
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

    // bt_init();

    // connect to wireless AP
	status = connect_wifi();
	if (WIFI_SUCCESS != status)
	{
		ESP_LOGI(WIFI_TAG, "Failed to associate to AP, dying...");
		return;
	}
	
	status = connect_tcp_server();
	if (TCP_SUCCESS != status)
	{
		ESP_LOGI(WIFI_TAG, "Failed to connect to remote server, dying...");
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