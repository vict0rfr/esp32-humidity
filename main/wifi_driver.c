#include "wifi_driver.h"

static uint8_t tries = 0;
static EventGroupHandle_t wifi_event_group;

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

//use ret and esp_loge to gracefeully handle errors, wifi errors are not fatal.
esp_err_t connect_wifi(void){
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
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
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

esp_err_t connect_tcp_server(screen_update_callback_t update_screen){
	struct sockaddr_in serverInfo = {0};

    serverInfo.sin_family = AF_INET;
    serverInfo.sin_addr.s_addr = inet_addr("192.168.1.59");
    serverInfo.sin_port = htons(12345);

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        ESP_LOGE(WIFI_TAG, "Failed to create a socket");
        return TCP_FAILURE;
    }

    if (connect(sock, (struct sockaddr *)&serverInfo, sizeof(serverInfo)) != 0) {
        ESP_LOGE(WIFI_TAG, "Failed to connect to %s!", inet_ntoa(serverInfo.sin_addr.s_addr));
        close(sock);
        return TCP_FAILURE;
    }

    ESP_LOGI(WIFI_TAG, "Connected to TCP server.");
    handle_server_data(sock, update_screen);

    close(sock);
    return TCP_SUCCESS;
}

static void handle_server_data(int sock, screen_update_callback_t update_screen){
    static char readBuffer[16384] = {0};
    static bool header_parsed = false;
    static wav_header_t wav_header;

    while (1) {
        // Clear the buffer and read data from the server
        bzero(readBuffer, sizeof(readBuffer));
        int r = read(sock, readBuffer, sizeof(readBuffer));
        if (r > 0) {
            ESP_LOGI(WIFI_TAG, "Received %d bytes from server", r);

            if (!header_parsed) {
                if (!parse_wav_header((uint8_t *)readBuffer, &wav_header)) {
                    ESP_LOGE(WIFI_TAG, "Failed to parse WAV header");
                    break;
                }
                header_parsed = true;
                continue; // Skip the header
            }

            if (update_screen) {
                update_screen(readBuffer);
            }
        } else {
            ESP_LOGE(WIFI_TAG, "Failed to read from server");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second before reading again
    }
}