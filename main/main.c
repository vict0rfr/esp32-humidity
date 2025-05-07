#include <stdio.h>
#include <string.h>
#include "main.h"

#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)

static bool tnh_active = false;
static bool settings_active = false;
static bool wifi_active = false;
static bool bluetooth_active = false;

static int current_selection = 0;
static const char* main_menu_items[] = {
    "Flappy Bird",
    "T&H",
    "Time",
    "Settings",
    "Shutdown"
};
#define MAIN_MENU_COUNT (sizeof(main_menu_items) / sizeof(main_menu_items[0]))

// Settings submenu
static int settings_selection = 0;
static const char* settings_menu_items[] = {
    "WiFi",
    "Bluetooth",
    "Back"
};
#define SETTINGS_MENU_COUNT (sizeof(settings_menu_items) / sizeof(settings_menu_items[0]))

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

static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    //Set UART pins (using UART0 default pins)
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
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
    u8g2_DrawStr(&u8g2, 0, 35, buffer);
    u8g2_SendBuffer(&u8g2);
}

// Menu draw functions
static void draw_main_menu(void) {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    for (int i = 0; i < MAIN_MENU_COUNT; i++) {
        if (i == current_selection) {
            char line[32];
            snprintf(line, sizeof(line), "> %s", main_menu_items[i]);
            u8g2_DrawStr(&u8g2, 0, 10 + 12 * i, line);
        } else {
            u8g2_DrawStr(&u8g2, 10, 10 + 12 * i, main_menu_items[i]);
        }
    }
    u8g2_SendBuffer(&u8g2);
}

static void draw_settings_menu(void) {
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    for (int i = 0; i < SETTINGS_MENU_COUNT; i++) {
        if (i == settings_selection) {
            char line[32];
            snprintf(line, sizeof(line), "> %s", settings_menu_items[i]);
            u8g2_DrawStr(&u8g2, 0, 10 + 12 * i, line);
        } else {
            u8g2_DrawStr(&u8g2, 10, 10 + 12 * i, settings_menu_items[i]);
        }
    }
    u8g2_SendBuffer(&u8g2);
}

// Handling input
static void handle_input(const uint8_t* data, int len) {
    // If we’re in the main menu and not in “T&H”
    if (!settings_active && !tnh_active) {
        // Arrow keys for main menu
        if (len >= 3 && data[0] == 0x1B && data[1] == '[') {
            switch (data[2]) {
                case 'A': // Up
                    if (current_selection > 0) {
                        current_selection--;
                    }
                    draw_main_menu();
                    break;
                case 'B': // Down
                    if (current_selection < (MAIN_MENU_COUNT - 1)) {
                        current_selection++;
                    }
                    draw_main_menu();
                    break;
            }
        }
        // Enter key
        if (len == 1 && (data[0] == '\r' || data[0] == '\n')) {
            if (current_selection == 1) { // "T&H"
                tnh_active = true;
            } else if (current_selection == 3) { // "Settings"
                settings_active = true;
                draw_settings_menu();
            }
        }
    }

    // If settings menu is active 
    else if (settings_active && !wifi_active && !bluetooth_active) {
        // Arrow keys for settings menu
        if (len >= 3 && data[0] == 0x1B && data[1] == '[') {
            switch (data[2]) {
                case 'A': // Up
                    if (settings_selection > 0) {
                        settings_selection--;
                    }
                    draw_settings_menu();
                    break;
                case 'B': // Down
                    if (settings_selection < (SETTINGS_MENU_COUNT - 1)) {
                        settings_selection++;
                    }
                    draw_settings_menu();
                    break;
            }
        }
        // Enter key in settings
        if (len == 1 && (data[0] == '\r' || data[0] == '\n')) {
            switch (settings_selection) {
                case 0: // WiFi
                    wifi_active = true;
                    update_screen("WiFi Action", "string");
                    break;
                case 1: // Bluetooth
                    bluetooth_active = true;
                    update_screen("Bluetooth Action", "string");
                    break;
                case 2: // Back
                    settings_active = false;
                    draw_main_menu();
                    break;
            }
        }
    }
    // Add new handling for WiFi submenu
    else if (wifi_active) {
        // Handle specific WiFi menu actions here
        // For now, just prevent arrow keys from doing anything
        
        // You could add real functionality later:
        // if (len >= 3 && data[0] == 0x1B && data[1] == '[') {
        //     switch (data[2]) {
        //         case 'A': // Up - do something
        //         case 'B': // Down - do something
        //     }
        // }
    }
    // Add new handling for Bluetooth submenu
    else if (bluetooth_active) {
        // Handle specific Bluetooth menu actions here
        // For now, just prevent arrow keys from doing anything
    }

    // ESC key - update to clear all submenu states
    if (len == 1 && data[0] == 0x1B) {
        tnh_active = false;
        settings_active = false;
        wifi_active = false;
        bluetooth_active = false;
        draw_main_menu();
    }
}

// Main app
void app_main(void) {
    i2c_master_init();
    u8g2_init();
    uart_init();
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
	// status = connect_wifi();
	// if (WIFI_SUCCESS != status)
	// {
	// 	ESP_LOGI(WIFI_TAG, "Failed to associate to AP, dying...");
    //     update_screen("WiFi Failed", "string");
	// 	// return;
	// }
    
    // connect to tcp server
	// status = connect_tcp_server(update_screen);
	// if (TCP_SUCCESS != status)
	// {
	// 	ESP_LOGI(WIFI_TAG, "Failed to connect to remote server, dying...");
    //     update_screen("TCP Failed", "string");
	// 	// return;
	// }

    draw_main_menu(); // Start on main menu

    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    bool running = true;
    while (running) {
        int len = uart_read_bytes(UART_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            handle_input(data, len);
        }
        // Continuously update T&H if tnh_active
        if (tnh_active) {
            dht20_display(&u8g2);
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        // Add a condition to exit the loop if needed
        // Example: running = false; // Set this based on some condition
    }
    free(data);
}