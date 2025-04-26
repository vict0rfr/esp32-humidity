#ifndef BLE_DRIVER
#define BLE_DRIVER

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"

void ble_init(void);
static void ble_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
esp_err_t configure_ble5_advertising(void);
esp_err_t start_ble5_advertising(void);

#endif /* BLE_DRIVER */
