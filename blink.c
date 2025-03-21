#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

void app_main(void)
{
    DHT20.begin()
    char* taskName = pcTaskGetName(NULL);

    ESP_LOGI(taskName, "Hello, starting up\n");

    while(1)
    {
        ;;
    }
}
