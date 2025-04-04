#include "nrf24l01.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "MAIN";

void app_main() {
    
    ESP_LOGI(TAG, "🚀 Iniciando...");
    nrf24_init();

    while (1) {

        uint8_t command = 0x00; 
        nrf24_send_data(&command, 1);

        vTaskDelay(pdMS_TO_TICKS(100));

    }

}