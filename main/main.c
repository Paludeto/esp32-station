#include "nrf24l01.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

void app_main() {
    
    ESP_LOGI(TAG, "🚀 Iniciando NRF24L01...");

    nrf24_init();

    while (1) {

        ESP_LOGI(TAG, "📡 Enviando: ");
        
        check_status();

        nrf24_send_data((uint8_t *)"Test", 5);

        vTaskDelay(pdMS_TO_TICKS(1000));

    }

}