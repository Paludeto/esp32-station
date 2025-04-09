#include "nrf24l01.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "MAIN";

void app_main() {
    
    ESP_LOGI(TAG, "🚀 Iniciando...");
    nrf24_init();
    
    // Configuração do NRF24L01
    nrf24_tx_on();
    nrf24_set_channel(2);
    nrf24_setup();
    nrf24_set_tx_address((uint8_t[]){0, 0, 0, 0, 1});
    nrf24_set_rx_address(NRF_REG_RX_ADDR_P0, (uint8_t[]){0, 0, 0, 0, 1});
    nrf24_enable_auto_ack(0, true);

    while (1) {

        uint8_t command = 0x00; 
        nrf24_send_data(&command, 1);

        vTaskDelay(pdMS_TO_TICKS(100));

    }

}