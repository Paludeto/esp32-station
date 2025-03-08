#include "nrf24l01.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "MAIN";

void app_main() {
    
    ESP_LOGI(TAG, "🚀 Iniciando NRF24L01...");
    nrf24_init();

    while (1) {

        uint8_t command = 0x00; // Comando padrão (parar)
        char input;

        // Lê o input do teclado pela UART
        ESP_LOGI(TAG, "🖥️ Digite W/A/S/D para mover ou X para parar:");
        int res = scanf(" %c", &input); // Pega apenas um caractere

        if (res > 0) {
            switch (input) {
                case 'w': case 'W': command = 0x01; break; // Frente
                case 's': case 'S': command = 0x02; break; // Ré
                case 'a': case 'A': command = 0x03; break; // Esquerda
                case 'd': case 'D': command = 0x04; break; // Direita
                case 'x': case 'X': command = 0x00; break; // Parar
                default:
                    ESP_LOGW(TAG, "❌ Comando inválido! Use W, A, S, D ou X.");
                    continue;
            }

            ESP_LOGI(TAG, "📡 Enviando comando: 0x%02X", command);

            // Envia o comando pelo NRF24L01
            nrf24_send_data(&command, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

}