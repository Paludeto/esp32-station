#include "nrf24l01.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

#define TX_ADDR_SIZE 5

// Tag para usar em logs de debug
static const char *TAG = "NRF24";

// Barramento SPI da ESP32 e configurações (pinos e afins)
spi_device_handle_t spi;

/**
 * @brief Escreve dados em um registrador do módulo NRF24L01 via SPI.
 * 
 * A operação OR entre o comando NRF_CMD_W_REGISTER e o endereço do registrador gera o byte de comando.
 * O protocolo SPI espera esse byte seguido dos dados a serem escritos. 
 * Isso permite escrever múltiplos bytes consecutivos em registradores como TX_ADDR, RX_ADDR_Px, etc.
 * 
 * @param reg   Endereço do registrador base (5 bits menos significativos).
 * @param value Ponteiro para os dados a serem escritos.
 * @param size  Quantidade de bytes a serem enviados ao registrador.
 */
void nrf24_write_register(uint8_t reg, const uint8_t *value, size_t size) {

    // Registradores do módulo NRF suportam no máximo 5 bytes
    if (size > 5) {
        ESP_LOGW(TAG, "Tamanho de bytes excede o permitido\n");
        return;
    }

    uint8_t write_reg = NRF_CMD_W_REGISTER | reg;

    gpio_set_level(PIN_CSN, 0);

    // Comando de write + registrador
    spi_transaction_t reg_cmd = {
        .length = 8, 
        .tx_buffer = &write_reg,
    };
    spi_device_transmit(spi, &reg_cmd);

    // Dados a serem escritos
    spi_transaction_t data = {
        .length = size * 8,
        .tx_buffer = value,
    };
    spi_device_transmit(spi, &data);

    gpio_set_level(PIN_CSN, 1);

}

/**
 * @brief Função para envio e transmissão de dados.
 * 
 * @param data 
 * @param len 
 */
void nrf24_send_data(uint8_t *data, size_t len) {

    if (len > 32) {
        len = 32;
    }

    uint8_t status = check_status();
    
    if (status & 0x10) {  // MAX_RT atingido (bit 4 == 1)

        ESP_LOGW(TAG, "MAX_RT atingido! Resetando...");

        nrf24_write_register(NRF_REG_STATUS, (uint8_t[]){0x10}, 1);
        nrf24_flush_tx(); 

    }

    // Every new command must be started by a high to low transition on CSN.
    // CSN para 0, envio de comando de TX, envio de dados. CSN para 1, fim.
    gpio_set_level(PIN_CSN, 0);

    uint8_t cmd = NRF_CMD_W_TX_PAYLOAD;

    // Comando TX
    spi_transaction_t t_cmd = {
        .length = 8,
        .tx_buffer = &cmd
    };
    spi_device_transmit(spi, &t_cmd);
    
    // Dados a serem enviados
    spi_transaction_t t_data = {
        .length = len * 8,
        .tx_buffer = data
    };
    spi_device_transmit(spi, &t_data);

    gpio_set_level(PIN_CSN, 1); // Fim TX

    // Datasheet exige no mínimo 10µs para iniciar a transmissão com CE = 1 (envia). CE = 0 volta pro standby.
    gpio_set_level(PIN_CE, 1);
    ets_delay_us(10);
    gpio_set_level(PIN_CE, 0);

    ESP_LOGI(TAG, "DADOS ENVIADOS VIA SPI");

}

/**
 * @brief Função para receber dados via SPI e armazená-los em um buffer.
 * 
 * @param buffer 
 * @param len 
 * @return uint8_t 
 */
uint8_t nrf24_receive_data(uint8_t *buffer, size_t len) {   // Lógica similar a transferência de dados

    if (len > 32) {
        len = 32;
    }

    gpio_set_level(PIN_CSN, 0);

    uint8_t cmd = NRF_CMD_R_RX_PAYLOAD;
    spi_transaction_t t_cmd = {
        .length = 8,
        .tx_buffer = &cmd
    };
    spi_device_transmit(spi, &t_cmd);

    spi_transaction_t t_data = {
        .length = 8,
        .rx_buffer = buffer
    };
    spi_device_transmit(spi, &t_data);

    gpio_set_level(PIN_CSN, 1);

    return buffer[0];   // retorna apenas primeiro byte pra checagem

}

/**
 * @brief Função que checa se o buffer FIFO TX do NRF24L01 está cheio. 
 * 
 * @return true 
 * @return false 
 */
bool nrf24_tx_fifo_full() {

    uint8_t status;
    uint8_t cmd = NRF_REG_STATUS;

    gpio_set_level(PIN_CSN, 0);

    // Manda transação SPI para 
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd, // registrador STATUS
        .rx_buffer = &status
    };
    spi_device_transmit(spi, &t);

    gpio_set_level(PIN_CSN, 1);

    // Extrai o primeiro bit. Se for igual a 1, retorna true (full). Se for 0, retorna false.
    return status & 1;

}

/**
 * @brief Função que dá flush no FIFO de TX.
 * 
 */
void nrf24_flush_tx() {

    gpio_set_level(PIN_CSN, 0);
    uint8_t cmd = NRF_CMD_FLUSH_TX;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd
    };
    spi_device_transmit(spi, &t);
    gpio_set_level(PIN_CSN, 1);

    ESP_LOGI(TAG, "FLUSHED TX FIFO");

}

/**
 * @brief Função que dá flush no FIFO de RX.
 * 
 */
void nrf24_flush_rx() {

    gpio_set_level(PIN_CSN, 0);
    uint8_t cmd = NRF_CMD_FLUSH_RX;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd
    };
    spi_device_transmit(spi, &t);
    gpio_set_level(PIN_CSN, 1);

    ESP_LOGI(TAG, "FLUSHED RX FIFO");

}

/**
 * @brief Retorna o conteúdo armazenado no registrador especificado.
 * 
 * @param reg 
 * @return uint8_t 
 */
uint8_t nrf24_read_register(uint8_t reg) {  // Refatorar depois para ler mais bytes

    uint8_t cmd = NRF_CMD_R_REGISTER | (reg & 0x1F);  // Comando de leitura + endereço do registrador (mask)
    uint8_t value = 0;  // Armazena o valor lido

    gpio_set_level(PIN_CSN, 0);
    spi_transaction_t t = {
        .length = 8,  
        .tx_buffer = &cmd,
        .rx_buffer = &value
    };

    ESP_ERROR_CHECK(spi_device_transmit(spi, &t));  // Envia o comando e lê a resposta
    gpio_set_level(PIN_CSN, 1);

    return value;  // Retorna o valor do registrador lido
}

/**
 * @brief Printa o valor do registrador STATUS
 * 
 */
uint8_t nrf24_check_status() {

    uint8_t status = nrf24_read_register(NRF_REG_STATUS);

    ESP_LOGI("NRF24", "STATUS: 0x%02X", status);

    return status;

}

/**
 * @brief Reseta o valor do registrador STATUS.
 * 
 */
void nrf24_reset_status() {

    nrf24_write_register(NRF_REG_STATUS, (uint8_t[]){0x70}, 1);

}

/**
 * @brief Função para configurar o barramento SPI da ESP32 e inicializar o NRF24L01.
 * 
 */
void nrf24_init() {
    
    // Configuração dos barramentos SPI
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MISO,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, // 1MHz
        .mode = 0, // SPI modo 0
        .spics_io_num = PIN_CSN,
        .queue_size = 7
    };

    // Inicialização dos barramentos SPI
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    // Configura os pinos SPI para transmissão de dados
    gpio_set_direction(PIN_CE, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_CSN, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_CE, 0);
    gpio_set_level(PIN_CSN, 1);

    // Reseta STATUS e dá flush nos FIFOs
    nrf24_reset_status();
    nrf24_flush_rx();
    nrf24_flush_tx();

    // "When nRF24L01 is in power down mode it must settle for 1.5ms before it can enter the TX or RX modes."
    vTaskDelay(pdMS_TO_TICKS(1.5));

    // Configuração do NRF24L01
    nrf24_write_register(NRF_REG_CONFIG, (uint8_t[]){0x0E}, 1);     // Escreve no config. Bit PWR_UP = 1, bit PRIM_RX = 0.
    nrf24_write_register(NRF_REG_RF_CH, (uint8_t[]){0x02}, 1);      // Escreve no RF_CH para selecionar o canal 2.
    nrf24_write_register(NRF_REG_RF_SETUP, (uint8_t[]){0x07}, 1);   // PA Control: Escreve bi RF_SETUP 1Mbps, 0dBM 11.3mA

    nrf24_write_register(NRF_REG_TX_ADDR, (uint8_t[]){0, 0, 0, 0, 1}, TX_ADDR_SIZE);    // Seta o endereço de TX
    nrf24_write_register(NRF_REG_RX_ADDR_P0, (uint8_t[]){0, 0, 0, 0, 1}, TX_ADDR_SIZE); // Seta o pipe + endereço de RX

    nrf24_write_register(NRF_REG_EN_AA, (uint8_t[]){0x01}, 1);  // Enable AutoACK, pipe 0 

    ESP_LOGI(TAG, "NRF24L01 INICIALIZADO");
    
}