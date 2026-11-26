#ifndef NRF24L01_H
#define NRF24L01_H

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= *
 *  Pinos padrão (ESP32 VSPI / SPI2)                                         *
 * ========================================================================= */
#define NRF24_DEFAULT_PIN_CE        GPIO_NUM_4
#define NRF24_DEFAULT_PIN_CSN       GPIO_NUM_5
#define NRF24_DEFAULT_PIN_MISO      GPIO_NUM_19
#define NRF24_DEFAULT_PIN_MOSI      GPIO_NUM_23
#define NRF24_DEFAULT_PIN_SCK       GPIO_NUM_18
#define NRF24_DEFAULT_PIN_IRQ       GPIO_NUM_NC
#define NRF24_DEFAULT_SPI_CLOCK_HZ  (1000000) /* 1 MHz (seguro em protoboard; máx. 10 MHz) */

/* ========================================================================= *
 *  Limites de Hardware (Datasheet nRF24L01+ v1.0)                           *
 * ========================================================================= */
#define NRF24_MAX_PAYLOAD_SIZE      32U
#define NRF24_MAX_ADDR_WIDTH        5U
#define NRF24_MIN_ADDR_WIDTH        3U
#define NRF24_MAX_CHANNEL           125U /* 2400 MHz .. 2525 MHz */
#define NRF24_MAX_PIPE              5U   /* Pipes 0..5 */
#define NRF24_MAX_RETR_DELAY        15U  /* 250 us .. 4000 us em passos de 250 us */
#define NRF24_MAX_RETR_COUNT        15U

/* ========================================================================= *
 *  Mapa de Registradores do nRF24L01+                                       *
 * ========================================================================= */
#define NRF_REG_CONFIG              0x00U /* Configuração geral */
#define NRF_REG_EN_AA               0x01U /* Habilitação de Auto-ACK (Enhanced ShockBurst) */
#define NRF_REG_EN_RXADDR           0x02U /* Habilitação de pipes RX */
#define NRF_REG_SETUP_AW            0x03U /* Largura do endereço (3, 4 ou 5 bytes) */
#define NRF_REG_SETUP_RETR          0x04U /* Atraso e contagem de retransmissões */
#define NRF_REG_RF_CH               0x05U /* Canal de RF (0..125) */
#define NRF_REG_RF_SETUP            0x06U /* Taxa de transmissão e potência PA */
#define NRF_REG_STATUS              0x07U /* Status e flags de interrupção */
#define NRF_REG_OBSERVE_TX          0x08U /* Contadores de pacotes perdidos e retransmitidos */
#define NRF_REG_RPD                 0x09U /* Detecção de portadora (>= -64 dBm) */
#define NRF_REG_RX_ADDR_P0          0x0AU /* Endereço RX Pipe 0 (até 5 bytes) */
#define NRF_REG_RX_ADDR_P1          0x0BU /* Endereço RX Pipe 1 (até 5 bytes) */
#define NRF_REG_RX_ADDR_P2          0x0CU /* Endereço RX Pipe 2 (1 byte LSB) */
#define NRF_REG_RX_ADDR_P3          0x0DU /* Endereço RX Pipe 3 (1 byte LSB) */
#define NRF_REG_RX_ADDR_P4          0x0EU /* Endereço RX Pipe 4 (1 byte LSB) */
#define NRF_REG_RX_ADDR_P5          0x0FU /* Endereço RX Pipe 5 (1 byte LSB) */
#define NRF_REG_TX_ADDR             0x10U /* Endereço TX (até 5 bytes) */
#define NRF_REG_RX_PW_P0            0x11U /* Tamanho do payload RX Pipe 0 */
#define NRF_REG_RX_PW_P1            0x12U /* Tamanho do payload RX Pipe 1 */
#define NRF_REG_RX_PW_P2            0x13U /* Tamanho do payload RX Pipe 2 */
#define NRF_REG_RX_PW_P3            0x14U /* Tamanho do payload RX Pipe 3 */
#define NRF_REG_RX_PW_P4            0x15U /* Tamanho do payload RX Pipe 4 */
#define NRF_REG_RX_PW_P5            0x16U /* Tamanho do payload RX Pipe 5 */
#define NRF_REG_FIFO_STATUS         0x17U /* Status dos FIFOs TX e RX */
#define NRF_REG_DYNPD               0x1CU /* Payload dinâmico por pipe */
#define NRF_REG_FEATURE             0x1DU /* Recursos avançados (DPL, ACK_PAY, DYN_ACK) */

/* ========================================================================= *
 *  Comandos SPI do nRF24L01+                                                *
 * ========================================================================= */
#define NRF_CMD_R_REGISTER          0x00U /* Ler registrador (OR com endereço 5 bits) */
#define NRF_CMD_W_REGISTER          0x20U /* Escrever registrador (OR com endereço 5 bits) */
#define NRF_CMD_REG_MASK            0x1FU /* Máscara dos 5 bits de endereço */
#define NRF_CMD_R_RX_PL_WID         0x60U /* Ler tamanho do payload no topo do FIFO RX */
#define NRF_CMD_R_RX_PAYLOAD        0x61U /* Ler payload do FIFO RX */
#define NRF_CMD_W_TX_PAYLOAD        0xA0U /* Escrever payload no FIFO TX */
#define NRF_CMD_W_ACK_PAYLOAD       0xA8U /* Escrever payload de ACK (OR com pipe 0..5) */
#define NRF_CMD_W_TX_PAYLOAD_NOACK  0xB0U /* Escrever payload TX sem Auto-ACK */
#define NRF_CMD_FLUSH_TX            0xE1U /* Limpar FIFO TX */
#define NRF_CMD_FLUSH_RX            0xE2U /* Limpar FIFO RX */
#define NRF_CMD_REUSE_TX_PL         0xE3U /* Reutilizar último payload TX */
#define NRF_CMD_NOP                 0xFFU /* Sem operação (retorna STATUS em MISO) */

/* ========================================================================= *
 *  Máscaras de Bits dos Registradores                                       *
 * ========================================================================= */
/* CONFIG (0x00) */
#define NRF_CONFIG_PRIM_RX          (1U << 0) /* 1 = RX, 0 = TX */
#define NRF_CONFIG_PWR_UP           (1U << 1) /* 1 = Power Up, 0 = Power Down */
#define NRF_CONFIG_CRCO             (1U << 2) /* 1 = CRC 2 bytes, 0 = CRC 1 byte */
#define NRF_CONFIG_EN_CRC           (1U << 3) /* 1 = CRC habilitado */
#define NRF_CONFIG_MASK_MAX_RT      (1U << 4) /* 1 = mascara IRQ MAX_RT */
#define NRF_CONFIG_MASK_TX_DS       (1U << 5) /* 1 = mascara IRQ TX_DS */
#define NRF_CONFIG_MASK_RX_DR       (1U << 6) /* 1 = mascara IRQ RX_DR */

/* RF_SETUP (0x06) */
#define NRF_RF_SETUP_RF_PWR_MASK    0x06U
#define NRF_RF_SETUP_RF_DR_HIGH     (1U << 3)
#define NRF_RF_SETUP_PLL_LOCK       (1U << 4)
#define NRF_RF_SETUP_RF_DR_LOW      (1U << 5)
#define NRF_RF_SETUP_CONT_WAVE      (1U << 7)

/* STATUS (0x07) */
#define NRF_STATUS_TX_FULL          (1U << 0) /* FIFO TX cheio */
#define NRF_STATUS_RX_P_NO_MASK     0x0EU     /* Número do pipe do payload disponível */
#define NRF_STATUS_RX_P_NO_SHIFT    1U
#define NRF_STATUS_RX_FIFO_EMPTY_NO 0x07U     /* Valor de RX_P_NO quando FIFO RX está vazio */
#define NRF_STATUS_MAX_RT           (1U << 4) /* Máximo de retransmissões atingido */
#define NRF_STATUS_TX_DS            (1U << 5) /* Dados transmitidos com sucesso */
#define NRF_STATUS_RX_DR            (1U << 6) /* Novos dados recebidos no FIFO RX */
#define NRF_STATUS_IRQ_ALL          (NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT)

/* FIFO_STATUS (0x17) */
#define NRF_FIFO_RX_EMPTY           (1U << 0)
#define NRF_FIFO_RX_FULL            (1U << 1)
#define NRF_FIFO_TX_EMPTY           (1U << 4)
#define NRF_FIFO_TX_FULL            (1U << 5)
#define NRF_FIFO_TX_REUSE           (1U << 6)

/* FEATURE (0x1D) */
#define NRF_FEATURE_EN_DYN_ACK      (1U << 0) /* Habilita comando W_TX_PAYLOAD_NOACK */
#define NRF_FEATURE_EN_ACK_PAY      (1U << 1) /* Habilita payloads em pacotes ACK */
#define NRF_FEATURE_EN_DPL          (1U << 2) /* Habilita payloads dinâmicos */

/* ========================================================================= *
 *  Tipos e Enumerações                                                      *
 * ========================================================================= */

/** @brief Taxa de transmissão de ar (Air Data Rate). */
typedef enum {
    NRF24_DR_250KBPS = 0, /**< 250 kbps (maior alcance, exclusivo nRF24L01+) */
    NRF24_DR_1MBPS   = 1, /**< 1 Mbps (equilíbrio entre alcance e velocidade) */
    NRF24_DR_2MBPS   = 2, /**< 2 Mbps (menor latência e consumo) */
} nrf24_data_rate_t;

/** @brief Nível de potência do amplificador de saída (PA). */
typedef enum {
    NRF24_PA_MIN  = 0, /**< -18 dBm (~7.0 mA) */
    NRF24_PA_LOW  = 1, /**< -12 dBm (~7.5 mA) */
    NRF24_PA_HIGH = 2, /**<  -6 dBm (~9.0 mA) */
    NRF24_PA_MAX  = 3, /**<   0 dBm (~11.3 mA) */
} nrf24_pa_level_t;

/** @brief Configuração de verificação CRC por hardware. */
typedef enum {
    NRF24_CRC_DISABLED = 0, /**< CRC desabilitado (incompatível com Auto-ACK) */
    NRF24_CRC_1BYTE    = 1, /**< CRC de 8 bits */
    NRF24_CRC_2BYTES   = 2, /**< CRC de 16 bits (recomendado) */
} nrf24_crc_t;

/** @brief Largura dos endereços TX/RX em bytes. */
typedef enum {
    NRF24_ADDR_WIDTH_3 = 3, /**< 3 bytes (SETUP_AW = 0x01) */
    NRF24_ADDR_WIDTH_4 = 4, /**< 4 bytes (SETUP_AW = 0x02) */
    NRF24_ADDR_WIDTH_5 = 5, /**< 5 bytes (SETUP_AW = 0x03, padrão) */
} nrf24_addr_width_t;

/** @brief Estado operacional atual do transceptor. */
typedef enum {
    NRF24_MODE_POWER_DOWN = 0,
    NRF24_MODE_STANDBY    = 1,
    NRF24_MODE_TX         = 2,
    NRF24_MODE_RX         = 3,
} nrf24_mode_t;

/** @brief Estatísticas acumuladas de comunicação pelo driver. */
typedef struct {
    uint32_t tx_ok;       /**< Pacotes transmitidos com sucesso (TX_DS) */
    uint32_t tx_max_rt;   /**< Falhas por máximo de retransmissões (MAX_RT) */
    uint32_t tx_timeouts; /**< Falhas por timeout de software */
    uint32_t rx_packets;  /**< Pacotes lidos com sucesso do FIFO RX */
} nrf24_stats_t;

/** @brief Estrutura de configuração para inicialização do módulo nRF24L01+. */
typedef struct {
    spi_host_device_t  spi_host;         /**< Host SPI da ESP32 (ex.: SPI2_HOST) */
    gpio_num_t         pin_ce;           /**< Pino Chip Enable (ativo em nível alto) */
    gpio_num_t         pin_csn;          /**< Pino Chip Select Not (gerenciado via SPI HW) */
    gpio_num_t         pin_miso;         /**< Pino MISO */
    gpio_num_t         pin_mosi;         /**< Pino MOSI */
    gpio_num_t         pin_sck;          /**< Pino SCK */
    gpio_num_t         pin_irq;          /**< Pino IRQ opcional (GPIO_NUM_NC se não usado) */
    int                spi_clock_hz;     /**< Frequência do clock SPI em Hz (máx. 10 MHz) */
    uint8_t            channel;          /**< Canal RF inicial (0..125 -> 2400..2525 MHz) */
    nrf24_data_rate_t  data_rate;        /**< Taxa de dados de ar */
    nrf24_pa_level_t   pa_level;         /**< Potência de transmissão */
    nrf24_crc_t        crc;              /**< Tamanho do CRC */
    nrf24_addr_width_t addr_width;       /**< Largura de endereço (3, 4 ou 5 bytes) */
    uint8_t            retr_delay;       /**< Atraso ARD (0..15 -> (ARD+1)*250 us) */
    uint8_t            retr_count;       /**< Contagem ARC (0..15 retransmissões) */
    uint8_t            payload_size;     /**< Tamanho fixo padrão do payload (1..32 bytes) */
    bool               auto_ack;         /**< Habilita Auto-ACK em todos os pipes no init */
    bool               dynamic_payloads; /**< Habilita payloads de tamanho dinâmico */
    bool               init_spi_bus;     /**< true = driver inicializa spi_bus_initialize() */
} nrf24_config_t;

/** @brief Macro com configuração padrão recomendada para o nRF24L01+. */
#define NRF24_CONFIG_DEFAULT() {               \
    .spi_host         = SPI2_HOST,             \
    .pin_ce           = NRF24_DEFAULT_PIN_CE,  \
    .pin_csn          = NRF24_DEFAULT_PIN_CSN, \
    .pin_miso         = NRF24_DEFAULT_PIN_MISO,\
    .pin_mosi         = NRF24_DEFAULT_PIN_MOSI,\
    .pin_sck          = NRF24_DEFAULT_PIN_SCK, \
    .pin_irq          = NRF24_DEFAULT_PIN_IRQ, \
    .spi_clock_hz     = NRF24_DEFAULT_SPI_CLOCK_HZ, \
    .channel          = 2U,                    \
    .data_rate        = NRF24_DR_1MBPS,        \
    .pa_level         = NRF24_PA_MAX,          \
    .crc              = NRF24_CRC_2BYTES,      \
    .addr_width       = NRF24_ADDR_WIDTH_5,    \
    .retr_delay       = 5U,                    \
    .retr_count       = 15U,                   \
    .payload_size     = 8U,                    \
    .auto_ack         = true,                  \
    .dynamic_payloads = false,                 \
    .init_spi_bus     = true,                  \
}

/** @brief Contexto (handle) de uma instância do transceptor nRF24L01+. */
typedef struct {
    spi_device_handle_t spi;
    nrf24_config_t      cfg;
    nrf24_mode_t        mode;
    uint8_t             addr_width;
    uint8_t             tx_addr[NRF24_MAX_ADDR_WIDTH];
    bool                tx_addr_configured;
    uint8_t             rx_p0_addr[NRF24_MAX_ADDR_WIDTH];
    bool                rx_p0_configured;
    uint8_t             rx_pipe_pw[NRF24_MAX_PIPE + 1U];
    bool                dynamic_payloads;
    bool                ack_payload_enabled;
    bool                initialized;
    nrf24_stats_t       stats;
} nrf24_dev_t;

/* ========================================================================= *
 *  Ciclo de Vida e Verificação de Hardware                                  *
 * ========================================================================= */

/**
 * @brief Inicializa o barramento SPI, os GPIOs e configura o módulo nRF24L01+.
 *
 * @param dev Ponteiro para o contexto do dispositivo (será zerado e preenchido).
 * @param cfg Ponteiro para a configuração desejada (ou NULL para usar padrões).
 * @return ESP_OK em sucesso ou código de erro ESP-IDF.
 */
esp_err_t nrf24_init(nrf24_dev_t *dev, const nrf24_config_t *cfg);

/**
 * @brief Coloca o rádio em Power Down e libera os recursos de SPI.
 *
 * @param dev Ponteiro para o dispositivo inicializado.
 * @return ESP_OK em sucesso ou código de erro ESP-IDF.
 */
esp_err_t nrf24_deinit(nrf24_dev_t *dev);

/**
 * @brief Testa a comunicação SPI com o nRF24L01+ verificando escrita/leitura em registrador.
 *
 * Salva e restaura o valor original de RF_CH para não interferir na configuração ativa.
 *
 * @param dev Ponteiro para o dispositivo inicializado.
 * @return true se o chip respondeu corretamente, false se desconectado ou barramento flutuante.
 */
bool nrf24_is_connected(nrf24_dev_t *dev);

/* ========================================================================= *
 *  Acesso de Baixo Nível via SPI (Transação Única Full-Duplex)              *
 * ========================================================================= */

/**
 * @brief Escreve de 1 a 5 bytes em um registrador do nRF24L01+.
 */
esp_err_t nrf24_write_register(nrf24_dev_t *dev, uint8_t reg, const uint8_t *data, size_t len);

/**
 * @brief Lê de 1 a 5 bytes de um registrador do nRF24L01+.
 */
esp_err_t nrf24_read_register(nrf24_dev_t *dev, uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief Escreve um único byte em um registrador.
 */
esp_err_t nrf24_write_reg_byte(nrf24_dev_t *dev, uint8_t reg, uint8_t value);

/**
 * @brief Lê um único byte de um registrador.
 */
esp_err_t nrf24_read_reg_byte(nrf24_dev_t *dev, uint8_t reg, uint8_t *value);

/**
 * @brief Envia um comando SPI de 1 byte (ex.: FLUSH_TX, FLUSH_RX, NOP) e opcionalmente retorna STATUS.
 */
esp_err_t nrf24_send_command(nrf24_dev_t *dev, uint8_t cmd, uint8_t *status_out);

/* ========================================================================= *
 *  Configuração de Rádio, Endereços e Enhanced ShockBurst                   *
 * ========================================================================= */

/**
 * @brief Seleciona o canal RF (0..125, correspondendo a 2400..2525 MHz).
 */
esp_err_t nrf24_set_channel(nrf24_dev_t *dev, uint8_t channel);

/**
 * @brief Configura a taxa de dados de ar (250 kbps, 1 Mbps ou 2 Mbps) e potência PA.
 */
esp_err_t nrf24_set_rf_setup(nrf24_dev_t *dev, nrf24_data_rate_t data_rate, nrf24_pa_level_t pa_level);

/**
 * @brief Configura o esquema de CRC por hardware (desabilitado, 1 byte ou 2 bytes).
 */
esp_err_t nrf24_set_crc(nrf24_dev_t *dev, nrf24_crc_t crc);

/**
 * @brief Configura a largura dos endereços TX e RX (3, 4 ou 5 bytes).
 */
esp_err_t nrf24_set_address_width(nrf24_dev_t *dev, nrf24_addr_width_t width);

/**
 * @brief Configura atraso (ARD: 0..15 -> 250..4000 us) e contagem (ARC: 0..15) de retransmissões.
 */
esp_err_t nrf24_set_retries(nrf24_dev_t *dev, uint8_t delay, uint8_t count);

/**
 * @brief Define o endereço de transmissão (TX_ADDR).
 *
 * Quando Auto-ACK está ativo, também configura RX_ADDR_P0 com o mesmo endereço
 * conforme exigido pelo datasheet do nRF24L01+ para recepção do pacote ACK.
 */
esp_err_t nrf24_set_tx_address(nrf24_dev_t *dev, const uint8_t *address);

/**
 * @brief Abre e habilita um pipe de recepção (0..5).
 *
 * Para pipes 0 e 1, copia `addr_width` bytes do endereço.
 * Para pipes 2..5, copia apenas 1 byte (LSB), pois compartilham os MSBs do pipe 1.
 * Também configura RX_PW_Px e ativa o bit correspondente em EN_RXADDR.
 */
esp_err_t nrf24_open_rx_pipe(nrf24_dev_t *dev, uint8_t pipe, const uint8_t *address, uint8_t payload_size);

/**
 * @brief Desabilita um pipe de recepção (0..5) em EN_RXADDR.
 */
esp_err_t nrf24_close_rx_pipe(nrf24_dev_t *dev, uint8_t pipe);

/**
 * @brief Habilita ou desabilita Auto-ACK em um pipe específico (0..5).
 */
esp_err_t nrf24_enable_auto_ack(nrf24_dev_t *dev, uint8_t pipe, bool enable);

/**
 * @brief Habilita ou desabilita payloads dinâmicos (FEATURE.EN_DPL + DYNPD).
 */
esp_err_t nrf24_enable_dynamic_payloads(nrf24_dev_t *dev, bool enable);

/**
 * @brief Habilita ou desabilita payloads anexados a pacotes ACK (FEATURE.EN_ACK_PAY).
 */
esp_err_t nrf24_enable_ack_payload(nrf24_dev_t *dev, bool enable);

/**
 * @brief Habilita ou desabilita o comando W_TX_PAYLOAD_NOACK (FEATURE.EN_DYN_ACK).
 */
esp_err_t nrf24_enable_dynamic_ack(nrf24_dev_t *dev, bool enable);

/* ========================================================================= *
 *  Controle de Modos Operacionais                                           *
 * ========================================================================= */

/**
 * @brief Liga o oscilador do rádio (PWR_UP = 1) aguardando o tempo de estabilização Tpd2stby.
 */
esp_err_t nrf24_power_up(nrf24_dev_t *dev);

/**
 * @brief Coloca o rádio em modo de baixo consumo (Power Down, ~900 nA).
 */
esp_err_t nrf24_power_down(nrf24_dev_t *dev);

/**
 * @brief Configura o transceptor em modo transmissor (PRIM_RX = 0, CE = 0, Standby-I).
 */
esp_err_t nrf24_set_tx_mode(nrf24_dev_t *dev);

/**
 * @brief Configura o transceptor em modo receptor ativo (PRIM_RX = 1, CE = 1).
 *
 * Restaura RX_ADDR_P0 caso tenha sido sobrescrito por uma transmissão anterior.
 */
esp_err_t nrf24_set_rx_mode(nrf24_dev_t *dev);

/* ========================================================================= *
 *  Transmissão e Recepção de Dados                                          *
 * ========================================================================= */

/**
 * @brief Envia um pacote de dados (1..32 bytes) e aguarda confirmação (TX_DS) ou falha (MAX_RT).
 *
 * @param dev        Dispositivo inicializado em modo TX.
 * @param data       Ponteiro para os dados a transmitir.
 * @param len        Tamanho em bytes (1..32).
 * @param timeout_ms Tempo máximo de espera pela conclusão em milissegundos (ex.: 50 ms).
 * @return ESP_OK se transmitido/confirmado, ESP_FAIL se MAX_RT, ESP_ERR_TIMEOUT se esgotar o tempo.
 */
esp_err_t nrf24_send(nrf24_dev_t *dev, const uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief Envia um pacote sem aguardar Auto-ACK usando W_TX_PAYLOAD_NOACK.
 *
 * Requer que nrf24_enable_dynamic_ack(dev, true) tenha sido chamado previamente.
 */
esp_err_t nrf24_send_no_ack(nrf24_dev_t *dev, const uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief Verifica se há pacotes disponíveis no FIFO RX e opcionalmente informa o número do pipe.
 *
 * @param dev      Dispositivo inicializado.
 * @param pipe_out Se não NULL, recebe o número do pipe (0..5) do pacote no topo do FIFO.
 * @return true se há dados prontos para leitura, false caso contrário.
 */
bool nrf24_data_available(nrf24_dev_t *dev, uint8_t *pipe_out);

/**
 * @brief Lê o tamanho em bytes do payload dinâmico no topo do FIFO RX (R_RX_PL_WID).
 *
 * Conforme datasheet, se o valor lido for > 32, executa FLUSH_RX automaticamente e retorna ESP_ERR_INVALID_SIZE.
 */
esp_err_t nrf24_get_dynamic_payload_size(nrf24_dev_t *dev, uint8_t *size_out);

/**
 * @brief Lê o próximo payload do FIFO RX para o buffer fornecido e limpa a flag RX_DR.
 *
 * @param dev        Dispositivo inicializado.
 * @param buffer     Buffer de destino.
 * @param max_len    Capacidade do buffer em bytes (1..32).
 * @param bytes_read Opcional: recebe a quantidade de bytes efetivamente lidos.
 * @return ESP_OK em sucesso ou código de erro ESP-IDF.
 */
esp_err_t nrf24_receive(nrf24_dev_t *dev, uint8_t *buffer, size_t max_len, size_t *bytes_read);

/**
 * @brief Carrega um payload de resposta para ser enviado junto ao próximo Auto-ACK no pipe indicado.
 */
esp_err_t nrf24_write_ack_payload(nrf24_dev_t *dev, uint8_t pipe, const uint8_t *data, size_t len);

/* ========================================================================= *
 *  Status, FIFOs e Diagnóstico                                              *
 * ========================================================================= */

/**
 * @brief Lê o registrador STATUS via comando NOP (1 byte SPI).
 */
uint8_t nrf24_get_status(nrf24_dev_t *dev);

/**
 * @brief Limpa as flags de interrupção especificadas em STATUS (ex.: NRF_STATUS_IRQ_ALL).
 */
esp_err_t nrf24_clear_irq_flags(nrf24_dev_t *dev, uint8_t flags);

/**
 * @brief Esvazia o FIFO de transmissão (FLUSH_TX).
 */
esp_err_t nrf24_flush_tx(nrf24_dev_t *dev);

/**
 * @brief Esvazia o FIFO de recepção (FLUSH_RX).
 */
esp_err_t nrf24_flush_rx(nrf24_dev_t *dev);

/**
 * @brief Retorna true se o FIFO TX estiver cheio.
 */
bool nrf24_tx_fifo_full(nrf24_dev_t *dev);

/**
 * @brief Retorna true se o FIFO RX estiver vazio.
 */
bool nrf24_rx_fifo_empty(nrf24_dev_t *dev);

/**
 * @brief Lê o detector de portadora (RPD >= -64 dBm) no canal atual (requer modo RX).
 */
bool nrf24_carrier_detect(nrf24_dev_t *dev);

/**
 * @brief Lê os contadores do registrador OBSERVE_TX (pacotes perdidos e retransmissões).
 */
esp_err_t nrf24_get_observe_tx(nrf24_dev_t *dev, uint8_t *plos_cnt, uint8_t *arc_cnt);

/**
 * @brief Retorna uma cópia das estatísticas de TX/RX acumuladas pelo driver.
 */
nrf24_stats_t nrf24_get_stats(const nrf24_dev_t *dev);

/**
 * @brief Zera as estatísticas acumuladas pelo driver.
 */
void nrf24_reset_stats(nrf24_dev_t *dev);

/**
 * @brief Imprime via ESP_LOGI um resumo completo dos registradores e parâmetros do rádio.
 */
void nrf24_print_details(nrf24_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* NRF24L01_H */
