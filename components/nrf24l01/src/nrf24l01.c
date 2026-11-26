#include "nrf24l01.h"

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "NRF24";

/* Tempos do Datasheet nRF24L01+ (v1.0, Tabela 16) */
#define NRF24_DELAY_POWER_ON_MS      10U   /* Estabilização pós-reset */
#define NRF24_DELAY_PWR_UP_US        5000U /* Tpd2stby máx. 4.5 ms com cristal externo */
#define NRF24_DELAY_CE_PULSE_US      15U   /* Thce mín. 10 us (+ margem de segurança) */
#define NRF24_DELAY_STANDBY_TO_RX_US 150U  /* Tstby2a mín. 130 us */

/* ========================================================================= *
 *  Funções Auxiliares Internas                                              *
 * ========================================================================= */

static inline bool nrf24_check_ready(const nrf24_dev_t *dev) {
    return (dev != NULL) && dev->initialized && (dev->spi != NULL);
}

static inline void nrf24_set_ce(const nrf24_dev_t *dev, int level) {
    gpio_set_level(dev->cfg.pin_ce, level);
}

/**
 * @brief Executa uma transação SPI full-duplex única (CSN gerenciado por hardware).
 *
 * Em uma única transação:
 *   TX: [cmd, tx_data[0..len-1]] (ou NRF_CMD_NOP se tx_data == NULL)
 *   RX: [status, rx_data[0..len-1]]
 *
 * Evita qualquer pulso indevido em CSN entre o byte de comando e o payload.
 */
static esp_err_t nrf24_spi_transfer(nrf24_dev_t *dev,
                                    uint8_t cmd,
                                    const uint8_t *tx_data,
                                    uint8_t *rx_data,
                                    size_t len,
                                    uint8_t *status_out) {
    if (!nrf24_check_ready(dev) || len > NRF24_MAX_PAYLOAD_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx_buf[NRF24_MAX_PAYLOAD_SIZE + 1U];
    uint8_t rx_buf[NRF24_MAX_PAYLOAD_SIZE + 1U];

    tx_buf[0] = cmd;
    if (len > 0U) {
        if (tx_data != NULL) {
            memcpy(&tx_buf[1], tx_data, len);
        } else {
            memset(&tx_buf[1], NRF_CMD_NOP, len);
        }
    }

    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));
    trans.length    = (len + 1U) * 8U;
    trans.tx_buffer = tx_buf;
    trans.rx_buffer = rx_buf;

    esp_err_t err = spi_device_transmit(dev->spi, &trans);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha na transação SPI (cmd=0x%02X): %s", cmd, esp_err_to_name(err));
        return err;
    }

    if (status_out != NULL) {
        *status_out = rx_buf[0];
    }
    if ((rx_data != NULL) && (len > 0U)) {
        memcpy(rx_data, &rx_buf[1], len);
    }

    return ESP_OK;
}

/**
 * @brief Função interna comum de transmissão (com ou sem Auto-ACK).
 */
static esp_err_t nrf24_send_internal(nrf24_dev_t *dev,
                                     uint8_t cmd,
                                     const uint8_t *data,
                                     size_t len,
                                     uint32_t timeout_ms) {
    if (!nrf24_check_ready(dev) || (data == NULL) || (len == 0U) || (len > NRF24_MAX_PAYLOAD_SIZE)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->mode != NRF24_MODE_TX) {
        esp_err_t mode_err = nrf24_set_tx_mode(dev);
        if (mode_err != ESP_OK) {
            return mode_err;
        }
    }

    /* Limpa flags pendentes de transmissões anteriores */
    uint8_t status = nrf24_get_status(dev);
    if (status & (NRF_STATUS_MAX_RT | NRF_STATUS_TX_DS)) {
        nrf24_clear_irq_flags(dev, NRF_STATUS_MAX_RT | NRF_STATUS_TX_DS);
    }
    if (status & NRF_STATUS_TX_FULL) {
        ESP_LOGW(TAG, "FIFO TX cheio antes do envio; executando FLUSH_TX");
        nrf24_flush_tx(dev);
    }

    /* Prepara payload (preenche com zeros se payload fixo for maior que len) */
    uint8_t payload[NRF24_MAX_PAYLOAD_SIZE];
    size_t tx_len = len;
    if (!dev->dynamic_payloads && (dev->cfg.payload_size > 0U) && (dev->cfg.payload_size <= NRF24_MAX_PAYLOAD_SIZE)) {
        tx_len = dev->cfg.payload_size;
        memset(payload, 0, tx_len);
        size_t copy_len = (len < tx_len) ? len : tx_len;
        memcpy(payload, data, copy_len);
    } else {
        memcpy(payload, data, len);
    }

    /* Carrega payload no FIFO TX */
    esp_err_t err = nrf24_spi_transfer(dev, cmd, payload, NULL, tx_len, NULL);
    if (err != ESP_OK) {
        return err;
    }

    /* Pulso de CE >= 10 us para iniciar transmissão RF */
    nrf24_set_ce(dev, 1);
    esp_rom_delay_us(NRF24_DELAY_CE_PULSE_US);
    nrf24_set_ce(dev, 0);

    /* Aguarda conclusão da transmissão (TX_DS ou MAX_RT) com timeout */
    int64_t deadline_us = esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);
    while (true) {
        status = nrf24_get_status(dev);

        if (status & NRF_STATUS_TX_DS) {
            nrf24_clear_irq_flags(dev, NRF_STATUS_TX_DS);
            dev->stats.tx_ok++;
            return ESP_OK;
        }

        if (status & NRF_STATUS_MAX_RT) {
            nrf24_clear_irq_flags(dev, NRF_STATUS_MAX_RT);
            nrf24_flush_tx(dev);
            dev->stats.tx_max_rt++;
            ESP_LOGW(TAG, "MAX_RT atingido (sem ACK do receptor)");
            return ESP_FAIL;
        }

        if (esp_timer_get_time() >= deadline_us) {
            nrf24_flush_tx(dev);
            dev->stats.tx_timeouts++;
            ESP_LOGW(TAG, "Timeout na transmissão TX (%lu ms)", (unsigned long)timeout_ms);
            return ESP_ERR_TIMEOUT;
        }

        esp_rom_delay_us(100);
    }
}

/* ========================================================================= *
 *  Acesso de Baixo Nível                                                    *
 * ========================================================================= */

esp_err_t nrf24_write_register(nrf24_dev_t *dev, uint8_t reg, const uint8_t *data, size_t len) {
    if (!nrf24_check_ready(dev) || (data == NULL) || (len == 0U) || (len > NRF24_MAX_ADDR_WIDTH)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t cmd = (uint8_t)(NRF_CMD_W_REGISTER | (reg & NRF_CMD_REG_MASK));
    return nrf24_spi_transfer(dev, cmd, data, NULL, len, NULL);
}

esp_err_t nrf24_read_register(nrf24_dev_t *dev, uint8_t reg, uint8_t *data, size_t len) {
    if (!nrf24_check_ready(dev) || (data == NULL) || (len == 0U) || (len > NRF24_MAX_ADDR_WIDTH)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t cmd = (uint8_t)(NRF_CMD_R_REGISTER | (reg & NRF_CMD_REG_MASK));
    return nrf24_spi_transfer(dev, cmd, NULL, data, len, NULL);
}

esp_err_t nrf24_write_reg_byte(nrf24_dev_t *dev, uint8_t reg, uint8_t value) {
    return nrf24_write_register(dev, reg, &value, 1U);
}

esp_err_t nrf24_read_reg_byte(nrf24_dev_t *dev, uint8_t reg, uint8_t *value) {
    return nrf24_read_register(dev, reg, value, 1U);
}

esp_err_t nrf24_send_command(nrf24_dev_t *dev, uint8_t cmd, uint8_t *status_out) {
    if (!nrf24_check_ready(dev)) {
        return ESP_ERR_INVALID_STATE;
    }
    return nrf24_spi_transfer(dev, cmd, NULL, NULL, 0U, status_out);
}

/* ========================================================================= *
 *  Ciclo de Vida e Diagnóstico de Hardware                                  *
 * ========================================================================= */

esp_err_t nrf24_init(nrf24_dev_t *dev, const nrf24_config_t *cfg) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));

    if (cfg != NULL) {
        dev->cfg = *cfg;
    } else {
        nrf24_config_t def = NRF24_CONFIG_DEFAULT();
        dev->cfg = def;
    }

    if (dev->cfg.spi_clock_hz <= 0) {
        dev->cfg.spi_clock_hz = NRF24_DEFAULT_SPI_CLOCK_HZ;
    }
    if (dev->cfg.channel > NRF24_MAX_CHANNEL) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((dev->cfg.addr_width < NRF24_ADDR_WIDTH_3) || (dev->cfg.addr_width > NRF24_ADDR_WIDTH_5)) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((dev->cfg.payload_size == 0U) || (dev->cfg.payload_size > NRF24_MAX_PAYLOAD_SIZE)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Configura pino CE como saída (CSN fica sob controle exclusivo do periférico SPI) */
    gpio_config_t ce_conf = {
        .pin_bit_mask = (1ULL << (uint32_t)dev->cfg.pin_ce),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&ce_conf);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(dev->cfg.pin_ce, 0);

    /* Configura pino IRQ opcional como entrada com pull-up */
    if (dev->cfg.pin_irq != GPIO_NUM_NC) {
        gpio_config_t irq_conf = {
            .pin_bit_mask = (1ULL << (uint32_t)dev->cfg.pin_irq),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        err = gpio_config(&irq_conf);
        if (err != ESP_OK) {
            return err;
        }
    }

    /* Inicializa barramento SPI (se solicitado) */
    if (dev->cfg.init_spi_bus) {
        spi_bus_config_t buscfg = {
            .miso_io_num     = dev->cfg.pin_miso,
            .mosi_io_num     = dev->cfg.pin_mosi,
            .sclk_io_num     = dev->cfg.pin_sck,
            .quadwp_io_num   = -1,
            .quadhd_io_num   = -1,
            .max_transfer_sz = (int)(NRF24_MAX_PAYLOAD_SIZE + 4U),
        };
        err = spi_bus_initialize(dev->cfg.spi_host, &buscfg, SPI_DMA_CH_AUTO);
        if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
            ESP_LOGE(TAG, "Erro ao inicializar barramento SPI: %s", esp_err_to_name(err));
            return err;
        }
    }

    /* Adiciona dispositivo nRF24L01+ no barramento SPI (Modo 0: CPOL=0, CPHA=0) */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = dev->cfg.spi_clock_hz,
        .mode           = 0,
        .spics_io_num   = dev->cfg.pin_csn,
        .queue_size     = 4,
    };
    err = spi_bus_add_device(dev->cfg.spi_host, &devcfg, &dev->spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao adicionar dispositivo SPI: %s", esp_err_to_name(err));
        return err;
    }

    dev->initialized = true;
    dev->mode        = NRF24_MODE_POWER_DOWN;
    dev->addr_width  = (uint8_t)dev->cfg.addr_width;

    /* Aguarda estabilização após energização */
    vTaskDelay(pdMS_TO_TICKS(NRF24_DELAY_POWER_ON_MS));

    /* Garante estado limpo inicial */
    if ((err = nrf24_write_reg_byte(dev, NRF_REG_CONFIG, 0x00U)) != ESP_OK) goto fail;
    if ((err = nrf24_set_crc(dev, dev->cfg.crc)) != ESP_OK) goto fail;
    if ((err = nrf24_set_address_width(dev, dev->cfg.addr_width)) != ESP_OK) goto fail;
    if ((err = nrf24_set_retries(dev, dev->cfg.retr_delay, dev->cfg.retr_count)) != ESP_OK) goto fail;
    if ((err = nrf24_set_channel(dev, dev->cfg.channel)) != ESP_OK) goto fail;
    if ((err = nrf24_set_rf_setup(dev, dev->cfg.data_rate, dev->cfg.pa_level)) != ESP_OK) goto fail;

    /* Configura Auto-ACK global inicial */
    uint8_t en_aa = dev->cfg.auto_ack ? 0x3FU : 0x00U;
    if ((err = nrf24_write_reg_byte(dev, NRF_REG_EN_AA, en_aa)) != ESP_OK) goto fail;

    /* Configura payload dinâmico se solicitado */
    if ((err = nrf24_enable_dynamic_payloads(dev, dev->cfg.dynamic_payloads)) != ESP_OK) goto fail;

    /* Limpa flags e FIFOs */
    if ((err = nrf24_clear_irq_flags(dev, NRF_STATUS_IRQ_ALL)) != ESP_OK) goto fail;
    if ((err = nrf24_flush_tx(dev)) != ESP_OK) goto fail;
    if ((err = nrf24_flush_rx(dev)) != ESP_OK) goto fail;

    /* Liga oscilador do módulo (Standby-I) */
    if ((err = nrf24_power_up(dev)) != ESP_OK) goto fail;

    ESP_LOGI(TAG, "nRF24L01+ inicializado (CH=%u, %u MHz, SPI=%d Hz)",
             dev->cfg.channel, 2400U + dev->cfg.channel, dev->cfg.spi_clock_hz);
    return ESP_OK;

fail:
    nrf24_deinit(dev);
    return err;
}

esp_err_t nrf24_deinit(nrf24_dev_t *dev) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dev->initialized && (dev->spi != NULL)) {
        (void)nrf24_power_down(dev);
        (void)spi_bus_remove_device(dev->spi);
        dev->spi = NULL;
    }

    if (dev->cfg.init_spi_bus && dev->initialized) {
        (void)spi_bus_free(dev->cfg.spi_host);
    }

    dev->initialized = false;
    dev->mode        = NRF24_MODE_POWER_DOWN;
    return ESP_OK;
}

bool nrf24_is_connected(nrf24_dev_t *dev) {
    if (!nrf24_check_ready(dev)) {
        return false;
    }

    uint8_t aw = 0x00U;
    if (nrf24_read_reg_byte(dev, NRF_REG_SETUP_AW, &aw) != ESP_OK) {
        return false;
    }
    /* SETUP_AW válido só pode ser 0x01, 0x02 ou 0x03 (descarta barramento preso em 0x00 ou 0xFF) */
    if ((aw < 0x01U) || (aw > 0x03U)) {
        return false;
    }

    /* Teste de escrita e leitura de volta em RF_CH preservando o canal original */
    uint8_t original_ch = 0U;
    if (nrf24_read_reg_byte(dev, NRF_REG_RF_CH, &original_ch) != ESP_OK) {
        return false;
    }

    uint8_t test_ch = (original_ch == 0x55U) ? 0x2AU : 0x55U;
    uint8_t readback = 0U;

    bool was_rx = (dev->mode == NRF24_MODE_RX);
    if (was_rx) {
        nrf24_set_ce(dev, 0);
    }

    if (nrf24_write_reg_byte(dev, NRF_REG_RF_CH, test_ch) != ESP_OK) {
        if (was_rx) nrf24_set_ce(dev, 1);
        return false;
    }
    esp_err_t err = nrf24_read_reg_byte(dev, NRF_REG_RF_CH, &readback);
    (void)nrf24_write_reg_byte(dev, NRF_REG_RF_CH, original_ch);

    if (was_rx) {
        nrf24_set_ce(dev, 1);
    }

    return (err == ESP_OK) && (readback == test_ch);
}

/* ========================================================================= *
 *  Configuração de RF, Endereços e Pipes                                    *
 * ========================================================================= */

esp_err_t nrf24_set_channel(nrf24_dev_t *dev, uint8_t channel) {
    if (!nrf24_check_ready(dev) || (channel > NRF24_MAX_CHANNEL)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = nrf24_write_reg_byte(dev, NRF_REG_RF_CH, channel);
    if (err == ESP_OK) {
        dev->cfg.channel = channel;
    }
    return err;
}

esp_err_t nrf24_set_rf_setup(nrf24_dev_t *dev, nrf24_data_rate_t data_rate, nrf24_pa_level_t pa_level) {
    if (!nrf24_check_ready(dev)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t setup = 0x00U;

    switch (data_rate) {
        case NRF24_DR_250KBPS:
            setup |= NRF_RF_SETUP_RF_DR_LOW;
            break;
        case NRF24_DR_1MBPS:
            break;
        case NRF24_DR_2MBPS:
            setup |= NRF_RF_SETUP_RF_DR_HIGH;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    switch (pa_level) {
        case NRF24_PA_MIN:
            setup |= (0x00U << 1);
            break;
        case NRF24_PA_LOW:
            setup |= (0x01U << 1);
            break;
        case NRF24_PA_HIGH:
            setup |= (0x02U << 1);
            break;
        case NRF24_PA_MAX:
            setup |= (0x03U << 1);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = nrf24_write_reg_byte(dev, NRF_REG_RF_SETUP, setup);
    if (err == ESP_OK) {
        dev->cfg.data_rate = data_rate;
        dev->cfg.pa_level  = pa_level;
    }
    return err;
}

esp_err_t nrf24_set_crc(nrf24_dev_t *dev, nrf24_crc_t crc) {
    if (!nrf24_check_ready(dev)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t config = 0U;
    esp_err_t err = nrf24_read_reg_byte(dev, NRF_REG_CONFIG, &config);
    if (err != ESP_OK) {
        return err;
    }

    config &= (uint8_t)~(NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO);
    switch (crc) {
        case NRF24_CRC_DISABLED:
            break;
        case NRF24_CRC_1BYTE:
            config |= NRF_CONFIG_EN_CRC;
            break;
        case NRF24_CRC_2BYTES:
            config |= (NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO);
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    err = nrf24_write_reg_byte(dev, NRF_REG_CONFIG, config);
    if (err == ESP_OK) {
        dev->cfg.crc = crc;
    }
    return err;
}

esp_err_t nrf24_set_address_width(nrf24_dev_t *dev, nrf24_addr_width_t width) {
    if (!nrf24_check_ready(dev)) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((width < NRF24_ADDR_WIDTH_3) || (width > NRF24_ADDR_WIDTH_5)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg_val = (uint8_t)((uint8_t)width - 2U); /* 3->1, 4->2, 5->3 */
    esp_err_t err = nrf24_write_reg_byte(dev, NRF_REG_SETUP_AW, reg_val);
    if (err == ESP_OK) {
        dev->addr_width     = (uint8_t)width;
        dev->cfg.addr_width = width;
    }
    return err;
}

esp_err_t nrf24_set_retries(nrf24_dev_t *dev, uint8_t delay, uint8_t count) {
    if (!nrf24_check_ready(dev) || (delay > NRF24_MAX_RETR_DELAY) || (count > NRF24_MAX_RETR_COUNT)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t retr = (uint8_t)(((delay & 0x0FU) << 4) | (count & 0x0FU));
    esp_err_t err = nrf24_write_reg_byte(dev, NRF_REG_SETUP_RETR, retr);
    if (err == ESP_OK) {
        dev->cfg.retr_delay = delay;
        dev->cfg.retr_count = count;
    }
    return err;
}

esp_err_t nrf24_set_tx_address(nrf24_dev_t *dev, const uint8_t *address) {
    if (!nrf24_check_ready(dev) || (address == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = nrf24_write_register(dev, NRF_REG_TX_ADDR, address, dev->addr_width);
    if (err != ESP_OK) {
        return err;
    }

    memcpy(dev->tx_addr, address, dev->addr_width);
    dev->tx_addr_configured = true;

    /* Para receber pacotes Auto-ACK, RX_ADDR_P0 deve coincidir com TX_ADDR */
    if (dev->cfg.auto_ack) {
        err = nrf24_write_register(dev, NRF_REG_RX_ADDR_P0, address, dev->addr_width);
    }
    return err;
}

esp_err_t nrf24_open_rx_pipe(nrf24_dev_t *dev, uint8_t pipe, const uint8_t *address, uint8_t payload_size) {
    if (!nrf24_check_ready(dev) || (pipe > NRF24_MAX_PIPE) || (address == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((payload_size == 0U) || (payload_size > NRF24_MAX_PAYLOAD_SIZE)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t addr_reg = (uint8_t)(NRF_REG_RX_ADDR_P0 + pipe);
    size_t  addr_len = (pipe < 2U) ? dev->addr_width : 1U;

    esp_err_t err = nrf24_write_register(dev, addr_reg, address, addr_len);
    if (err != ESP_OK) {
        return err;
    }

    /* Guarda endereço do pipe 0 para restaurar caso TX sobrescreva RX_ADDR_P0 */
    if (pipe == 0U) {
        memcpy(dev->rx_p0_addr, address, dev->addr_width);
        dev->rx_p0_configured = true;
    }
    dev->rx_pipe_pw[pipe] = payload_size;

    uint8_t pw_reg = (uint8_t)(NRF_REG_RX_PW_P0 + pipe);
    err = nrf24_write_reg_byte(dev, pw_reg, payload_size);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t en_rx = 0U;
    err = nrf24_read_reg_byte(dev, NRF_REG_EN_RXADDR, &en_rx);
    if (err != ESP_OK) {
        return err;
    }

    en_rx |= (uint8_t)(1U << pipe);
    return nrf24_write_reg_byte(dev, NRF_REG_EN_RXADDR, en_rx);
}

esp_err_t nrf24_close_rx_pipe(nrf24_dev_t *dev, uint8_t pipe) {
    if (!nrf24_check_ready(dev) || (pipe > NRF24_MAX_PIPE)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t en_rx = 0U;
    esp_err_t err = nrf24_read_reg_byte(dev, NRF_REG_EN_RXADDR, &en_rx);
    if (err != ESP_OK) {
        return err;
    }

    en_rx &= (uint8_t)~(1U << pipe);
    err = nrf24_write_reg_byte(dev, NRF_REG_EN_RXADDR, en_rx);
    if ((err == ESP_OK) && (pipe == 0U)) {
        dev->rx_p0_configured = false;
    }
    return err;
}

esp_err_t nrf24_enable_auto_ack(nrf24_dev_t *dev, uint8_t pipe, bool enable) {
    if (!nrf24_check_ready(dev) || (pipe > NRF24_MAX_PIPE)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t en_aa = 0U;
    esp_err_t err = nrf24_read_reg_byte(dev, NRF_REG_EN_AA, &en_aa);
    if (err != ESP_OK) {
        return err;
    }

    if (enable) {
        en_aa |= (uint8_t)(1U << pipe);
    } else {
        en_aa &= (uint8_t)~(1U << pipe);
    }

    return nrf24_write_reg_byte(dev, NRF_REG_EN_AA, en_aa);
}

esp_err_t nrf24_enable_dynamic_payloads(nrf24_dev_t *dev, bool enable) {
    if (!nrf24_check_ready(dev)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t feature = 0U;
    esp_err_t err = nrf24_read_reg_byte(dev, NRF_REG_FEATURE, &feature);
    if (err != ESP_OK) {
        return err;
    }

    if (enable) {
        feature |= NRF_FEATURE_EN_DPL;
    } else {
        feature &= (uint8_t)~(NRF_FEATURE_EN_DPL | NRF_FEATURE_EN_ACK_PAY);
    }

    err = nrf24_write_reg_byte(dev, NRF_REG_FEATURE, feature);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t dynpd = enable ? 0x3FU : 0x00U;
    err = nrf24_write_reg_byte(dev, NRF_REG_DYNPD, dynpd);
    if (err == ESP_OK) {
        dev->dynamic_payloads     = enable;
        dev->cfg.dynamic_payloads = enable;
        if (!enable) {
            dev->ack_payload_enabled = false;
        }
    }
    return err;
}

esp_err_t nrf24_enable_ack_payload(nrf24_dev_t *dev, bool enable) {
    if (!nrf24_check_ready(dev)) {
        return ESP_ERR_INVALID_STATE;
    }

    if (enable && !dev->dynamic_payloads) {
        /* Datasheet exige EN_DPL ativo para uso de ACK payloads */
        esp_err_t dpl_err = nrf24_enable_dynamic_payloads(dev, true);
        if (dpl_err != ESP_OK) {
            return dpl_err;
        }
    }

    uint8_t feature = 0U;
    esp_err_t err = nrf24_read_reg_byte(dev, NRF_REG_FEATURE, &feature);
    if (err != ESP_OK) {
        return err;
    }

    if (enable) {
        feature |= (NRF_FEATURE_EN_ACK_PAY | NRF_FEATURE_EN_DPL);
    } else {
        feature &= (uint8_t)~NRF_FEATURE_EN_ACK_PAY;
    }

    err = nrf24_write_reg_byte(dev, NRF_REG_FEATURE, feature);
    if (err == ESP_OK) {
        dev->ack_payload_enabled = enable;
    }
    return err;
}

esp_err_t nrf24_enable_dynamic_ack(nrf24_dev_t *dev, bool enable) {
    if (!nrf24_check_ready(dev)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t feature = 0U;
    esp_err_t err = nrf24_read_reg_byte(dev, NRF_REG_FEATURE, &feature);
    if (err != ESP_OK) {
        return err;
    }

    if (enable) {
        feature |= NRF_FEATURE_EN_DYN_ACK;
    } else {
        feature &= (uint8_t)~NRF_FEATURE_EN_DYN_ACK;
    }

    return nrf24_write_reg_byte(dev, NRF_REG_FEATURE, feature);
}

/* ========================================================================= *
 *  Controle de Modos Operacionais                                           *
 * ========================================================================= */

esp_err_t nrf24_power_up(nrf24_dev_t *dev) {
    if (!nrf24_check_ready(dev)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t config = 0U;
    esp_err_t err = nrf24_read_reg_byte(dev, NRF_REG_CONFIG, &config);
    if (err != ESP_OK) {
        return err;
    }

    if ((config & NRF_CONFIG_PWR_UP) == 0U) {
        config |= NRF_CONFIG_PWR_UP;
        err = nrf24_write_reg_byte(dev, NRF_REG_CONFIG, config);
        if (err != ESP_OK) {
            return err;
        }
        /* Aguarda estabilização do cristal/oscilador (Tpd2stby <= 4.5 ms) */
        esp_rom_delay_us(NRF24_DELAY_PWR_UP_US);
    }

    if (dev->mode == NRF24_MODE_POWER_DOWN) {
        dev->mode = NRF24_MODE_STANDBY;
    }
    return ESP_OK;
}

esp_err_t nrf24_power_down(nrf24_dev_t *dev) {
    if (!nrf24_check_ready(dev)) {
        return ESP_ERR_INVALID_STATE;
    }

    nrf24_set_ce(dev, 0);

    uint8_t config = 0U;
    esp_err_t err = nrf24_read_reg_byte(dev, NRF_REG_CONFIG, &config);
    if (err != ESP_OK) {
        return err;
    }

    config &= (uint8_t)~NRF_CONFIG_PWR_UP;
    err = nrf24_write_reg_byte(dev, NRF_REG_CONFIG, config);
    if (err == ESP_OK) {
        dev->mode = NRF24_MODE_POWER_DOWN;
    }
    return err;
}

esp_err_t nrf24_set_tx_mode(nrf24_dev_t *dev) {
    if (!nrf24_check_ready(dev)) {
        return ESP_ERR_INVALID_STATE;
    }

    nrf24_set_ce(dev, 0);

    esp_err_t err = nrf24_power_up(dev);
    if (err != ESP_OK) {
        return err;
    }

    /* Garante RX_ADDR_P0 == TX_ADDR para receber Auto-ACK após sair de modo RX */
    if (dev->cfg.auto_ack && dev->tx_addr_configured) {
        err = nrf24_write_register(dev, NRF_REG_RX_ADDR_P0, dev->tx_addr, dev->addr_width);
        if (err != ESP_OK) {
            return err;
        }
    }

    uint8_t config = 0U;
    err = nrf24_read_reg_byte(dev, NRF_REG_CONFIG, &config);
    if (err != ESP_OK) {
        return err;
    }

    config &= (uint8_t)~NRF_CONFIG_PRIM_RX;
    config |= NRF_CONFIG_PWR_UP;
    err = nrf24_write_reg_byte(dev, NRF_REG_CONFIG, config);
    if (err != ESP_OK) {
        return err;
    }

    err = nrf24_clear_irq_flags(dev, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    if (err == ESP_OK) {
        dev->mode = NRF24_MODE_TX;
    }
    return err;
}

esp_err_t nrf24_set_rx_mode(nrf24_dev_t *dev) {
    if (!nrf24_check_ready(dev)) {
        return ESP_ERR_INVALID_STATE;
    }

    nrf24_set_ce(dev, 0);

    esp_err_t err = nrf24_power_up(dev);
    if (err != ESP_OK) {
        return err;
    }

    /* Restaura endereço RX do pipe 0 se tiver sido configurado */
    if (dev->rx_p0_configured) {
        err = nrf24_write_register(dev, NRF_REG_RX_ADDR_P0, dev->rx_p0_addr, dev->addr_width);
        if (err != ESP_OK) {
            return err;
        }
    }

    uint8_t config = 0U;
    err = nrf24_read_reg_byte(dev, NRF_REG_CONFIG, &config);
    if (err != ESP_OK) {
        return err;
    }

    config |= (NRF_CONFIG_PWR_UP | NRF_CONFIG_PRIM_RX);
    err = nrf24_write_reg_byte(dev, NRF_REG_CONFIG, config);
    if (err != ESP_OK) {
        return err;
    }

    err = nrf24_clear_irq_flags(dev, NRF_STATUS_RX_DR);
    if (err != ESP_OK) {
        return err;
    }

    /* Ativa recepção RF (CE = 1) e aguarda Tstby2a (130 us) */
    nrf24_set_ce(dev, 1);
    esp_rom_delay_us(NRF24_DELAY_STANDBY_TO_RX_US);

    dev->mode = NRF24_MODE_RX;
    return ESP_OK;
}

/* ========================================================================= *
 *  Transmissão e Recepção de Dados                                          *
 * ========================================================================= */

esp_err_t nrf24_send(nrf24_dev_t *dev, const uint8_t *data, size_t len, uint32_t timeout_ms) {
    return nrf24_send_internal(dev, NRF_CMD_W_TX_PAYLOAD, data, len, timeout_ms);
}

esp_err_t nrf24_send_no_ack(nrf24_dev_t *dev, const uint8_t *data, size_t len, uint32_t timeout_ms) {
    return nrf24_send_internal(dev, NRF_CMD_W_TX_PAYLOAD_NOACK, data, len, timeout_ms);
}

bool nrf24_data_available(nrf24_dev_t *dev, uint8_t *pipe_out) {
    if (!nrf24_check_ready(dev)) {
        return false;
    }

    uint8_t status = nrf24_get_status(dev);
    uint8_t pipe   = (uint8_t)((status & NRF_STATUS_RX_P_NO_MASK) >> NRF_STATUS_RX_P_NO_SHIFT);

    if (pipe <= NRF24_MAX_PIPE) {
        if (pipe_out != NULL) {
            *pipe_out = pipe;
        }
        return true;
    }

    return false;
}

esp_err_t nrf24_get_dynamic_payload_size(nrf24_dev_t *dev, uint8_t *size_out) {
    if (!nrf24_check_ready(dev) || (size_out == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t width = 0U;
    esp_err_t err = nrf24_spi_transfer(dev, NRF_CMD_R_RX_PL_WID, NULL, &width, 1U, NULL);
    if (err != ESP_OK) {
        return err;
    }

    /* Datasheet seção 7.3: payload > 32 bytes indica erro de recepção -> flush obrigatório */
    if (width > NRF24_MAX_PAYLOAD_SIZE) {
        ESP_LOGW(TAG, "Payload dinâmico corrompido (%u bytes); executando FLUSH_RX", width);
        (void)nrf24_flush_rx(dev);
        *size_out = 0U;
        return ESP_ERR_INVALID_SIZE;
    }

    *size_out = width;
    return ESP_OK;
}

esp_err_t nrf24_receive(nrf24_dev_t *dev, uint8_t *buffer, size_t max_len, size_t *bytes_read) {
    if (!nrf24_check_ready(dev) || (buffer == NULL) || (max_len == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t to_read = (max_len > NRF24_MAX_PAYLOAD_SIZE) ? NRF24_MAX_PAYLOAD_SIZE : max_len;

    if (dev->dynamic_payloads) {
        uint8_t dyn_len = 0U;
        esp_err_t err = nrf24_get_dynamic_payload_size(dev, &dyn_len);
        if (err != ESP_OK) {
            return err;
        }
        if (dyn_len == 0U) {
            if (bytes_read != NULL) {
                *bytes_read = 0U;
            }
            return ESP_ERR_NOT_FOUND;
        }
        if (dyn_len < to_read) {
            to_read = dyn_len;
        }
    } else {
        uint8_t pipe = 0U;
        if (nrf24_data_available(dev, &pipe) && (dev->rx_pipe_pw[pipe] > 0U) &&
            (dev->rx_pipe_pw[pipe] < to_read)) {
            to_read = dev->rx_pipe_pw[pipe];
        }
    }

    esp_err_t err = nrf24_spi_transfer(dev, NRF_CMD_R_RX_PAYLOAD, NULL, buffer, to_read, NULL);
    if (err != ESP_OK) {
        return err;
    }

    (void)nrf24_clear_irq_flags(dev, NRF_STATUS_RX_DR);
    dev->stats.rx_packets++;

    if (bytes_read != NULL) {
        *bytes_read = to_read;
    }
    return ESP_OK;
}

esp_err_t nrf24_write_ack_payload(nrf24_dev_t *dev, uint8_t pipe, const uint8_t *data, size_t len) {
    if (!nrf24_check_ready(dev) || (pipe > NRF24_MAX_PIPE) || (data == NULL) ||
        (len == 0U) || (len > NRF24_MAX_PAYLOAD_SIZE)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t cmd = (uint8_t)(NRF_CMD_W_ACK_PAYLOAD | (pipe & 0x07U));
    return nrf24_spi_transfer(dev, cmd, data, NULL, len, NULL);
}

/* ========================================================================= *
 *  Status, FIFOs e Diagnóstico                                              *
 * ========================================================================= */

uint8_t nrf24_get_status(nrf24_dev_t *dev) {
    uint8_t status = 0xFFU;
    (void)nrf24_send_command(dev, NRF_CMD_NOP, &status);
    return status;
}

esp_err_t nrf24_clear_irq_flags(nrf24_dev_t *dev, uint8_t flags) {
    return nrf24_write_reg_byte(dev, NRF_REG_STATUS, (uint8_t)(flags & NRF_STATUS_IRQ_ALL));
}

esp_err_t nrf24_flush_tx(nrf24_dev_t *dev) {
    return nrf24_send_command(dev, NRF_CMD_FLUSH_TX, NULL);
}

esp_err_t nrf24_flush_rx(nrf24_dev_t *dev) {
    return nrf24_send_command(dev, NRF_CMD_FLUSH_RX, NULL);
}

bool nrf24_tx_fifo_full(nrf24_dev_t *dev) {
    if (!nrf24_check_ready(dev)) {
        return false;
    }
    return (nrf24_get_status(dev) & NRF_STATUS_TX_FULL) != 0U;
}

bool nrf24_rx_fifo_empty(nrf24_dev_t *dev) {
    if (!nrf24_check_ready(dev)) {
        return true;
    }
    uint8_t fifo = NRF_FIFO_RX_EMPTY;
    if (nrf24_read_reg_byte(dev, NRF_REG_FIFO_STATUS, &fifo) != ESP_OK) {
        return true;
    }
    return (fifo & NRF_FIFO_RX_EMPTY) != 0U;
}

bool nrf24_carrier_detect(nrf24_dev_t *dev) {
    if (!nrf24_check_ready(dev)) {
        return false;
    }
    uint8_t rpd = 0U;
    if (nrf24_read_reg_byte(dev, NRF_REG_RPD, &rpd) != ESP_OK) {
        return false;
    }
    return (rpd & 0x01U) != 0U;
}

esp_err_t nrf24_get_observe_tx(nrf24_dev_t *dev, uint8_t *plos_cnt, uint8_t *arc_cnt) {
    if (!nrf24_check_ready(dev)) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t obs = 0U;
    esp_err_t err = nrf24_read_reg_byte(dev, NRF_REG_OBSERVE_TX, &obs);
    if (err != ESP_OK) {
        return err;
    }
    if (plos_cnt != NULL) {
        *plos_cnt = (uint8_t)((obs >> 4) & 0x0FU);
    }
    if (arc_cnt != NULL) {
        *arc_cnt = (uint8_t)(obs & 0x0FU);
    }
    return ESP_OK;
}

nrf24_stats_t nrf24_get_stats(const nrf24_dev_t *dev) {
    if (dev == NULL) {
        nrf24_stats_t empty = {0};
        return empty;
    }
    return dev->stats;
}

void nrf24_reset_stats(nrf24_dev_t *dev) {
    if (dev != NULL) {
        memset(&dev->stats, 0, sizeof(dev->stats));
    }
}

void nrf24_print_details(nrf24_dev_t *dev) {
    if (!nrf24_check_ready(dev)) {
        ESP_LOGW(TAG, "Dispositivo não inicializado");
        return;
    }

    uint8_t config = 0, en_aa = 0, en_rx = 0, aw = 0, retr = 0;
    uint8_t rf_ch = 0, rf_setup = 0, fifo = 0, dynpd = 0, feature = 0;
    uint8_t tx_addr[NRF24_MAX_ADDR_WIDTH] = {0};
    uint8_t rx_p0[NRF24_MAX_ADDR_WIDTH]   = {0};
    uint8_t status = nrf24_get_status(dev);

    (void)nrf24_read_reg_byte(dev, NRF_REG_CONFIG, &config);
    (void)nrf24_read_reg_byte(dev, NRF_REG_EN_AA, &en_aa);
    (void)nrf24_read_reg_byte(dev, NRF_REG_EN_RXADDR, &en_rx);
    (void)nrf24_read_reg_byte(dev, NRF_REG_SETUP_AW, &aw);
    (void)nrf24_read_reg_byte(dev, NRF_REG_SETUP_RETR, &retr);
    (void)nrf24_read_reg_byte(dev, NRF_REG_RF_CH, &rf_ch);
    (void)nrf24_read_reg_byte(dev, NRF_REG_RF_SETUP, &rf_setup);
    (void)nrf24_read_reg_byte(dev, NRF_REG_FIFO_STATUS, &fifo);
    (void)nrf24_read_reg_byte(dev, NRF_REG_DYNPD, &dynpd);
    (void)nrf24_read_reg_byte(dev, NRF_REG_FEATURE, &feature);
    (void)nrf24_read_register(dev, NRF_REG_TX_ADDR, tx_addr, dev->addr_width);
    (void)nrf24_read_register(dev, NRF_REG_RX_ADDR_P0, rx_p0, dev->addr_width);

    const char *dr_str = "1 Mbps";
    if (rf_setup & NRF_RF_SETUP_RF_DR_LOW) {
        dr_str = "250 kbps";
    } else if (rf_setup & NRF_RF_SETUP_RF_DR_HIGH) {
        dr_str = "2 Mbps";
    }

    static const char *const pa_labels[4] = { "-18 dBm", "-12 dBm", "-6 dBm", "0 dBm" };
    const char *pa_str = pa_labels[(rf_setup & NRF_RF_SETUP_RF_PWR_MASK) >> 1];

    ESP_LOGI(TAG, "================ nRF24L01+ Status ================");
    ESP_LOGI(TAG, "STATUS      = 0x%02X (RX_DR=%u TX_DS=%u MAX_RT=%u RX_P_NO=%u TX_FULL=%u)",
             status,
             (status & NRF_STATUS_RX_DR) ? 1U : 0U,
             (status & NRF_STATUS_TX_DS) ? 1U : 0U,
             (status & NRF_STATUS_MAX_RT) ? 1U : 0U,
             (unsigned)((status & NRF_STATUS_RX_P_NO_MASK) >> NRF_STATUS_RX_P_NO_SHIFT),
             (status & NRF_STATUS_TX_FULL) ? 1U : 0U);
    ESP_LOGI(TAG, "CONFIG      = 0x%02X (PWR_UP=%u PRIM_RX=%u CRC=%s)",
             config,
             (config & NRF_CONFIG_PWR_UP) ? 1U : 0U,
             (config & NRF_CONFIG_PRIM_RX) ? 1U : 0U,
             (config & NRF_CONFIG_EN_CRC) ? ((config & NRF_CONFIG_CRCO) ? "2B" : "1B") : "OFF");
    ESP_LOGI(TAG, "RF_CH       = %u (%u MHz)", rf_ch, 2400U + rf_ch);
    ESP_LOGI(TAG, "RF_SETUP    = 0x%02X (Rate=%s, PA=%s)", rf_setup, dr_str, pa_str);
    ESP_LOGI(TAG, "EN_AA       = 0x%02X | EN_RXADDR = 0x%02X | SETUP_AW = %u bytes",
             en_aa, en_rx, (unsigned)(aw + 2U));
    ESP_LOGI(TAG, "SETUP_RETR  = 0x%02X (ARD=%u us, ARC=%u)",
             retr, (unsigned)((((retr >> 4) & 0x0FU) + 1U) * 250U), (unsigned)(retr & 0x0FU));
    ESP_LOGI(TAG, "FIFO_STATUS = 0x%02X | DYNPD = 0x%02X | FEATURE = 0x%02X",
             fifo, dynpd, feature);
    ESP_LOGI(TAG, "TX_ADDR     = %02X:%02X:%02X:%02X:%02X",
             tx_addr[0], tx_addr[1], tx_addr[2], tx_addr[3], tx_addr[4]);
    ESP_LOGI(TAG, "RX_ADDR_P0  = %02X:%02X:%02X:%02X:%02X",
             rx_p0[0], rx_p0[1], rx_p0[2], rx_p0[3], rx_p0[4]);
    ESP_LOGI(TAG, "Stats       = TX_OK=%lu | MAX_RT=%lu | TIMEOUT=%lu | RX=%lu",
             (unsigned long)dev->stats.tx_ok,
             (unsigned long)dev->stats.tx_max_rt,
             (unsigned long)dev->stats.tx_timeouts,
             (unsigned long)dev->stats.rx_packets);
    ESP_LOGI(TAG, "==================================================");
}
