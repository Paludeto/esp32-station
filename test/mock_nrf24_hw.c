#include "mock_nrf24_hw.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "nrf24l01.h"
#include <string.h>

mock_nrf24_state_t g_mock_nrf;

static int s_dummy_spi_handle = 1;

void mock_nrf24_reset(void) {
    memset(&g_mock_nrf, 0, sizeof(g_mock_nrf));
    /* Valores de reset do Datasheet nRF24L01+ */
    g_mock_nrf.regs[NRF_REG_CONFIG]      = 0x08U;
    g_mock_nrf.regs[NRF_REG_EN_AA]       = 0x3FU;
    g_mock_nrf.regs[NRF_REG_EN_RXADDR]   = 0x03U;
    g_mock_nrf.regs[NRF_REG_SETUP_AW]    = 0x03U;
    g_mock_nrf.regs[NRF_REG_SETUP_RETR]  = 0x03U;
    g_mock_nrf.regs[NRF_REG_RF_CH]       = 0x02U;
    g_mock_nrf.regs[NRF_REG_RF_SETUP]    = 0x0EU;
    g_mock_nrf.regs[NRF_REG_STATUS]      = 0x0EU; /* RX_P_NO = 111 (vazio) */
    g_mock_nrf.regs[NRF_REG_FIFO_STATUS] = 0x11U; /* TX_EMPTY | RX_EMPTY */
    memset(g_mock_nrf.rx_addr_p0, 0xE7, 5);
    memset(g_mock_nrf.rx_addr_p1, 0xC2, 5);
    memset(g_mock_nrf.tx_addr,    0xE7, 5);
    g_mock_nrf.tx_outcome = MOCK_TX_OUTCOME_OK;
}

void mock_nrf24_inject_rx_packet(uint8_t pipe, const uint8_t *data, size_t len) {
    if (len > 32U) len = 32U;
    memcpy(g_mock_nrf.rx_payload, data, len);
    g_mock_nrf.rx_payload_len = len;
    g_mock_nrf.rx_has_packet  = true;
    g_mock_nrf.rx_pipe        = pipe & 0x07U;

    g_mock_nrf.regs[NRF_REG_STATUS] &= (uint8_t)~NRF_STATUS_RX_P_NO_MASK;
    g_mock_nrf.regs[NRF_REG_STATUS] |= (uint8_t)(NRF_STATUS_RX_DR | ((pipe & 0x07U) << 1));
    g_mock_nrf.regs[NRF_REG_FIFO_STATUS] &= (uint8_t)~NRF_FIFO_RX_EMPTY;
}

void esp_rom_delay_us(uint32_t us) {
    g_mock_nrf.now_us += (int64_t)us;
}

int64_t esp_timer_get_time(void) {
    return g_mock_nrf.now_us;
}

void vTaskDelay(TickType_t ticks) {
    g_mock_nrf.now_us += (int64_t)ticks * 1000LL;
}

esp_err_t gpio_config(const gpio_config_t *pGPIOConfig) {
    if (pGPIOConfig == NULL) return ESP_ERR_INVALID_ARG;
    /* Verifica se alguem tentou reconfigurar CSN (GPIO 5 padrao) manualmente */
    if (pGPIOConfig->pin_bit_mask & (1ULL << NRF24_DEFAULT_PIN_CSN)) {
        g_mock_nrf.csn_pin_configured_as_gpio++;
    }
    return ESP_OK;
}

esp_err_t gpio_set_level(gpio_num_t gpio_num, uint32_t level) {
    if (gpio_num == NRF24_DEFAULT_PIN_CE) {
        if ((g_mock_nrf.ce_level == 0) && (level == 1)) {
            g_mock_nrf.ce_pulse_count++;
            /* Se em modo TX (PRIM_RX == 0) e ha payload carregado, simula transmissao RF */
            if (((g_mock_nrf.regs[NRF_REG_CONFIG] & NRF_CONFIG_PRIM_RX) == 0U) &&
                (g_mock_nrf.last_tx_len > 0U)) {
                if (g_mock_nrf.tx_outcome == MOCK_TX_OUTCOME_OK) {
                    g_mock_nrf.regs[NRF_REG_STATUS] |= NRF_STATUS_TX_DS;
                } else if (g_mock_nrf.tx_outcome == MOCK_TX_OUTCOME_MAX_RT) {
                    g_mock_nrf.regs[NRF_REG_STATUS] |= NRF_STATUS_MAX_RT;
                }
            }
        }
        g_mock_nrf.ce_level = (int)level;
    }
    return ESP_OK;
}

esp_err_t spi_bus_initialize(spi_host_device_t host_id, const spi_bus_config_t *bus_config, int dma_chan) {
    (void)host_id; (void)bus_config; (void)dma_chan;
    g_mock_nrf.bus_initialized = true;
    return ESP_OK;
}

esp_err_t spi_bus_free(spi_host_device_t host_id) {
    (void)host_id;
    g_mock_nrf.bus_initialized = false;
    return ESP_OK;
}

esp_err_t spi_bus_add_device(spi_host_device_t host_id, const spi_device_interface_config_t *dev_config, spi_device_handle_t *handle) {
    (void)host_id; (void)dev_config;
    if (handle == NULL) return ESP_ERR_INVALID_ARG;
    g_mock_nrf.device_added = true;
    *handle = &s_dummy_spi_handle;
    return ESP_OK;
}

esp_err_t spi_bus_remove_device(spi_device_handle_t handle) {
    (void)handle;
    g_mock_nrf.device_added = false;
    return ESP_OK;
}

esp_err_t spi_device_transmit(spi_device_handle_t handle, spi_transaction_t *trans_desc) {
    if ((handle == NULL) || (trans_desc == NULL) || (trans_desc->tx_buffer == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    g_mock_nrf.spi_tx_count++;

    const uint8_t *tx = (const uint8_t *)trans_desc->tx_buffer;
    uint8_t *rx       = (uint8_t *)trans_desc->rx_buffer;
    size_t nbytes     = trans_desc->length / 8U;

    if (nbytes == 0U) {
        return ESP_OK;
    }

    if (g_mock_nrf.disconnected) {
        if (rx != NULL) {
            memset(rx, g_mock_nrf.disconnected_byte, nbytes);
        }
        return ESP_OK;
    }

    uint8_t cmd = tx[0];
    if (rx != NULL) {
        memset(rx, 0, nbytes);
        rx[0] = g_mock_nrf.regs[NRF_REG_STATUS];
    }

    size_t data_len = (nbytes > 1U) ? (nbytes - 1U) : 0U;

    if ((cmd & 0xE0U) == NRF_CMD_R_REGISTER) {
        uint8_t reg = cmd & NRF_CMD_REG_MASK;
        if (rx != NULL) {
            if (reg == NRF_REG_RX_ADDR_P0) {
                memcpy(&rx[1], g_mock_nrf.rx_addr_p0, data_len <= 5U ? data_len : 5U);
            } else if (reg == NRF_REG_RX_ADDR_P1) {
                memcpy(&rx[1], g_mock_nrf.rx_addr_p1, data_len <= 5U ? data_len : 5U);
            } else if (reg == NRF_REG_TX_ADDR) {
                memcpy(&rx[1], g_mock_nrf.tx_addr, data_len <= 5U ? data_len : 5U);
            } else if (data_len >= 1U) {
                rx[1] = g_mock_nrf.regs[reg];
            }
        }
    } else if ((cmd & 0xE0U) == NRF_CMD_W_REGISTER) {
        uint8_t reg = cmd & NRF_CMD_REG_MASK;
        if (reg == NRF_REG_STATUS) {
            /* Bits 6..4 sao write-1-to-clear */
            if (data_len >= 1U) {
                g_mock_nrf.regs[NRF_REG_STATUS] &= (uint8_t)~(tx[1] & NRF_STATUS_IRQ_ALL);
            }
        } else if (reg == NRF_REG_RX_ADDR_P0) {
            memcpy(g_mock_nrf.rx_addr_p0, &tx[1], data_len <= 5U ? data_len : 5U);
        } else if (reg == NRF_REG_RX_ADDR_P1) {
            memcpy(g_mock_nrf.rx_addr_p1, &tx[1], data_len <= 5U ? data_len : 5U);
        } else if (reg == NRF_REG_TX_ADDR) {
            memcpy(g_mock_nrf.tx_addr, &tx[1], data_len <= 5U ? data_len : 5U);
        } else if (data_len >= 1U) {
            g_mock_nrf.regs[reg] = tx[1];
        }
    } else if ((cmd == NRF_CMD_W_TX_PAYLOAD) || (cmd == NRF_CMD_W_TX_PAYLOAD_NOACK)) {
        g_mock_nrf.last_tx_cmd = cmd;
        g_mock_nrf.last_tx_len = data_len <= 32U ? data_len : 32U;
        memcpy(g_mock_nrf.last_tx_payload, &tx[1], g_mock_nrf.last_tx_len);
    } else if (cmd == NRF_CMD_R_RX_PAYLOAD) {
        if ((rx != NULL) && g_mock_nrf.rx_has_packet) {
            size_t copy = (data_len < g_mock_nrf.rx_payload_len) ? data_len : g_mock_nrf.rx_payload_len;
            memcpy(&rx[1], g_mock_nrf.rx_payload, copy);
            g_mock_nrf.rx_has_packet = false;
            g_mock_nrf.regs[NRF_REG_STATUS] |= NRF_STATUS_RX_P_NO_MASK;
            g_mock_nrf.regs[NRF_REG_FIFO_STATUS] |= NRF_FIFO_RX_EMPTY;
        }
    } else if (cmd == NRF_CMD_R_RX_PL_WID) {
        if ((rx != NULL) && (data_len >= 1U)) {
            rx[1] = (g_mock_nrf.rx_pl_wid_override != 0U)
                        ? g_mock_nrf.rx_pl_wid_override
                        : (uint8_t)g_mock_nrf.rx_payload_len;
        }
    } else if ((cmd & 0xF8U) == NRF_CMD_W_ACK_PAYLOAD) {
        g_mock_nrf.last_ack_pipe = cmd & 0x07U;
        g_mock_nrf.last_ack_len  = data_len <= 32U ? data_len : 32U;
        memcpy(g_mock_nrf.last_ack_payload, &tx[1], g_mock_nrf.last_ack_len);
    } else if (cmd == NRF_CMD_FLUSH_TX) {
        g_mock_nrf.flush_tx_count++;
        g_mock_nrf.last_tx_len = 0U;
        g_mock_nrf.regs[NRF_REG_STATUS] &= (uint8_t)~NRF_STATUS_TX_FULL;
    } else if (cmd == NRF_CMD_FLUSH_RX) {
        g_mock_nrf.flush_rx_count++;
        g_mock_nrf.rx_has_packet  = false;
        g_mock_nrf.rx_payload_len = 0U;
        g_mock_nrf.regs[NRF_REG_STATUS] |= NRF_STATUS_RX_P_NO_MASK;
        g_mock_nrf.regs[NRF_REG_FIFO_STATUS] |= NRF_FIFO_RX_EMPTY;
    }

    return ESP_OK;
}
