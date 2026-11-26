#ifndef MOCK_NRF24_HW_H
#define MOCK_NRF24_HW_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    MOCK_TX_OUTCOME_OK = 0,
    MOCK_TX_OUTCOME_MAX_RT,
    MOCK_TX_OUTCOME_TIMEOUT,
} mock_tx_outcome_t;

typedef struct {
    /* Registradores internos simulados do nRF24L01+ */
    uint8_t regs[0x20];
    uint8_t rx_addr_p0[5];
    uint8_t rx_addr_p1[5];
    uint8_t tx_addr[5];

    /* FIFOs simulados */
    uint8_t last_tx_payload[32];
    size_t  last_tx_len;
    uint8_t last_tx_cmd;
    uint8_t rx_payload[32];
    size_t  rx_payload_len;
    bool    rx_has_packet;
    uint8_t rx_pipe;
    uint8_t rx_pl_wid_override; /* != 0 sobrescreve retorno de R_RX_PL_WID */

    uint8_t last_ack_payload[32];
    size_t  last_ack_len;
    uint8_t last_ack_pipe;

    /* Estado de GPIO e controle */
    int      ce_level;
    int      ce_pulse_count;
    int      csn_pin_configured_as_gpio;
    int      spi_tx_count;
    int      flush_tx_count;
    int      flush_rx_count;
    int64_t  now_us;

    /* Injeção de comportamento / falhas */
    bool              disconnected;
    uint8_t           disconnected_byte; /* 0x00 ou 0xFF */
    mock_tx_outcome_t tx_outcome;
    bool              bus_initialized;
    bool              device_added;
} mock_nrf24_state_t;

extern mock_nrf24_state_t g_mock_nrf;

void mock_nrf24_reset(void);
void mock_nrf24_inject_rx_packet(uint8_t pipe, const uint8_t *data, size_t len);

#endif /* MOCK_NRF24_HW_H */
