#ifndef MOCK_DRIVER_UART_H
#define MOCK_DRIVER_UART_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

typedef int uart_port_t;
#define UART_NUM_0 0

typedef enum { UART_DATA_8_BITS = 3 } uart_word_length_t;
typedef enum { UART_PARITY_DISABLE = 0 } uart_parity_t;
typedef enum { UART_STOP_BITS_1 = 1 } uart_stop_bits_t;
typedef enum { UART_HW_FLOWCTRL_DISABLE = 0 } uart_hw_flowcontrol_t;
typedef enum { UART_SCLK_DEFAULT = 0 } uart_sclk_t;

typedef struct {
    int                   baud_rate;
    uart_word_length_t    data_bits;
    uart_parity_t         parity;
    uart_stop_bits_t      stop_bits;
    uart_hw_flowcontrol_t flow_ctrl;
    uart_sclk_t           source_clk;
} uart_config_t;

static inline esp_err_t uart_param_config(uart_port_t port, const uart_config_t *cfg) {
    (void)port; (void)cfg; return ESP_OK;
}
static inline esp_err_t uart_driver_install(uart_port_t port, int rx_buf, int tx_buf, int q_sz, void *q, int flags) {
    (void)port; (void)rx_buf; (void)tx_buf; (void)q_sz; (void)q; (void)flags; return ESP_OK;
}
static inline int uart_read_bytes(uart_port_t port, void *buf, uint32_t len, uint32_t ticks) {
    (void)port; (void)buf; (void)len; (void)ticks; return 0;
}

#endif /* MOCK_DRIVER_UART_H */
