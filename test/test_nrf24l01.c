#include "mock_nrf24_hw.h"
#include "nrf24l01.h"
#include "station_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int s_tests_run = 0;
static int s_tests_failed = 0;

#define EXPECT_TRUE(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "  [FAIL] %s:%d: esperado verdadeiro: %s\n", __FILE__, __LINE__, #cond); \
        s_tests_failed++; \
        return; \
    } \
} while (0)

#define EXPECT_EQ(a, b) do { \
    long long _a = (long long)(a); \
    long long _b = (long long)(b); \
    if (_a != _b) { \
        fprintf(stderr, "  [FAIL] %s:%d: %s (%lld) != %s (%lld)\n", __FILE__, __LINE__, #a, _a, #b, _b); \
        s_tests_failed++; \
        return; \
    } \
} while (0)

#define RUN_TEST(fn) do { \
    mock_nrf24_reset(); \
    int _before = s_tests_failed; \
    s_tests_run++; \
    fn(); \
    if (s_tests_failed == _before) { \
        printf("  [PASS] %s\n", #fn); \
    } \
} while (0)

/* ========================================================================= *
 *  Testes do Driver nRF24L01+                                               *
 * ========================================================================= */

static void test_init_and_deinit(void) {
    nrf24_dev_t dev;
    EXPECT_EQ(nrf24_init(NULL, NULL), ESP_ERR_INVALID_ARG);

    EXPECT_EQ(nrf24_init(&dev, NULL), ESP_OK);
    EXPECT_TRUE(dev.initialized);
    EXPECT_TRUE(g_mock_nrf.bus_initialized);
    EXPECT_TRUE(g_mock_nrf.device_added);
    /* CSN NAO deve ser reconfigurado manualmente como GPIO */
    EXPECT_EQ(g_mock_nrf.csn_pin_configured_as_gpio, 0);
    /* PWR_UP deve estar ligado e Tpd2stby respeitado (>= 5 ms apos power-on) */
    EXPECT_TRUE((g_mock_nrf.regs[NRF_REG_CONFIG] & NRF_CONFIG_PWR_UP) != 0U);
    EXPECT_TRUE(g_mock_nrf.now_us >= 15000LL);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_RF_CH], 2U);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_RF_SETUP], 0x06U); /* 1 Mbps, 0 dBm */

    EXPECT_EQ(nrf24_deinit(&dev), ESP_OK);
    EXPECT_TRUE(!dev.initialized);
    EXPECT_TRUE((g_mock_nrf.regs[NRF_REG_CONFIG] & NRF_CONFIG_PWR_UP) == 0U);
}

static void test_is_connected_detection(void) {
    nrf24_dev_t dev;
    EXPECT_EQ(nrf24_init(&dev, NULL), ESP_OK);
    EXPECT_EQ(nrf24_set_channel(&dev, 76U), ESP_OK);

    EXPECT_TRUE(nrf24_is_connected(&dev));
    /* Deve restaurar o canal original apos o teste */
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_RF_CH], 76U);

    /* Simula modulo desconectado (MISO preso em 0xFF ou 0x00) */
    g_mock_nrf.disconnected = true;
    g_mock_nrf.disconnected_byte = 0xFFU;
    EXPECT_TRUE(!nrf24_is_connected(&dev));

    g_mock_nrf.disconnected_byte = 0x00U;
    EXPECT_TRUE(!nrf24_is_connected(&dev));
}

static void test_channel_and_rf_setup_encoding(void) {
    nrf24_dev_t dev;
    EXPECT_EQ(nrf24_init(&dev, NULL), ESP_OK);

    EXPECT_EQ(nrf24_set_channel(&dev, 125U), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_RF_CH], 125U);
    EXPECT_EQ(nrf24_set_channel(&dev, 126U), ESP_ERR_INVALID_ARG);

    EXPECT_EQ(nrf24_set_rf_setup(&dev, NRF24_DR_250KBPS, NRF24_PA_MIN), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_RF_SETUP], 0x20U);

    EXPECT_EQ(nrf24_set_rf_setup(&dev, NRF24_DR_1MBPS, NRF24_PA_HIGH), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_RF_SETUP], 0x04U);

    EXPECT_EQ(nrf24_set_rf_setup(&dev, NRF24_DR_2MBPS, NRF24_PA_MAX), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_RF_SETUP], 0x0EU);
}

static void test_crc_addr_width_and_retries(void) {
    nrf24_dev_t dev;
    EXPECT_EQ(nrf24_init(&dev, NULL), ESP_OK);

    EXPECT_EQ(nrf24_set_crc(&dev, NRF24_CRC_DISABLED), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_CONFIG] & (NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO), 0U);

    EXPECT_EQ(nrf24_set_crc(&dev, NRF24_CRC_1BYTE), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_CONFIG] & (NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO), NRF_CONFIG_EN_CRC);

    EXPECT_EQ(nrf24_set_crc(&dev, NRF24_CRC_2BYTES), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_CONFIG] & (NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO),
              NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO);

    EXPECT_EQ(nrf24_set_address_width(&dev, NRF24_ADDR_WIDTH_3), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_SETUP_AW], 0x01U);
    EXPECT_EQ(nrf24_set_address_width(&dev, NRF24_ADDR_WIDTH_5), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_SETUP_AW], 0x03U);

    EXPECT_EQ(nrf24_set_retries(&dev, 10U, 7U), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_SETUP_RETR], 0xA7U);
    EXPECT_EQ(nrf24_set_retries(&dev, 16U, 5U), ESP_ERR_INVALID_ARG);
}

static void test_pipes_and_tx_rx_address_restoration(void) {
    nrf24_dev_t dev;
    EXPECT_EQ(nrf24_init(&dev, NULL), ESP_OK);

    const uint8_t tx_addr[5] = { 0x11, 0x22, 0x33, 0x44, 0x55 };
    const uint8_t rx_p0[5]   = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE };
    const uint8_t rx_p2      = 0x77;

    EXPECT_EQ(nrf24_set_tx_address(&dev, tx_addr), ESP_OK);
    EXPECT_EQ(memcmp(g_mock_nrf.tx_addr, tx_addr, 5), 0);
    EXPECT_EQ(memcmp(g_mock_nrf.rx_addr_p0, tx_addr, 5), 0);

    EXPECT_EQ(nrf24_open_rx_pipe(&dev, 0U, rx_p0, 8U), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_RX_PW_P0], 8U);
    EXPECT_TRUE((g_mock_nrf.regs[NRF_REG_EN_RXADDR] & (1U << 0)) != 0U);

    /* Pipe 2 usa apenas 1 byte LSB */
    EXPECT_EQ(nrf24_open_rx_pipe(&dev, 2U, &rx_p2, 16U), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_RX_ADDR_P2], 0x77U);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_RX_PW_P2], 16U);
    EXPECT_TRUE((g_mock_nrf.regs[NRF_REG_EN_RXADDR] & (1U << 2)) != 0U);

    /* Troca TX <-> RX deve preservar RX_ADDR_P0 corretamente em cada modo */
    EXPECT_EQ(nrf24_set_tx_mode(&dev), ESP_OK);
    EXPECT_EQ(g_mock_nrf.ce_level, 0);
    EXPECT_EQ(memcmp(g_mock_nrf.rx_addr_p0, tx_addr, 5), 0);

    EXPECT_EQ(nrf24_set_rx_mode(&dev), ESP_OK);
    EXPECT_EQ(g_mock_nrf.ce_level, 1);
    EXPECT_TRUE((g_mock_nrf.regs[NRF_REG_CONFIG] & NRF_CONFIG_PRIM_RX) != 0U);
    EXPECT_EQ(memcmp(g_mock_nrf.rx_addr_p0, rx_p0, 5), 0);

    EXPECT_EQ(nrf24_close_rx_pipe(&dev, 2U), ESP_OK);
    EXPECT_TRUE((g_mock_nrf.regs[NRF_REG_EN_RXADDR] & (1U << 2)) == 0U);
}

static void test_send_ok_max_rt_and_timeout(void) {
    nrf24_dev_t dev;
    EXPECT_EQ(nrf24_init(&dev, NULL), ESP_OK);
    EXPECT_EQ(nrf24_set_tx_mode(&dev), ESP_OK);

    const uint8_t payload[8] = { 0xA5, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };

    /* 1. Envio bem-sucedido (TX_DS) */
    g_mock_nrf.tx_outcome = MOCK_TX_OUTCOME_OK;
    EXPECT_EQ(nrf24_send(&dev, payload, sizeof(payload), 20U), ESP_OK);
    EXPECT_EQ(g_mock_nrf.last_tx_cmd, NRF_CMD_W_TX_PAYLOAD);
    EXPECT_EQ(g_mock_nrf.last_tx_len, 8U);
    EXPECT_EQ(memcmp(g_mock_nrf.last_tx_payload, payload, 8U), 0);
    /* TX_DS deve ter sido limpo apos confirmacao */
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_STATUS] & NRF_STATUS_TX_DS, 0U);
    EXPECT_EQ(dev.stats.tx_ok, 1U);

    /* 2. Falha por MAX_RT */
    g_mock_nrf.tx_outcome = MOCK_TX_OUTCOME_MAX_RT;
    int flushes_before = g_mock_nrf.flush_tx_count;
    EXPECT_EQ(nrf24_send(&dev, payload, sizeof(payload), 20U), ESP_FAIL);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_STATUS] & NRF_STATUS_MAX_RT, 0U);
    EXPECT_EQ(g_mock_nrf.flush_tx_count, flushes_before + 1);
    EXPECT_EQ(dev.stats.tx_max_rt, 1U);

    /* 3. Timeout */
    g_mock_nrf.tx_outcome = MOCK_TX_OUTCOME_TIMEOUT;
    EXPECT_EQ(nrf24_send(&dev, payload, sizeof(payload), 5U), ESP_ERR_TIMEOUT);
    EXPECT_EQ(dev.stats.tx_timeouts, 1U);

    /* 4. Argumentos invalidos */
    EXPECT_EQ(nrf24_send(&dev, payload, 0U, 10U), ESP_ERR_INVALID_ARG);
    EXPECT_EQ(nrf24_send(&dev, payload, 33U, 10U), ESP_ERR_INVALID_ARG);
    EXPECT_EQ(nrf24_send(&dev, NULL, 4U, 10U), ESP_ERR_INVALID_ARG);
}

static void test_receive_multibyte_and_dynamic_payloads(void) {
    nrf24_dev_t dev;
    nrf24_config_t cfg = NRF24_CONFIG_DEFAULT();
    cfg.payload_size = 16U;
    EXPECT_EQ(nrf24_init(&dev, &cfg), ESP_OK);

    const uint8_t addr[5] = { 0, 0, 0, 0, 1 };
    EXPECT_EQ(nrf24_open_rx_pipe(&dev, 0U, addr, 16U), ESP_OK);
    EXPECT_EQ(nrf24_set_rx_mode(&dev), ESP_OK);

    EXPECT_TRUE(!nrf24_data_available(&dev, NULL));

    /* Injeta 16 bytes e verifica leitura completa (corrige bug legado .length = 8) */
    uint8_t expected[16];
    for (size_t i = 0; i < sizeof(expected); i++) expected[i] = (uint8_t)(0x30U + i);
    mock_nrf24_inject_rx_packet(0U, expected, sizeof(expected));

    uint8_t pipe = 0xFFU;
    EXPECT_TRUE(nrf24_data_available(&dev, &pipe));
    EXPECT_EQ(pipe, 0U);

    uint8_t rx_buf[32] = {0};
    size_t read_len = 0U;
    EXPECT_EQ(nrf24_receive(&dev, rx_buf, sizeof(rx_buf), &read_len), ESP_OK);
    EXPECT_EQ(read_len, 16U);
    EXPECT_EQ(memcmp(rx_buf, expected, 16U), 0);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_STATUS] & NRF_STATUS_RX_DR, 0U);
    EXPECT_EQ(dev.stats.rx_packets, 1U);

    /* Testa modo de payload dinamico + protecao contra largura > 32 bytes */
    EXPECT_EQ(nrf24_enable_dynamic_payloads(&dev, true), ESP_OK);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_FEATURE] & NRF_FEATURE_EN_DPL, NRF_FEATURE_EN_DPL);
    EXPECT_EQ(g_mock_nrf.regs[NRF_REG_DYNPD], 0x3FU);

    const uint8_t dyn_msg[5] = { 'E', 'S', 'P', '3', '2' };
    mock_nrf24_inject_rx_packet(1U, dyn_msg, sizeof(dyn_msg));
    EXPECT_EQ(nrf24_receive(&dev, rx_buf, sizeof(rx_buf), &read_len), ESP_OK);
    EXPECT_EQ(read_len, 5U);
    EXPECT_EQ(memcmp(rx_buf, dyn_msg, 5U), 0);

    /* Largura corrompida (> 32) deve disparar FLUSH_RX automatico */
    g_mock_nrf.rx_pl_wid_override = 45U;
    int rx_flushes = g_mock_nrf.flush_rx_count;
    EXPECT_EQ(nrf24_receive(&dev, rx_buf, sizeof(rx_buf), &read_len), ESP_ERR_INVALID_SIZE);
    EXPECT_EQ(g_mock_nrf.flush_rx_count, rx_flushes + 1);
}

static void test_ack_payload_and_diagnostics(void) {
    nrf24_dev_t dev;
    EXPECT_EQ(nrf24_init(&dev, NULL), ESP_OK);

    EXPECT_EQ(nrf24_enable_ack_payload(&dev, true), ESP_OK);
    EXPECT_TRUE((g_mock_nrf.regs[NRF_REG_FEATURE] & (NRF_FEATURE_EN_ACK_PAY | NRF_FEATURE_EN_DPL)) ==
                (NRF_FEATURE_EN_ACK_PAY | NRF_FEATURE_EN_DPL));

    const uint8_t ack_data[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
    EXPECT_EQ(nrf24_write_ack_payload(&dev, 0U, ack_data, sizeof(ack_data)), ESP_OK);
    EXPECT_EQ(g_mock_nrf.last_ack_pipe, 0U);
    EXPECT_EQ(g_mock_nrf.last_ack_len, 4U);
    EXPECT_EQ(memcmp(g_mock_nrf.last_ack_payload, ack_data, 4U), 0);

    /* Carrier detect e OBSERVE_TX */
    g_mock_nrf.regs[NRF_REG_RPD] = 0x01U;
    EXPECT_TRUE(nrf24_carrier_detect(&dev));
    g_mock_nrf.regs[NRF_REG_RPD] = 0x00U;
    EXPECT_TRUE(!nrf24_carrier_detect(&dev));

    g_mock_nrf.regs[NRF_REG_OBSERVE_TX] = 0x3AU; /* PLOS=3, ARC=10 */
    uint8_t plos = 0, arc = 0;
    EXPECT_EQ(nrf24_get_observe_tx(&dev, &plos, &arc), ESP_OK);
    EXPECT_EQ(plos, 3U);
    EXPECT_EQ(arc, 10U);

    /* TX FIFO full via STATUS bit 0 */
    g_mock_nrf.regs[NRF_REG_STATUS] |= NRF_STATUS_TX_FULL;
    EXPECT_TRUE(nrf24_tx_fifo_full(&dev));
    g_mock_nrf.regs[NRF_REG_STATUS] &= (uint8_t)~NRF_STATUS_TX_FULL;
    EXPECT_TRUE(!nrf24_tx_fifo_full(&dev));
}

/* ========================================================================= *
 *  Testes do Protocolo da Estacao (CRC-8 e Enquadramento)                   *
 * ========================================================================= */

static void test_station_protocol_packet_and_crc8(void) {
    station_packet_t pkt;
    station_packet_build(&pkt, STATION_CMD_FORWARD, 200U, 42U, 0x01U);

    EXPECT_EQ(sizeof(pkt), 8U);
    EXPECT_EQ(pkt.magic, STATION_PACKET_MAGIC);
    EXPECT_EQ(pkt.version, STATION_PROTOCOL_VERSION);
    EXPECT_EQ(pkt.cmd, STATION_CMD_FORWARD);
    EXPECT_EQ(pkt.speed, 200U);
    EXPECT_EQ(pkt.seq, 42U);
    EXPECT_TRUE(station_packet_validate(&pkt));

    /* Corrompe 1 bit no payload -> CRC-8 deve detectar */
    pkt.speed ^= 0x04U;
    EXPECT_TRUE(!station_packet_validate(&pkt));

    /* Restaura e corrompe magic */
    pkt.speed ^= 0x04U;
    EXPECT_TRUE(station_packet_validate(&pkt));
    pkt.magic = 0x00U;
    EXPECT_TRUE(!station_packet_validate(&pkt));

    EXPECT_EQ(strcmp(station_cmd_name(STATION_CMD_STOP), "STOP"), 0);
    EXPECT_EQ(strcmp(station_cmd_name(STATION_CMD_PING), "PING"), 0);
    EXPECT_EQ(strcmp(station_cmd_name(0xFEU), "UNKNOWN"), 0);
}

int main(void) {
    printf("=== Executando Suite de Testes Unitarios (esp32-station / nRF24L01+) ===\n");

    RUN_TEST(test_init_and_deinit);
    RUN_TEST(test_is_connected_detection);
    RUN_TEST(test_channel_and_rf_setup_encoding);
    RUN_TEST(test_crc_addr_width_and_retries);
    RUN_TEST(test_pipes_and_tx_rx_address_restoration);
    RUN_TEST(test_send_ok_max_rt_and_timeout);
    RUN_TEST(test_receive_multibyte_and_dynamic_payloads);
    RUN_TEST(test_ack_payload_and_diagnostics);
    RUN_TEST(test_station_protocol_packet_and_crc8);

    printf("========================================================================\n");
    printf("Resultado: %d testes executados, %d falhas.\n", s_tests_run, s_tests_failed);
    return (s_tests_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
