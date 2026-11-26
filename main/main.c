#include "nrf24l01.h"
#include "station_protocol.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

/* ========================================================================= *
 *  Fallbacks de Configuração (quando sdkconfig ainda não foi gerado)        *
 * ========================================================================= */
#ifndef CONFIG_STATION_PIN_CE
#define CONFIG_STATION_PIN_CE          4
#endif
#ifndef CONFIG_STATION_PIN_CSN
#define CONFIG_STATION_PIN_CSN         5
#endif
#ifndef CONFIG_STATION_PIN_MISO
#define CONFIG_STATION_PIN_MISO        19
#endif
#ifndef CONFIG_STATION_PIN_MOSI
#define CONFIG_STATION_PIN_MOSI        23
#endif
#ifndef CONFIG_STATION_PIN_SCK
#define CONFIG_STATION_PIN_SCK         18
#endif
#ifndef CONFIG_STATION_PIN_IRQ
#define CONFIG_STATION_PIN_IRQ         -1
#endif
#ifndef CONFIG_STATION_RF_CHANNEL
#define CONFIG_STATION_RF_CHANNEL      2
#endif
#ifndef CONFIG_STATION_TX_INTERVAL_MS
#define CONFIG_STATION_TX_INTERVAL_MS  250
#endif

#if !defined(CONFIG_STATION_MODE_TX_INTERACTIVE) && \
    !defined(CONFIG_STATION_MODE_TX_BEACON) && \
    !defined(CONFIG_STATION_MODE_RX_MONITOR)
#define CONFIG_STATION_MODE_TX_INTERACTIVE 1
#endif

#ifndef CONFIG_STATION_AUTO_ACK
#define CONFIG_STATION_AUTO_ACK 1
#endif

static const char *TAG = "STATION";

/* Endereço de 5 bytes compartilhado entre estação (TX) e nó remoto (RX) */
static const uint8_t STATION_PIPE_ADDR[NRF24_MAX_ADDR_WIDTH] = { 0x00, 0x00, 0x00, 0x00, 0x01 };

#define STATION_TX_TIMEOUT_MS  50U
#define STATION_UART_PORT      UART_NUM_0
#define STATION_UART_BUF_SIZE  256

/* ========================================================================= *
 *  Inicialização do Rádio                                                   *
 * ========================================================================= */

static nrf24_data_rate_t station_configured_data_rate(void) {
#if defined(CONFIG_STATION_DR_250KBPS)
    return NRF24_DR_250KBPS;
#elif defined(CONFIG_STATION_DR_2MBPS)
    return NRF24_DR_2MBPS;
#else
    return NRF24_DR_1MBPS;
#endif
}

static nrf24_pa_level_t station_configured_pa_level(void) {
#if defined(CONFIG_STATION_PA_MIN)
    return NRF24_PA_MIN;
#elif defined(CONFIG_STATION_PA_LOW)
    return NRF24_PA_LOW;
#elif defined(CONFIG_STATION_PA_HIGH)
    return NRF24_PA_HIGH;
#else
    return NRF24_PA_MAX;
#endif
}

static esp_err_t station_radio_init(nrf24_dev_t *radio) {
    nrf24_config_t cfg = NRF24_CONFIG_DEFAULT();
    cfg.pin_ce       = (gpio_num_t)CONFIG_STATION_PIN_CE;
    cfg.pin_csn      = (gpio_num_t)CONFIG_STATION_PIN_CSN;
    cfg.pin_miso     = (gpio_num_t)CONFIG_STATION_PIN_MISO;
    cfg.pin_mosi     = (gpio_num_t)CONFIG_STATION_PIN_MOSI;
    cfg.pin_sck      = (gpio_num_t)CONFIG_STATION_PIN_SCK;
    cfg.pin_irq      = (CONFIG_STATION_PIN_IRQ >= 0) ? (gpio_num_t)CONFIG_STATION_PIN_IRQ : GPIO_NUM_NC;
    cfg.channel      = (uint8_t)CONFIG_STATION_RF_CHANNEL;
    cfg.data_rate    = station_configured_data_rate();
    cfg.pa_level     = station_configured_pa_level();
    cfg.payload_size = (uint8_t)sizeof(station_packet_t);
    cfg.auto_ack     = (CONFIG_STATION_AUTO_ACK != 0);

    esp_err_t err = nrf24_init(radio, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao inicializar driver nRF24L01+: %s", esp_err_to_name(err));
        return err;
    }

    if (!nrf24_is_connected(radio)) {
        ESP_LOGE(TAG, "Modulo nRF24L01+ NAO detectado no barramento SPI!");
        ESP_LOGE(TAG, "Verifique: 3.3V estavel (+ capacitor 10uF), GND, MISO=%d, MOSI=%d, SCK=%d, CSN=%d",
                 CONFIG_STATION_PIN_MISO, CONFIG_STATION_PIN_MOSI,
                 CONFIG_STATION_PIN_SCK, CONFIG_STATION_PIN_CSN);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_ERROR_CHECK(nrf24_set_tx_address(radio, STATION_PIPE_ADDR));
    ESP_ERROR_CHECK(nrf24_open_rx_pipe(radio, 0U, STATION_PIPE_ADDR, (uint8_t)sizeof(station_packet_t)));
    ESP_ERROR_CHECK(nrf24_enable_auto_ack(radio, 0U, cfg.auto_ack));

    return ESP_OK;
}

/* ========================================================================= *
 *  Scanner de Espectro 2.4 GHz (Usa RPD do nRF24L01+)                       *
 * ========================================================================= */

#if defined(CONFIG_STATION_MODE_TX_INTERACTIVE)
static void station_scan_rf_channels(nrf24_dev_t *radio) {
    const uint8_t original_channel = radio->cfg.channel;
    const nrf24_mode_t original_mode = radio->mode;
    uint8_t hits[NRF24_MAX_CHANNEL + 1U];
    memset(hits, 0, sizeof(hits));

    const uint8_t samples_per_ch = 20U;
    ESP_LOGI(TAG, "Escaneando canais 0..%u (2400..2525 MHz, %u amostras/canal)...",
             NRF24_MAX_CHANNEL, samples_per_ch);

    for (uint8_t s = 0; s < samples_per_ch; s++) {
        for (uint8_t ch = 0; ch <= NRF24_MAX_CHANNEL; ch++) {
            (void)nrf24_set_channel(radio, ch);
            (void)nrf24_set_rx_mode(radio);
            esp_rom_delay_us(170); /* Tempo minimo para integracao do RPD */
            if (nrf24_carrier_detect(radio)) {
                hits[ch]++;
            }
            gpio_set_level(radio->cfg.pin_ce, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    /* Restaura canal e modo anteriores */
    (void)nrf24_set_channel(radio, original_channel);
    if (original_mode == NRF24_MODE_RX) {
        (void)nrf24_set_rx_mode(radio);
    } else {
        (void)nrf24_set_tx_mode(radio);
    }

    uint8_t best_ch = 76U;
    uint8_t min_hits = 255U;

    printf("\n--- Mapa de Ocupacao RF 2.4 GHz (0=livre, 9=ocupado) ---\n");
    for (uint8_t ch = 0; ch <= NRF24_MAX_CHANNEL; ch++) {
        if ((ch % 25U) == 0U) {
            printf("\nCH %3u (%4u MHz): ", ch, (unsigned)(2400U + ch));
        }
        uint8_t level = (uint8_t)((hits[ch] * 9U) / samples_per_ch);
        printf("%u", level);

        /* Prioriza canais fora da banda Wi-Fi principal (CH >= 76) */
        if ((ch >= 76U) && (hits[ch] < min_hits)) {
            min_hits = hits[ch];
            best_ch = ch;
        }
    }
    printf("\n\nCanal livre sugerido (acima do Wi-Fi): CH %u (%u MHz, ruido=%u/%u)\n\n",
           best_ch, (unsigned)(2400U + best_ch), min_hits, samples_per_ch);
}

static void station_print_help(void) {
    printf("\n================ ESP32 Station - Comandos ================\n");
    printf("  W / S / A / D : Frente / Re / Esquerda / Direita\n");
    printf("  X             : Parar (STOP)\n");
    printf("  + / -         : Aumentar / Diminuir velocidade (PWM)\n");
    printf("  P             : Ping RF (mede latencia TX + ACK em us)\n");
    printf("  I             : Exibir registradores e estatisticas\n");
    printf("  C             : Escanear espectro 2.4 GHz (canais 0..125)\n");
    printf("  H / ?         : Exibir esta ajuda\n");
    printf("==========================================================\n\n");
}

static void station_run_tx_interactive(nrf24_dev_t *radio) {
    ESP_ERROR_CHECK(nrf24_set_tx_mode(radio));
    nrf24_print_details(radio);

    /* Instala driver UART0 para leitura nao-bloqueante do console */
    const uart_config_t uart_cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    (void)uart_param_config(STATION_UART_PORT, &uart_cfg);
    (void)uart_driver_install(STATION_UART_PORT, STATION_UART_BUF_SIZE, 0, 0, NULL, 0);

    uint16_t seq = 0U;
    uint8_t speed = 180U; /* ~70% */
    station_print_help();

    while (true) {
        uint8_t ch = 0U;
        int len = uart_read_bytes(STATION_UART_PORT, &ch, 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            continue;
        }

        if ((ch == '\r') || (ch == '\n') || (ch == ' ')) {
            continue;
        }

        station_cmd_t cmd = STATION_CMD_STOP;
        bool should_send = false;
        bool is_ping = false;

        switch (ch) {
            case 'w': case 'W': cmd = STATION_CMD_FORWARD;  should_send = true; break;
            case 's': case 'S': cmd = STATION_CMD_BACKWARD; should_send = true; break;
            case 'a': case 'A': cmd = STATION_CMD_LEFT;     should_send = true; break;
            case 'd': case 'D': cmd = STATION_CMD_RIGHT;    should_send = true; break;
            case 'x': case 'X': cmd = STATION_CMD_STOP;     should_send = true; break;
            case 'p': case 'P': cmd = STATION_CMD_PING;     should_send = true; is_ping = true; break;
            case '+':
                speed = (speed <= 230U) ? (uint8_t)(speed + 25U) : 255U;
                ESP_LOGI(TAG, "Velocidade ajustada: %u/255 (%u%%)", speed, (unsigned)((speed * 100U) / 255U));
                break;
            case '-':
                speed = (speed >= 25U) ? (uint8_t)(speed - 25U) : 0U;
                ESP_LOGI(TAG, "Velocidade ajustada: %u/255 (%u%%)", speed, (unsigned)((speed * 100U) / 255U));
                break;
            case 'i': case 'I': {
                uint8_t plos = 0U, arc = 0U;
                (void)nrf24_get_observe_tx(radio, &plos, &arc);
                nrf24_print_details(radio);
                ESP_LOGI(TAG, "OBSERVE_TX  = PLOS=%u | ARC=%u", plos, arc);
                break;
            }
            case 'c': case 'C':
                station_scan_rf_channels(radio);
                break;
            case 'h': case 'H': case '?':
                station_print_help();
                break;
            default:
                ESP_LOGW(TAG, "Tecla '%c' (0x%02X) nao reconhecida. Pressione 'H' para ajuda.", ch, ch);
                break;
        }

        if (should_send) {
            station_packet_t pkt;
            station_packet_build(&pkt, cmd, (cmd == STATION_CMD_STOP) ? 0U : speed, seq++, 0U);

            int64_t t0 = esp_timer_get_time();
            esp_err_t res = nrf24_send(radio, (const uint8_t *)&pkt, sizeof(pkt), STATION_TX_TIMEOUT_MS);
            int64_t elapsed_us = esp_timer_get_time() - t0;

            if (res == ESP_OK) {
                uint8_t plos = 0U, arc = 0U;
                (void)nrf24_get_observe_tx(radio, &plos, &arc);
                if (is_ping) {
                    ESP_LOGI(TAG, "PONG seq=%u | RTT=%lld us | retries=%u", pkt.seq, (long long)elapsed_us, arc);
                } else {
                    ESP_LOGI(TAG, "TX OK -> %-8s | seq=%u | speed=%u | %lld us (arc=%u)",
                             station_cmd_name(pkt.cmd), pkt.seq, pkt.speed, (long long)elapsed_us, arc);
                }
            } else {
                ESP_LOGW(TAG, "TX FALHOU (%s) -> %-8s | seq=%u",
                         esp_err_to_name(res), station_cmd_name(pkt.cmd), pkt.seq);
            }
        }
    }
}
#endif /* CONFIG_STATION_MODE_TX_INTERACTIVE */

/* ========================================================================= *
 *  Modo TX Beacon Periodico                                                 *
 * ========================================================================= */

#if defined(CONFIG_STATION_MODE_TX_BEACON)
static void station_run_tx_beacon(nrf24_dev_t *radio) {
    ESP_ERROR_CHECK(nrf24_set_tx_mode(radio));
    nrf24_print_details(radio);

    uint16_t seq = 0U;
    ESP_LOGI(TAG, "Modo TX Beacon iniciado (intervalo = %d ms)", CONFIG_STATION_TX_INTERVAL_MS);

    while (true) {
        station_packet_t pkt;
        station_packet_build(&pkt, STATION_CMD_BEACON, 0U, seq++, 0U);

        esp_err_t res = nrf24_send(radio, (const uint8_t *)&pkt, sizeof(pkt), STATION_TX_TIMEOUT_MS);
        nrf24_stats_t st = nrf24_get_stats(radio);
        uint32_t total = st.tx_ok + st.tx_max_rt + st.tx_timeouts;
        uint32_t pct = (total > 0U) ? ((st.tx_ok * 100U) / total) : 0U;

        if (res == ESP_OK) {
            ESP_LOGI(TAG, "BEACON #%u enviado (entrega: %lu%% | ok=%lu falhas=%lu)",
                     pkt.seq, (unsigned long)pct, (unsigned long)st.tx_ok,
                     (unsigned long)(st.tx_max_rt + st.tx_timeouts));
        } else {
            ESP_LOGW(TAG, "BEACON #%u falhou: %s (entrega: %lu%%)",
                     pkt.seq, esp_err_to_name(res), (unsigned long)pct);
        }

        vTaskDelay(pdMS_TO_TICKS(CONFIG_STATION_TX_INTERVAL_MS));
    }
}
#endif /* CONFIG_STATION_MODE_TX_BEACON */

/* ========================================================================= *
 *  Modo RX Receptor / Monitor                                               *
 * ========================================================================= */

#if defined(CONFIG_STATION_MODE_RX_MONITOR)
static void station_run_rx_monitor(nrf24_dev_t *radio) {
    ESP_ERROR_CHECK(nrf24_set_rx_mode(radio));
    nrf24_print_details(radio);

    bool has_last_seq = false;
    uint16_t last_seq = 0U;
    uint32_t crc_errors = 0U;
    uint32_t lost_packets = 0U;

    ESP_LOGI(TAG, "Modo RX Monitor ativo — aguardando pacotes no canal %u (%u MHz)...",
             radio->cfg.channel, (unsigned)(2400U + radio->cfg.channel));

    while (true) {
        uint8_t pipe = 0U;
        while (nrf24_data_available(radio, &pipe)) {
            uint8_t buf[NRF24_MAX_PAYLOAD_SIZE];
            size_t read_len = 0U;

            esp_err_t err = nrf24_receive(radio, buf, sizeof(buf), &read_len);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Erro ao ler payload RX: %s", esp_err_to_name(err));
                continue;
            }

            if (read_len == sizeof(station_packet_t)) {
                station_packet_t pkt;
                memcpy(&pkt, buf, sizeof(pkt));

                if (station_packet_validate(&pkt)) {
                    if (has_last_seq) {
                        uint16_t expected = (uint16_t)(last_seq + 1U);
                        if (pkt.seq != expected) {
                            uint16_t gap = (uint16_t)(pkt.seq - expected);
                            lost_packets += gap;
                        }
                    }
                    last_seq = pkt.seq;
                    has_last_seq = true;

                    ESP_LOGI(TAG, "RX [pipe=%u] %-8s | seq=%u | speed=%u | perdidos=%lu | crc_err=%lu",
                             pipe, station_cmd_name(pkt.cmd), pkt.seq, pkt.speed,
                             (unsigned long)lost_packets, (unsigned long)crc_errors);
                } else {
                    crc_errors++;
                    ESP_LOGW(TAG, "RX [pipe=%u] Pacote invalido/CRC corrompido (total=%lu)",
                             pipe, (unsigned long)crc_errors);
                }
            } else {
                ESP_LOGI(TAG, "RX [pipe=%u] Payload bruto (%u bytes): first=0x%02X",
                         pipe, (unsigned)read_len, buf[0]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif /* CONFIG_STATION_MODE_RX_MONITOR */

/* ========================================================================= *
 *  Ponto de Entrada (ESP-IDF)                                               *
 * ========================================================================= */

void app_main(void) {
    ESP_LOGI(TAG, "Iniciando ESP32 Station (nRF24L01+)...");

    nrf24_dev_t radio;
    esp_err_t err = station_radio_init(&radio);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Inicializacao abortada (%s). Corrija o hardware e reinicie.", esp_err_to_name(err));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

#if defined(CONFIG_STATION_MODE_RX_MONITOR)
    station_run_rx_monitor(&radio);
#elif defined(CONFIG_STATION_MODE_TX_BEACON)
    station_run_tx_beacon(&radio);
#else
    station_run_tx_interactive(&radio);
#endif
}
