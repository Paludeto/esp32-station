# 📘 Referência da API — Componente `nrf24l01`

> **🇧🇷 Português** · [🇺🇸 English](en/API.md)

Esta documentação detalha a interface pública do componente [`components/nrf24l01`](../components/nrf24l01/include/nrf24l01.h) para ESP-IDF.

---

## 1. Tipos e Enumerações

### `nrf24_data_rate_t`
| Constante | Taxa de Ar | Observação |
| :--- | :---: | :--- |
| `NRF24_DR_250KBPS` | 250 kbps | Maior sensibilidade/alcance (exclusivo nRF24L01+) |
| `NRF24_DR_1MBPS` | 1 Mbps | Padrão — equilíbrio entre alcance e latência |
| `NRF24_DR_2MBPS` | 2 Mbps | Menor tempo no ar e menor consumo médio |

### `nrf24_pa_level_t`
| Constante | Potência de Saída | Corrente Típica (TX) |
| :--- | :---: | :---: |
| `NRF24_PA_MIN` | `-18 dBm` | ~7.0 mA |
| `NRF24_PA_LOW` | `-12 dBm` | ~7.5 mA |
| `NRF24_PA_HIGH` | `-6 dBm` | ~9.0 mA |
| `NRF24_PA_MAX` | `0 dBm` | ~11.3 mA |

### `nrf24_crc_t` e `nrf24_addr_width_t`
- **CRC:** `NRF24_CRC_DISABLED`, `NRF24_CRC_1BYTE`, `NRF24_CRC_2BYTES` (recomendado e obrigatório com Auto-ACK).
- **Largura de endereço:** `NRF24_ADDR_WIDTH_3` (3B), `NRF24_ADDR_WIDTH_4` (4B), `NRF24_ADDR_WIDTH_5` (5B, padrão).

### `nrf24_config_t` e `NRF24_CONFIG_DEFAULT()`
Estrutura de inicialização contendo host SPI, pinos (`CE`, `CSN`, `MISO`, `MOSI`, `SCK`, `IRQ`), clock SPI, canal, taxa, potência, CRC, retries (`retr_delay`, `retr_count`), tamanho de payload e flags (`auto_ack`, `dynamic_payloads`, `init_spi_bus`).

Use `nrf24_config_t cfg = NRF24_CONFIG_DEFAULT();` para preencher todos os campos com valores seguros e sobrescrever apenas o necessário.

---

## 2. Funções Principais

### Ciclo de Vida e Diagnóstico

| Assinatura | Descrição |
| :--- | :--- |
| `esp_err_t nrf24_init(nrf24_dev_t *dev, const nrf24_config_t *cfg)` | Inicializa GPIO `CE`, barramento SPI2 (se `init_spi_bus=true`), configura registradores, limpa FIFOs e entra em `Standby-I`. |
| `esp_err_t nrf24_deinit(nrf24_dev_t *dev)` | Coloca o rádio em `Power Down` e remove o dispositivo/barramento SPI. |
| `bool nrf24_is_connected(nrf24_dev_t *dev)` | Valida `SETUP_AW` e realiza teste de escrita/leitura em `RF_CH` (preservando o canal atual) para confirmar comunicação SPI real. |
| `void nrf24_print_details(nrf24_dev_t *dev)` | Imprime todos os registradores principais e estatísticas via `ESP_LOGI`. |

### Configuração de Rádio e Pipes

| Assinatura | Descrição |
| :--- | :--- |
| `esp_err_t nrf24_set_channel(nrf24_dev_t *dev, uint8_t channel)` | Seleciona canal RF `0..125` (`2400..2525 MHz`). |
| `esp_err_t nrf24_set_rf_setup(nrf24_dev_t *dev, nrf24_data_rate_t dr, nrf24_pa_level_t pa)` | Configura `RF_SETUP` (taxa de dados + potência PA). |
| `esp_err_t nrf24_set_crc(nrf24_dev_t *dev, nrf24_crc_t crc)` | Configura CRC de hardware no registrador `CONFIG`. |
| `esp_err_t nrf24_set_address_width(nrf24_dev_t *dev, nrf24_addr_width_t width)` | Configura `SETUP_AW` para 3, 4 ou 5 bytes. |
| `esp_err_t nrf24_set_retries(nrf24_dev_t *dev, uint8_t delay, uint8_t count)` | Define `ARD` (`(delay+1)*250 us`, `0..15`) e `ARC` (`0..15`). |
| `esp_err_t nrf24_set_tx_address(nrf24_dev_t *dev, const uint8_t *addr)` | Define `TX_ADDR` e sincroniza `RX_ADDR_P0` quando Auto-ACK está ativo. |
| `esp_err_t nrf24_open_rx_pipe(nrf24_dev_t *dev, uint8_t pipe, const uint8_t *addr, uint8_t payload_size)` | Configura endereço (`addr_width` bytes em P0/P1; 1 byte LSB em P2–P5), `RX_PW_Px` e habilita `EN_RXADDR`. |
| `esp_err_t nrf24_close_rx_pipe(nrf24_dev_t *dev, uint8_t pipe)` | Desabilita o pipe em `EN_RXADDR`. |
| `esp_err_t nrf24_enable_auto_ack(nrf24_dev_t *dev, uint8_t pipe, bool enable)` | Ativa/desativa Auto-ACK no pipe `0..5`. |
| `esp_err_t nrf24_enable_dynamic_payloads(nrf24_dev_t *dev, bool enable)` | Configura `FEATURE.EN_DPL` e `DYNPD`. |
| `esp_err_t nrf24_enable_ack_payload(nrf24_dev_t *dev, bool enable)` | Habilita resposta embutida no ACK (`FEATURE.EN_ACK_PAY`, ativa `EN_DPL` automaticamente). |
| `esp_err_t nrf24_enable_dynamic_ack(nrf24_dev_t *dev, bool enable)` | Habilita transmissões pontuais sem ACK (`FEATURE.EN_DYN_ACK`). |

### Controle de Modo, Transmissão e Recepção

| Assinatura | Descrição |
| :--- | :--- |
| `esp_err_t nrf24_power_up(nrf24_dev_t *dev)` | Seta `PWR_UP=1` e aguarda `Tpd2stby` (`5 ms`). |
| `esp_err_t nrf24_power_down(nrf24_dev_t *dev)` | Zera `CE` e `PWR_UP` (~900 nA). |
| `esp_err_t nrf24_set_tx_mode(nrf24_dev_t *dev)` | Entra em modo TX (`CE=0`, `PRIM_RX=0`), restaurando `RX_ADDR_P0 = TX_ADDR` se Auto-ACK ativo. |
| `esp_err_t nrf24_set_rx_mode(nrf24_dev_t *dev)` | Entra em modo RX (`PRIM_RX=1`, `CE=1`), restaurando `RX_ADDR_P0` do pipe 0 e aguardando `150 us`. |
| `esp_err_t nrf24_send(nrf24_dev_t *dev, const uint8_t *data, size_t len, uint32_t timeout_ms)` | Envia 1..32 bytes e aguarda `TX_DS` (`ESP_OK`), `MAX_RT` (`ESP_FAIL` + `FLUSH_TX`) ou timeout (`ESP_ERR_TIMEOUT`). |
| `esp_err_t nrf24_send_no_ack(nrf24_dev_t *dev, const uint8_t *data, size_t len, uint32_t timeout_ms)` | Envia via `W_TX_PAYLOAD_NOACK`. |
| `bool nrf24_data_available(nrf24_dev_t *dev, uint8_t *pipe_out)` | Retorna `true` se há pacote no FIFO RX e opcionalmente informa o pipe (`0..5`). |
| `esp_err_t nrf24_receive(nrf24_dev_t *dev, uint8_t *buf, size_t max_len, size_t *bytes_read)` | Lê o payload (fixo ou dinâmico) e limpa `RX_DR`. |
| `esp_err_t nrf24_write_ack_payload(nrf24_dev_t *dev, uint8_t pipe, const uint8_t *data, size_t len)` | Pré-carrega payload de resposta anexado ao próximo Auto-ACK do pipe. |

### Status e Telemetria de Hardware

- `uint8_t nrf24_get_status(nrf24_dev_t *dev)` — lê `STATUS` via comando `NOP` (1 byte).
- `bool nrf24_tx_fifo_full(nrf24_dev_t *dev)` / `bool nrf24_rx_fifo_empty(nrf24_dev_t *dev)`
- `bool nrf24_carrier_detect(nrf24_dev_t *dev)` — lê `RPD` ($\ge -64\text{ dBm}$).
- `esp_err_t nrf24_get_observe_tx(nrf24_dev_t *dev, uint8_t *plos_cnt, uint8_t *arc_cnt)`
- `nrf24_stats_t nrf24_get_stats(const nrf24_dev_t *dev)` / `void nrf24_reset_stats(nrf24_dev_t *dev)`

---

## 3. Exemplos Rápidos de Uso

### Transmissor (TX) com Auto-ACK

```c
nrf24_dev_t radio;
nrf24_config_t cfg = NRF24_CONFIG_DEFAULT();
cfg.channel      = 76;
cfg.data_rate    = NRF24_DR_1MBPS;
cfg.pa_level     = NRF24_PA_MAX;
cfg.payload_size = 8;

ESP_ERROR_CHECK(nrf24_init(&radio, &cfg));
if (!nrf24_is_connected(&radio)) {
    ESP_LOGE("APP", "nRF24L01+ desconectado!");
    return;
}

const uint8_t addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0x01 };
ESP_ERROR_CHECK(nrf24_set_tx_address(&radio, addr));
ESP_ERROR_CHECK(nrf24_set_tx_mode(&radio));

uint8_t msg[8] = { 0xA5, 0x01, 0x01, 200, 0, 0, 0, 0 };
esp_err_t err = nrf24_send(&radio, msg, sizeof(msg), 50);
```

### Receptor (RX) com Payload Dinâmico e ACK Payload

```c
nrf24_dev_t radio;
nrf24_config_t cfg = NRF24_CONFIG_DEFAULT();
cfg.channel          = 76;
cfg.dynamic_payloads = true;

ESP_ERROR_CHECK(nrf24_init(&radio, &cfg));
ESP_ERROR_CHECK(nrf24_enable_ack_payload(&radio, true));

const uint8_t addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0x01 };
ESP_ERROR_CHECK(nrf24_open_rx_pipe(&radio, 0, addr, 32));
ESP_ERROR_CHECK(nrf24_set_rx_mode(&radio));

uint8_t telemetry[4] = { 0x42, 0x10, 0x00, 0xFF };
nrf24_write_ack_payload(&radio, 0, telemetry, sizeof(telemetry));

uint8_t pipe = 0, buf[32];
size_t len = 0;
if (nrf24_data_available(&radio, &pipe)) {
    if (nrf24_receive(&radio, buf, sizeof(buf), &len) == ESP_OK) {
        ESP_LOGI("APP", "Recebido %u bytes no pipe %u", (unsigned)len, pipe);
    }
}
```
