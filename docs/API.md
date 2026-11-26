# 📘 API Reference — `nrf24l01` Component

> **🇺🇸 English** · [🇧🇷 Português](pt-BR/API.md)

This document describes the public interface of the [`components/nrf24l01`](../components/nrf24l01/include/nrf24l01.h) ESP-IDF component.

---

## 1. Types & Enumerations

### `nrf24_data_rate_t`
| Constant | Air Data Rate | Notes |
| :--- | :---: | :--- |
| `NRF24_DR_250KBPS` | 250 kbps | Highest sensitivity / longest range (nRF24L01+ only) |
| `NRF24_DR_1MBPS` | 1 Mbps | Default — balanced range and latency |
| `NRF24_DR_2MBPS` | 2 Mbps | Lowest on-air time and average power consumption |

### `nrf24_pa_level_t`
| Constant | Output Power | Typical TX Current |
| :--- | :---: | :---: |
| `NRF24_PA_MIN` | `-18 dBm` | ~7.0 mA |
| `NRF24_PA_LOW` | `-12 dBm` | ~7.5 mA |
| `NRF24_PA_HIGH` | `-6 dBm` | ~9.0 mA |
| `NRF24_PA_MAX` | `0 dBm` | ~11.3 mA |

### `nrf24_crc_t` & `nrf24_addr_width_t`
- **CRC:** `NRF24_CRC_DISABLED`, `NRF24_CRC_1BYTE`, `NRF24_CRC_2BYTES` (recommended and required when Auto-ACK is enabled).
- **Address width:** `NRF24_ADDR_WIDTH_3` (3B), `NRF24_ADDR_WIDTH_4` (4B), `NRF24_ADDR_WIDTH_5` (5B, default).

### `nrf24_config_t` & `NRF24_CONFIG_DEFAULT()`
Initialization structure holding the SPI host, GPIO pins (`CE`, `CSN`, `MISO`, `MOSI`, `SCK`, `IRQ`), SPI clock frequency, RF channel, air data rate, PA level, CRC mode, auto-retry parameters (`retr_delay`, `retr_count`), fixed payload size, and feature flags (`auto_ack`, `dynamic_payloads`, `init_spi_bus`).

Initialize with `nrf24_config_t cfg = NRF24_CONFIG_DEFAULT();` and override only the fields you need.

---

## 2. Core Functions

### Lifecycle & Hardware Diagnostics

| Signature | Description |
| :--- | :--- |
| `esp_err_t nrf24_init(nrf24_dev_t *dev, const nrf24_config_t *cfg)` | Configures `CE` GPIO, initializes the SPI bus (when `init_spi_bus=true`), writes radio registers, flushes FIFOs, and enters `Standby-I`. |
| `esp_err_t nrf24_deinit(nrf24_dev_t *dev)` | Places the radio into `Power Down` and releases SPI device/bus resources. |
| `bool nrf24_is_connected(nrf24_dev_t *dev)` | Validates `SETUP_AW` and performs a non-destructive write/readback test on `RF_CH` to confirm live SPI communication. |
| `void nrf24_print_details(nrf24_dev_t *dev)` | Logs a human-readable dump of all key registers and driver counters via `ESP_LOGI`. |

### Radio & Pipe Configuration

| Signature | Description |
| :--- | :--- |
| `esp_err_t nrf24_set_channel(nrf24_dev_t *dev, uint8_t channel)` | Sets RF channel `0..125` (`2400..2525 MHz`). |
| `esp_err_t nrf24_set_rf_setup(nrf24_dev_t *dev, nrf24_data_rate_t dr, nrf24_pa_level_t pa)` | Configures `RF_SETUP` (air data rate + PA output power). |
| `esp_err_t nrf24_set_crc(nrf24_dev_t *dev, nrf24_crc_t crc)` | Sets hardware CRC mode in `CONFIG`. |
| `esp_err_t nrf24_set_address_width(nrf24_dev_t *dev, nrf24_addr_width_t width)` | Sets `SETUP_AW` to 3, 4, or 5 bytes. |
| `esp_err_t nrf24_set_retries(nrf24_dev_t *dev, uint8_t delay, uint8_t count)` | Configures `ARD` (`(delay+1)*250 us`, `0..15`) and `ARC` (`0..15`). |
| `esp_err_t nrf24_set_tx_address(nrf24_dev_t *dev, const uint8_t *addr)` | Writes `TX_ADDR` and mirrors it into `RX_ADDR_P0` when Auto-ACK is enabled. |
| `esp_err_t nrf24_open_rx_pipe(nrf24_dev_t *dev, uint8_t pipe, const uint8_t *addr, uint8_t payload_size)` | Writes pipe address (`addr_width` bytes for P0/P1; 1 LSB byte for P2–P5), sets `RX_PW_Px`, and enables the pipe in `EN_RXADDR`. |
| `esp_err_t nrf24_close_rx_pipe(nrf24_dev_t *dev, uint8_t pipe)` | Disables the pipe in `EN_RXADDR`. |
| `esp_err_t nrf24_enable_auto_ack(nrf24_dev_t *dev, uint8_t pipe, bool enable)` | Enables/disables Auto-ACK on pipe `0..5`. |
| `esp_err_t nrf24_enable_dynamic_payloads(nrf24_dev_t *dev, bool enable)` | Configures `FEATURE.EN_DPL` and `DYNPD`. |
| `esp_err_t nrf24_enable_ack_payload(nrf24_dev_t *dev, bool enable)` | Enables payloads piggybacked on Auto-ACK packets (`FEATURE.EN_ACK_PAY`, enables `EN_DPL` automatically). |
| `esp_err_t nrf24_enable_dynamic_ack(nrf24_dev_t *dev, bool enable)` | Enables per-packet no-ACK sends (`FEATURE.EN_DYN_ACK`). |

### Mode Control, TX & RX

| Signature | Description |
| :--- | :--- |
| `esp_err_t nrf24_power_up(nrf24_dev_t *dev)` | Sets `PWR_UP=1` and waits `Tpd2stby` (`5 ms`). |
| `esp_err_t nrf24_power_down(nrf24_dev_t *dev)` | Pulls `CE=0` and clears `PWR_UP` (~900 nA). |
| `esp_err_t nrf24_set_tx_mode(nrf24_dev_t *dev)` | Enters TX mode (`CE=0`, `PRIM_RX=0`), restoring `RX_ADDR_P0 = TX_ADDR` if Auto-ACK is active. |
| `esp_err_t nrf24_set_rx_mode(nrf24_dev_t *dev)` | Enters active RX mode (`PRIM_RX=1`, `CE=1`), restoring Pipe 0's RX address and waiting `150 us`. |
| `esp_err_t nrf24_send(nrf24_dev_t *dev, const uint8_t *data, size_t len, uint32_t timeout_ms)` | Transmits 1..32 bytes and waits for `TX_DS` (`ESP_OK`), `MAX_RT` (`ESP_FAIL` + `FLUSH_TX`), or timeout (`ESP_ERR_TIMEOUT`). |
| `esp_err_t nrf24_send_no_ack(nrf24_dev_t *dev, const uint8_t *data, size_t len, uint32_t timeout_ms)` | Transmits using `W_TX_PAYLOAD_NOACK`. |
| `bool nrf24_data_available(nrf24_dev_t *dev, uint8_t *pipe_out)` | Returns `true` when an RX payload is ready and optionally outputs its pipe (`0..5`). |
| `esp_err_t nrf24_receive(nrf24_dev_t *dev, uint8_t *buf, size_t max_len, size_t *bytes_read)` | Reads the payload (fixed or dynamic width) and clears `RX_DR`. |
| `esp_err_t nrf24_write_ack_payload(nrf24_dev_t *dev, uint8_t pipe, const uint8_t *data, size_t len)` | Preloads a response payload to be sent with the next Auto-ACK on the given pipe. |

### Status & Hardware Telemetry

- `uint8_t nrf24_get_status(nrf24_dev_t *dev)` — reads `STATUS` using a 1-byte `NOP` command.
- `bool nrf24_tx_fifo_full(nrf24_dev_t *dev)` / `bool nrf24_rx_fifo_empty(nrf24_dev_t *dev)`
- `bool nrf24_carrier_detect(nrf24_dev_t *dev)` — reads `RPD` ($\ge -64\text{ dBm}$).
- `esp_err_t nrf24_get_observe_tx(nrf24_dev_t *dev, uint8_t *plos_cnt, uint8_t *arc_cnt)`
- `nrf24_stats_t nrf24_get_stats(const nrf24_dev_t *dev)` / `void nrf24_reset_stats(nrf24_dev_t *dev)`

---

## 3. Quick Usage Examples

### Transmitter (TX) with Auto-ACK

```c
nrf24_dev_t radio;
nrf24_config_t cfg = NRF24_CONFIG_DEFAULT();
cfg.channel      = 76;
cfg.data_rate    = NRF24_DR_1MBPS;
cfg.pa_level     = NRF24_PA_MAX;
cfg.payload_size = 8;

ESP_ERROR_CHECK(nrf24_init(&radio, &cfg));
if (!nrf24_is_connected(&radio)) {
    ESP_LOGE("APP", "nRF24L01+ not detected!");
    return;
}

const uint8_t addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0x01 };
ESP_ERROR_CHECK(nrf24_set_tx_address(&radio, addr));
ESP_ERROR_CHECK(nrf24_set_tx_mode(&radio));

uint8_t msg[8] = { 0xA5, 0x01, 0x01, 200, 0, 0, 0, 0 };
esp_err_t err = nrf24_send(&radio, msg, sizeof(msg), 50);
```

### Receiver (RX) with Dynamic Payloads & ACK Payload

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
        ESP_LOGI("APP", "Received %u bytes on pipe %u", (unsigned)len, pipe);
    }
}
```
