# 🔧 Hardware & Register Guide — nRF24L01+ + ESP32

> **🇺🇸 English** · [🇧🇷 Português](pt-BR/HARDWARE.md)

---

## 1. Wiring Diagram

### nRF24L01+ 2x4 Header Pinout (Top View)

```text
          ┌───────────────────────────────┐
          │   [1] GND         [2] VCC 3V3 │   ← Place 10uF + 100nF caps here!
  PCB /   │   [3] CE (IO4)    [4] CSN(IO5)│
  SMA     │   [5] SCK (IO18)  [6] MOSI(23)│
  Antenna │   [7] IRQ (NC)    [8] MISO(19)│
          └───────────────────────────────┘
```

### Critical Electrical Requirements

- **Supply voltage (`VCC`):** `1.9 V` to `3.6 V` (**`3.3 V`** nominal). **Connecting `5 V` permanently damages the chip.**
- **Logic levels on SPI/CE pins:** ESP32 GPIOs operate natively at `3.3 V`, so no level shifter is required.
- **Mandatory decoupling:** Connect a **10 µF – 47 µF** electrolytic capacitor in parallel with a **100 nF** ceramic capacitor as close as possible to the module's `VCC` and `GND` pins. Without decoupling, TX current spikes cause continuous `MAX_RT` errors or silent radio resets.
- **PA+LNA modules (external antenna):** Can draw up to `115 mA` peak at `0 dBm`. Use a dedicated `3.3 V` regulator (e.g., AMS1117-3.3) sharing a common `GND` with the ESP32.

---

## 2. SPI Protocol & Timings (Datasheet v1.0)

The driver uses **SPI Mode 0 (`CPOL = 0, CPHA = 0`)**, MSB first, with a default clock of **`1 MHz`** (nRF24L01+ limit: `10 MHz`).

Every operation executes as **a single contiguous SPI transaction**:
- `CSN` is pulled low automatically at transaction start by the `SPI2_HOST` peripheral.
- While the command byte is shifted in on `MOSI`, the chip simultaneously shifts out the **`STATUS`** register on `MISO`.
- `CSN` returns high only after the final register/payload byte completes.

### Datasheet Timings Enforced by the Driver

| Parameter | Datasheet (Min/Max) | Driver Value | Purpose |
| :--- | :---: | :---: | :--- |
| Power-on reset settle | `10.3 ms` | `10 ms` (`vTaskDelay`) | Initial settle delay before SPI access |
| `Tpd2stby` (Power Down → Standby-I) | max `1.5 ms` / `4.5 ms` (crystal) | `5000 µs` (`esp_rom_delay_us`) | Wait after setting `CONFIG.PWR_UP = 1` |
| `Thce` (Minimum TX `CE` pulse) | min `10 µs` | `15 µs` | Triggers RF transmission from TX FIFO |
| `Tstby2a` (Standby-I → Active RX/TX) | `130 µs` | `150 µs` | PLL lock delay when entering RX mode |
| `RPD` integration (RF Scanner) | `170 µs` | `170 µs` | Minimum RX dwell time before sampling carrier |

---

## 3. Operating Frequency Formula

The center frequency for channel `RF_CH` (`0` to `125`) is:

$$f_{\text{RF}} = 2400 + \text{RF\_CH}\;(\text{MHz})$$

- **Channels `0..75` (`2400–2475 MHz`):** Overlap with 2.4 GHz Wi-Fi channels 1–11.
- **Channels `76..125` (`2476–2525 MHz`):** Typically free of domestic Wi-Fi traffic — ideal for reliable links. *(Check local ISM band regulations for `2400–2483.5 MHz` when operating outdoors.)*

---

## 4. Register Map Summary (`0x00`–`0x1D`)

| Address | Name | Reset | Description |
| :---: | :--- | :---: | :--- |
| `0x00` | `CONFIG` | `0x08` | IRQ masks, `EN_CRC`, `CRCO`, `PWR_UP`, `PRIM_RX` |
| `0x01` | `EN_AA` | `0x3F` | Auto-ACK enable for pipes `P0..P5` |
| `0x02` | `EN_RXADDR` | `0x03` | Enabled RX pipe addresses `P0..P5` |
| `0x03` | `SETUP_AW` | `0x03` | Address width (`01`=3B, `10`=4B, `11`=5B) |
| `0x04` | `SETUP_RETR` | `0x03` | `ARD[7:4]` delay `(ARD+1)*250µs`, `ARC[3:0]` retry count |
| `0x05` | `RF_CH` | `0x02` | RF channel (`0..125`) |
| `0x06` | `RF_SETUP` | `0x0E` | Air data rate (`RF_DR_LOW`, `RF_DR_HIGH`) and PA power (`RF_PWR[2:1]`) |
| `0x07` | `STATUS` | `0x0E` | `RX_DR`, `TX_DS`, `MAX_RT` (write-1-to-clear), `RX_P_NO`, `TX_FULL` |
| `0x08` | `OBSERVE_TX` | `0x00` | `PLOS_CNT[7:4]` (lost packets), `ARC_CNT[3:0]` (retransmissions of last packet) |
| `0x09` | `RPD` | `0x00` | Received Power Detector (`1` if signal $\ge -64\text{ dBm}$) |
| `0x0A–0x0F` | `RX_ADDR_P0..P5` | — | RX addresses (`P0/P1`: up to 5 bytes; `P2..P5`: 1 byte LSB) |
| `0x10` | `TX_ADDR` | `0xE7..` | Destination TX address (up to 5 bytes, LSB first) |
| `0x11–0x16` | `RX_PW_P0..P5` | `0x00` | Fixed RX payload width (`0` = unused, `1..32` bytes) |
| `0x17` | `FIFO_STATUS` | `0x11` | `TX_REUSE`, `TX_FULL`, `TX_EMPTY`, `RX_FULL`, `RX_EMPTY` |
| `0x1C` | `DYNPD` | `0x00` | Per-pipe dynamic payload length enable (`DPL_P0..P5`) |
| `0x1D` | `FEATURE` | `0x00` | `EN_DPL` (bit 2), `EN_ACK_PAY` (bit 1), `EN_DYN_ACK` (bit 0) |
