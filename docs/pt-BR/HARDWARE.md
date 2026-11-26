# 🔧 Guia de Hardware e Registradores — nRF24L01+ + ESP32

> [🇺🇸 English](../HARDWARE.md) · **🇧🇷 Português**

---

## 1. Esquema de Ligação

### Orientação dos Pinos do Módulo nRF24L01+ (Vista Superior — Conector 2x4)

```text
          ┌───────────────────────────────┐
          │   [1] GND         [2] VCC 3V3 │   ← Ligar capacitor 10uF + 100nF aqui!
  Antena  │   [3] CE (IO4)    [4] CSN(IO5)│
  PCB /   │   [5] SCK (IO18)  [6] MOSI(23)│
  SMA     │   [7] IRQ (NC)    [8] MISO(19)│
          └───────────────────────────────┘
```

### Requisitos Elétricos Críticos

- **Tensão de alimentação (`VCC`):** `1.9 V` a `3.6 V` (nominal **`3.3 V`**). **Ligar em `5 V` queima permanentemente o chip.**
- **Tolerância lógica nos pinos SPI/CE:** Os pinos digitais da ESP32 já operam nativamente em `3.3 V`, não exigindo conversor de nível.
- **Desacoplamento obrigatório:** Conecte um capacitor eletrolítico de **10 µF – 47 µF** em paralelo com um cerâmico de **100 nF** o mais próximo possível dos pinos `VCC` e `GND` do módulo. Sem isso, quedas transitórias de tensão durante a transmissão provocam `MAX_RT` contínuos ou reinicialização silenciosa do rádio.
- **Módulos PA+LNA (com antena externa):** Podem exigir até `115 mA` de pico em `0 dBm`. Recomenda-se regulador dedicado `3.3 V` (ex.: AMS1117-3.3) com `GND` compartilhado com a ESP32.

---

## 2. Protocolo SPI e Temporizações (Datasheet v1.0)

O driver usa **SPI Modo 0 (`CPOL = 0, CPHA = 0`)**, MSB primeiro, com clock padrão de **`1 MHz`** (limite do nRF24L01+: `10 MHz`).

Cada operação é enviada como **uma única transação SPI contínua**:
- `CSN` cai para nível baixo automaticamente no início da transação (gerenciado pelo periférico `SPI2_HOST`).
- Enquanto o byte de comando entra em `MOSI`, o chip desloca simultaneamente o registrador **`STATUS`** em `MISO`.
- `CSN` só retorna para nível alto após o último byte de payload/registrador.

### Temporizações Respeitadas pelo Driver

| Parâmetro | Datasheet (Mín./Máx.) | Valor no Driver | Descrição |
| :--- | :---: | :---: | :--- |
| Estabilização pós-reset | `10.3 ms` | `10 ms` (`vTaskDelay`) | Espera inicial antes do acesso SPI |
| `Tpd2stby` (Power Down → Standby-I) | máx. `1.5 ms` / `4.5 ms` (cristal) | `5000 µs` (`esp_rom_delay_us`) | Após setar `CONFIG.PWR_UP = 1` |
| `Thce` (Pulso mínimo de `CE` em TX) | mín. `10 µs` | `15 µs` | Inicia a transmissão RF do FIFO TX |
| `Tstby2a` (Standby-I → RX/TX ativo) | `130 µs` | `150 µs` | Estabilização do PLL ao ativar RX |
| Integração `RPD` (Scanner RF) | `170 µs` | `170 µs` | Tempo mínimo em RX para amostragem de portadora |

---

## 3. Cálculo da Frequência de Operação

A frequência central do canal `RF_CH` (`0` a `125`) é dada por:

$$f_{\text{RF}} = 2400 + \text{RF\_CH}\;(\text{MHz})$$

- **Canais `0..75` (`2400–2475 MHz`):** Compartilham espectro com redes Wi-Fi 2.4 GHz (canais 1–11).
- **Canais `76..125` (`2476–2525 MHz`):** Geralmente livres de tráfego Wi-Fi doméstico — ideais para enlaces confiáveis. *(Verifique a regulamentação local da faixa ISM 2.400–2.4835 GHz para operação externa.)*

---

## 4. Mapa Resumido de Registradores (`0x00`–`0x1D`)

| Endereço | Nome | Reset | Descrição |
| :---: | :--- | :---: | :--- |
| `0x00` | `CONFIG` | `0x08` | Máscaras de IRQ, `EN_CRC`, `CRCO`, `PWR_UP`, `PRIM_RX` |
| `0x01` | `EN_AA` | `0x3F` | Habilita Auto-ACK nos pipes `P0..P5` |
| `0x02` | `EN_RXADDR` | `0x03` | Habilita endereços de recepção nos pipes `P0..P5` |
| `0x03` | `SETUP_AW` | `0x03` | Largura de endereço (`01`=3B, `10`=4B, `11`=5B) |
| `0x04` | `SETUP_RETR` | `0x03` | `ARD[7:4]` atraso `(ARD+1)*250µs`, `ARC[3:0]` retransmissões |
| `0x05` | `RF_CH` | `0x02` | Canal RF (`0..125`) |
| `0x06` | `RF_SETUP` | `0x0E` | Taxa (`RF_DR_LOW`, `RF_DR_HIGH`) e potência (`RF_PWR[2:1]`) |
| `0x07` | `STATUS` | `0x0E` | Flags `RX_DR`, `TX_DS`, `MAX_RT` (write-1-to-clear), `RX_P_NO`, `TX_FULL` |
| `0x08` | `OBSERVE_TX` | `0x00` | `PLOS_CNT[7:4]` (pacotes perdidos), `ARC_CNT[3:0]` (retransmissões do último envio) |
| `0x09` | `RPD` | `0x00` | Detector de portadora (`1` se potência recebida $\ge -64\text{ dBm}$) |
| `0x0A–0x0F` | `RX_ADDR_P0..P5` | — | Endereços RX (`P0/P1`: 5 bytes; `P2..P5`: 1 byte LSB) |
| `0x10` | `TX_ADDR` | `0xE7..` | Endereço de destino TX (até 5 bytes, LSB primeiro) |
| `0x11–0x16` | `RX_PW_P0..P5` | `0x00` | Largura fixa do payload RX (`0` = desativado, `1..32` bytes) |
| `0x17` | `FIFO_STATUS` | `0x11` | Flags `TX_REUSE`, `TX_FULL`, `TX_EMPTY`, `RX_FULL`, `RX_EMPTY` |
| `0x1C` | `DYNPD` | `0x00` | Habilita payload dinâmico por pipe (`DPL_P0..P5`) |
| `0x1D` | `FEATURE` | `0x00` | `EN_DPL` (bit 2), `EN_ACK_PAY` (bit 1), `EN_DYN_ACK` (bit 0) |
