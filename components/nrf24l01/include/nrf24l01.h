#ifndef NRF24L01_H
#define NRF24L01_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <stdbool.h>

// **Pinos do NRF24L01 (ajuste conforme necessário)**
#define PIN_CE    4
#define PIN_CSN   5
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_SCK   18

// **Definição dos registradores do NRF24L01**
#define NRF_REG_CONFIG        0x00  // Configuração do módulo
#define NRF_REG_EN_AA         0x01  // Habilitação de Auto ACK
#define NRF_REG_EN_RXADDR     0x02  // Habilitação de canais RX
#define NRF_REG_SETUP_AW      0x03  // Largura do endereço
#define NRF_REG_SETUP_RETR    0x04  // Configuração de retransmissão
#define NRF_REG_RF_CH         0x05  // Canal de rádio
#define NRF_REG_RF_SETUP      0x06  // Configuração de potência e taxa de transmissão
#define NRF_REG_STATUS        0x07  // Status do NRF24L01
#define NRF_REG_OBSERVE_TX    0x08  // Contador de pacotes perdidos
#define NRF_REG_RPD           0x09  // Detecção de portadora
#define NRF_REG_RX_ADDR_P0    0x0A  // Endereço RX Pipe 0
#define NRF_REG_RX_ADDR_P1    0x0B  // Endereço RX Pipe 1
#define NRF_REG_RX_ADDR_P2    0x0C  // Endereço RX Pipe 2
#define NRF_REG_RX_ADDR_P3    0x0D  // Endereço RX Pipe 3
#define NRF_REG_RX_ADDR_P4    0x0E  // Endereço RX Pipe 4
#define NRF_REG_RX_ADDR_P5    0x0F  // Endereço RX Pipe 5
#define NRF_REG_TX_ADDR       0x10  // Endereço TX
#define NRF_REG_RX_PW_P0      0x11  // Tamanho do payload do Pipe 0
#define NRF_REG_RX_PW_P1      0x12  // Tamanho do payload do Pipe 1
#define NRF_REG_RX_PW_P2      0x13  // Tamanho do payload do Pipe 2
#define NRF_REG_RX_PW_P3      0x14  // Tamanho do payload do Pipe 3
#define NRF_REG_RX_PW_P4      0x15  // Tamanho do payload do Pipe 4
#define NRF_REG_RX_PW_P5      0x16  // Tamanho do payload do Pipe 5
#define NRF_REG_FIFO_STATUS   0x17  // Status do FIFO TX e RX
#define NRF_REG_DYNPD         0x1C  // Habilitação de payload dinâmico
#define NRF_REG_FEATURE       0x1D  // Habilitação de funcionalidades adicionais

// **Definição de comandos do NRF24L01**
#define NRF_CMD_R_REGISTER    0x00  // Ler registrador
#define NRF_CMD_W_REGISTER    0x20  // Escrever registrador
#define NRF_CMD_R_RX_PAYLOAD  0x61  // Ler payload do FIFO RX
#define NRF_CMD_W_TX_PAYLOAD  0xA0  // Escrever payload no FIFO TX
#define NRF_CMD_FLUSH_TX      0xE1  // Limpa o buffer de transmissão
#define NRF_CMD_FLUSH_RX      0xE2  // Limpa o buffer de recepção
#define NRF_CMD_REUSE_TX_PL   0xE3  // Reutiliza último payload enviado
#define NRF_CMD_NOP           0xFF  // No Operation (ler STATUS)

// **Definição de bits do registrador CONFIG**
#define NRF_CONFIG_PRIM_RX    0x01  // 1 = Modo RX, 0 = Modo TX
#define NRF_CONFIG_PWR_UP     0x02  // 1 = Power Up, 0 = Power Down
#define NRF_CONFIG_CRCO       0x04  // 1 = CRC 2 bytes, 0 = CRC 1 byte
#define NRF_CONFIG_EN_CRC     0x08  // 1 = Habilita CRC, 0 = Sem CRC
#define NRF_CONFIG_MASK_MAX_RT 0x10 // Máscara de interrupção de retransmissão máxima

// **Definição de bits do registrador STATUS**
#define NRF_STATUS_RX_DR      0x40  // Dados recebidos
#define NRF_STATUS_TX_DS      0x20  // Transmissão bem-sucedida
#define NRF_STATUS_MAX_RT     0x10  // Número máximo de retransmissões atingido

// **Funções de inicialização e configuração**
void nrf24_init();

// IMPLEMENTAR
void nrf24_set_channel(uint8_t channel);
void nrf24_set_tx_address(const uint8_t *address, uint8_t size);
void nrf24_set_rx_address(uint8_t pipe, const uint8_t *address, uint8_t size);
void nrf24_set_data_rate(uint8_t data_rate);
void nrf24_set_pa_level(uint8_t pa_level);
void nrf24_enable_auto_ack(bool enable);
void nrf24_enable_crc(bool enable);

// **Funções de leitura e escrita**
void nrf24_write_register(uint8_t reg, const uint8_t *value, size_t size);
uint8_t nrf24_read_register(uint8_t reg);
void nrf24_send_data(uint8_t *data, size_t len);
uint8_t nrf24_receive_data(uint8_t *buffer, size_t len);

// **Funções de verificação e controle**
uint8_t nrf24_check_status();
void nrf24_reset_status();
bool nrf24_tx_fifo_full();
void nrf24_flush_tx();
void nrf24_flush_rx();

#endif
