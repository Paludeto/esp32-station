#ifndef STATION_PROTOCOL_H
#define STATION_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STATION_PACKET_MAGIC      0xA5U
#define STATION_PROTOCOL_VERSION  0x01U

/** @brief Códigos de comando da estação de controle remoto. */
typedef enum {
    STATION_CMD_STOP     = 0x00, /**< Parar atuadores / estado seguro */
    STATION_CMD_FORWARD  = 0x01, /**< Mover para frente (W) */
    STATION_CMD_BACKWARD = 0x02, /**< Mover para trás / ré (S) */
    STATION_CMD_LEFT     = 0x03, /**< Girar à esquerda (A) */
    STATION_CMD_RIGHT    = 0x04, /**< Girar à direita (D) */
    STATION_CMD_PING     = 0x10, /**< Teste de conectividade / latência (P) */
    STATION_CMD_BEACON   = 0x20, /**< Heartbeat periódico de telemetria */
} station_cmd_t;

/**
 * @brief Quadro binário de 8 bytes trafegado via nRF24L01+.
 *
 * Layout (little-endian):
 *   Byte 0   : magic   (0xA5)
 *   Byte 1   : version (0x01)
 *   Byte 2   : cmd     (station_cmd_t)
 *   Byte 3   : speed   (0..255, intensidade/PWM)
 *   Bytes 4-5: seq     (número sequencial para detecção de perda)
 *   Byte 6   : flags   (reservado para bits de estado/telemetria)
 *   Byte 7   : crc8    (CRC-8/SMBUS polinômio 0x07 dos bytes 0..6)
 */
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  cmd;
    uint8_t  speed;
    uint16_t seq;
    uint8_t  flags;
    uint8_t  crc8;
} station_packet_t;

_Static_assert(sizeof(station_packet_t) == 8U, "station_packet_t deve ter exatamente 8 bytes");

/**
 * @brief Calcula CRC-8/SMBUS (polinômio 0x07, valor inicial 0x00).
 */
static inline uint8_t station_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00U;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; bit++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Monta um pacote preenchido com cabeçalho e CRC-8 calculado.
 */
static inline void station_packet_build(station_packet_t *pkt,
                                        station_cmd_t cmd,
                                        uint8_t speed,
                                        uint16_t seq,
                                        uint8_t flags) {
    pkt->magic   = STATION_PACKET_MAGIC;
    pkt->version = STATION_PROTOCOL_VERSION;
    pkt->cmd     = (uint8_t)cmd;
    pkt->speed   = speed;
    pkt->seq     = seq;
    pkt->flags   = flags;
    pkt->crc8    = station_crc8((const uint8_t *)pkt, sizeof(station_packet_t) - 1U);
}

/**
 * @brief Valida cabeçalho (magic, versão) e integridade CRC-8 de um pacote recebido.
 */
static inline bool station_packet_validate(const station_packet_t *pkt) {
    if (pkt == NULL) {
        return false;
    }
    if ((pkt->magic != STATION_PACKET_MAGIC) || (pkt->version != STATION_PROTOCOL_VERSION)) {
        return false;
    }
    uint8_t expected = station_crc8((const uint8_t *)pkt, sizeof(station_packet_t) - 1U);
    return pkt->crc8 == expected;
}

/**
 * @brief Retorna o nome legível de um código de comando.
 */
static inline const char *station_cmd_name(uint8_t cmd) {
    switch (cmd) {
        case STATION_CMD_STOP:     return "STOP";
        case STATION_CMD_FORWARD:  return "FORWARD";
        case STATION_CMD_BACKWARD: return "BACKWARD";
        case STATION_CMD_LEFT:     return "LEFT";
        case STATION_CMD_RIGHT:    return "RIGHT";
        case STATION_CMD_PING:     return "PING";
        case STATION_CMD_BEACON:   return "BEACON";
        default:                   return "UNKNOWN";
    }
}

#ifdef __cplusplus
}
#endif

#endif /* STATION_PROTOCOL_H */
