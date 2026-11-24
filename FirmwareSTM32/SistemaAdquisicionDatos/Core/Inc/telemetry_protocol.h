/**
 * @file telemetry_protocol.h
 * @brief Formato binario ASTRA-I compartido entre los tres nodos.
 * @version 1.0.2
 *
 * CRC-8/ATM: poly=0x07, init=0x00, refin=false, refout=false, xorout=0x00.
 * Los bytes multibyte viajan big-endian; los valores con signo usan C2.
 */
#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_SYNC_0         0xA5U
#define TELEMETRY_SYNC_1         0x5AU
#define TELEMETRY_VERSION        0x01U
#define TELEMETRY_TYPE_DATA      0x01U
#define TELEMETRY_TYPE_ACK       0x80U
#define TELEMETRY_DATA_LENGTH    32U
#define TELEMETRY_ACK_LENGTH     6U
#define TELEMETRY_CRC_INDEX      31U

#define TELEMETRY_FLAG_MPU_VALID 0x01U
#define TELEMETRY_FLAG_BME_VALID 0x02U

typedef enum {
    TELEMETRY_DECODE_OK = 0,
    TELEMETRY_DECODE_BAD_FORMAT,
    TELEMETRY_DECODE_BAD_CRC
} telemetry_decode_status_t;

typedef struct {
    uint8_t sequence;
    uint8_t flags;
    int16_t accel_x_q14;
    int16_t accel_y_q14;
    int16_t accel_z_q14;
    int16_t gyro_x_q7;
    int16_t gyro_y_q7;
    int16_t gyro_z_q7;
    int16_t mpu_temp_q8;
    int16_t bme_temp_q8;
    uint32_t pressure_pa;
    uint16_t humidity_q8;
    uint32_t gas_ohms;
    uint8_t crc_received;
    uint8_t crc_calculated;
} telemetry_sample_t;

static inline uint8_t telemetry_crc8_atm(const uint8_t *data, size_t length)
{
    uint8_t crc = 0U;

    for (size_t i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x80U) ?
                  (uint8_t)((crc << 1) ^ 0x07U) :
                  (uint8_t)(crc << 1);
        }
    }

    return crc;
}

static inline uint16_t telemetry_read_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static inline int16_t telemetry_read_i16_be(const uint8_t *p)
{
    uint16_t raw = telemetry_read_u16_be(p);
    int32_t signed_value = (raw & 0x8000U) ?
                           ((int32_t)raw - 65536L) : (int32_t)raw;
    return (int16_t)signed_value;
}

static inline uint32_t telemetry_read_u24_be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 16) |
           ((uint32_t)p[1] << 8) |
           (uint32_t)p[2];
}

static inline uint32_t telemetry_read_u32_be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

static inline telemetry_decode_status_t
telemetry_decode_data(const uint8_t *frame, size_t length,
                      telemetry_sample_t *sample)
{
    if (frame == NULL || sample == NULL || length != TELEMETRY_DATA_LENGTH ||
        frame[0] != TELEMETRY_SYNC_0 || frame[1] != TELEMETRY_SYNC_1 ||
        frame[2] != TELEMETRY_VERSION || frame[3] != TELEMETRY_TYPE_DATA) {
        return TELEMETRY_DECODE_BAD_FORMAT;
    }

    sample->sequence = frame[4];
    sample->flags = frame[5];
    sample->crc_received = frame[TELEMETRY_CRC_INDEX];
    sample->crc_calculated =
        telemetry_crc8_atm(&frame[2], TELEMETRY_CRC_INDEX - 2U);

    if (sample->crc_received != sample->crc_calculated)
        return TELEMETRY_DECODE_BAD_CRC;

    sample->accel_x_q14 = telemetry_read_i16_be(&frame[6]);
    sample->accel_y_q14 = telemetry_read_i16_be(&frame[8]);
    sample->accel_z_q14 = telemetry_read_i16_be(&frame[10]);
    sample->gyro_x_q7 = telemetry_read_i16_be(&frame[12]);
    sample->gyro_y_q7 = telemetry_read_i16_be(&frame[14]);
    sample->gyro_z_q7 = telemetry_read_i16_be(&frame[16]);
    sample->mpu_temp_q8 = telemetry_read_i16_be(&frame[18]);
    sample->bme_temp_q8 = telemetry_read_i16_be(&frame[20]);
    sample->pressure_pa = telemetry_read_u24_be(&frame[22]);
    sample->humidity_q8 = telemetry_read_u16_be(&frame[25]);
    sample->gas_ohms = telemetry_read_u32_be(&frame[27]);

    return TELEMETRY_DECODE_OK;
}

static inline void telemetry_make_ack(uint8_t sequence,
                                      uint8_t ack[TELEMETRY_ACK_LENGTH])
{
    ack[0] = TELEMETRY_SYNC_0;
    ack[1] = TELEMETRY_SYNC_1;
    ack[2] = TELEMETRY_VERSION;
    ack[3] = TELEMETRY_TYPE_ACK;
    ack[4] = sequence;
    ack[5] = telemetry_crc8_atm(&ack[2], 3U);
}

static inline bool telemetry_validate_ack(const uint8_t *ack, size_t length,
                                          uint8_t expected_sequence)
{
    return ack != NULL && length == TELEMETRY_ACK_LENGTH &&
           ack[0] == TELEMETRY_SYNC_0 && ack[1] == TELEMETRY_SYNC_1 &&
           ack[2] == TELEMETRY_VERSION && ack[3] == TELEMETRY_TYPE_ACK &&
           ack[4] == expected_sequence &&
           ack[5] == telemetry_crc8_atm(&ack[2], 3U);
}

#endif /* TELEMETRY_PROTOCOL_H */
