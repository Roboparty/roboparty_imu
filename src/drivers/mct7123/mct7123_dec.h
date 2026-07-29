/*
 * SPDX-License-Identifier: GPL-3.0
 *
 * MCT MD7123 Decoder Library
 * Byte-level UART protocol decoder for MD7123 inertial measurement unit.
 *
 * Frame format (ref: MD7123 User Manual V2.0):
 *   Header:  0xEB 0x90
 *   Msg_ID:  1 byte (0x81=IMU raw, 0x82=Attitude, 0x83=Config)
 *   Length:  1 byte (fixed 0x40 = 64 byte payload)
 *   Payload: 64 bytes
 *   Tail:    0xED
 *   CRC16:   CCITT (poly 0x1021, init 0xFFFF) over payload[0..61]
 */

#ifndef __MCT7123_DEC_H__
#define __MCT7123_DEC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define MCT7123_MSG_IMU_DATA  0x81
#define MCT7123_MSG_ATT_DATA  0x82
#define MCT7123_MSG_CFG_DATA  0x83

#define MCT7123_PAYLOAD_LEN   64
#define MCT7123_BUF_SIZE      (2 + 1 + 1 + MCT7123_PAYLOAD_LEN + 1)  /* 69 */

typedef struct {
    uint8_t  buf[MCT7123_BUF_SIZE];
    int      idx;

    uint8_t  msg_id;
    uint8_t  payload_len;
    uint8_t  payload[MCT7123_PAYLOAD_LEN];
} mct7123_raw_t;

/**
 * Process one byte, returns:
 *  1  = complete frame decoded
 *  0  = need more data
 * -1  = error (CRC mismatch, sync lost, etc.)
 */
int mct7123_input(mct7123_raw_t *raw, uint8_t data);

/** CRC16-CCITT over payload bytes (poly 0x1021, init 0xFFFF). */
uint16_t mct7123_crc16(const uint8_t *data, size_t len);

/** Parse payload into float fields after successful mct7123_input(). */
void mct7123_parse_imu(const uint8_t payload[64],
                       float *gyr_x, float *gyr_y, float *gyr_z,
                       float *acc_x, float *acc_y, float *acc_z,
                       float *mag_x, float *mag_y, float *mag_z,
                       float *temp, uint64_t *systimer_us, uint8_t *cycle);

void mct7123_parse_att(const uint8_t payload[64],
                       float *roll, float *pitch, float *yaw,
                       float *qw, float *qx, float *qy, float *qz,
                       float *temp, uint8_t *running_status, uint32_t *fusion_status,
                       uint64_t *systimer_us, uint8_t *cycle);

/** MD7123 config frame (Msg_ID 0x83 / CAN ID 0x183). All pointers may be NULL. */
void mct7123_parse_cfg(const uint8_t payload[64],
                       uint8_t *sentence_ctrl, uint8_t *output_rate,
                       uint8_t *orientation, uint8_t *fusion_mode,
                       double *latitude, double *longitude, float *height);

#ifdef __cplusplus
}
#endif

#endif /* __MCT7123_DEC_H__ */
