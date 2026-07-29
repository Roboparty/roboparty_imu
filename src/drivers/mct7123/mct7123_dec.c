/*
 * SPDX-License-Identifier: GPL-3.0
 *
 * MCT MD7123 byte-level UART protocol decoder.
 */

#include "mct7123_dec.h"
#include <string.h>
#include <stdio.h>

/* CRC16-CCITT: poly 0x1021, init 0xFFFF */
uint16_t mct7123_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint64_t le64(const uint8_t *p)
{
    uint64_t v;
    memcpy(&v, p, 8);
    return v;
}

static float r4(const uint8_t *p)
{
    float v;
    memcpy(&v, p, 4);
    return v;
}

int mct7123_input(mct7123_raw_t *raw, uint8_t data)
{
    /* state 0: waiting for 0xEB */
    if (raw->idx == 0) {
        if (data == 0xEB) {
            raw->buf[raw->idx++] = data;
        }
        return 0;
    }

    /* state 1: expect 0x90 */
    if (raw->idx == 1) {
        if (data == 0x90) {
            raw->buf[raw->idx++] = data;
        } else {
            /* false sync, restart */
            raw->idx = 0;
            if (data == 0xEB) {
                raw->buf[raw->idx++] = data;
            }
        }
        return 0;
    }

    /* state 2: MsgID */
    if (raw->idx == 2) {
        raw->msg_id = data;
        raw->buf[raw->idx++] = data;
        return 0;
    }

    /* state 3: payload length (must be 0x40) */
    if (raw->idx == 3) {
        if (data != MCT7123_PAYLOAD_LEN) {
            /* invalid length, restart */
            raw->idx = 0;
            return -1;
        }
        raw->payload_len = data;
        raw->buf[raw->idx++] = data;
        return 0;
    }

    /* state 4: collecting payload (64 bytes) */
    if (raw->idx < 4 + MCT7123_PAYLOAD_LEN) {
        raw->payload[raw->idx - 4] = data;
        raw->buf[raw->idx++] = data;
        return 0;
    }

    /* state 5: tail (must be 0xED) */
    if (raw->idx == 4 + MCT7123_PAYLOAD_LEN) {
        raw->buf[raw->idx++] = data;
        if (data != 0xED) {
            raw->idx = 0;
            return -1;
        }

        /* verify CRC16 over payload[0..61], CRC at payload[62..63] */
        uint16_t crc_expected = le16(&raw->payload[62]);
        uint16_t crc_calc = mct7123_crc16(raw->payload, 62);
        if (crc_calc != crc_expected) {
            raw->idx = 0;
            return -1;
        }

        raw->idx = 0;
        return 1;  /* complete frame */
    }

    return 0;
}

void mct7123_parse_imu(const uint8_t payload[64],
                       float *gyr_x, float *gyr_y, float *gyr_z,
                       float *acc_x, float *acc_y, float *acc_z,
                       float *mag_x, float *mag_y, float *mag_z,
                       float *temp, uint64_t *systimer_us)
{
    if (systimer_us) memcpy(systimer_us, payload, 8);
    *gyr_x = r4(payload + 8);
    *gyr_y = r4(payload + 12);
    *gyr_z = r4(payload + 16);
    *acc_x = r4(payload + 20);
    *acc_y = r4(payload + 24);
    *acc_z = r4(payload + 28);
    *mag_x = r4(payload + 32);
    *mag_y = r4(payload + 36);
    *mag_z = r4(payload + 40);
    *temp  = r4(payload + 44);
}

void mct7123_parse_att(const uint8_t payload[64],
                       float *roll, float *pitch, float *yaw,
                       float *qw, float *qx, float *qy, float *qz,
                       float *temp, uint8_t *running_status, uint32_t *fusion_status,
                       uint64_t *systimer_us)
{
    if (systimer_us) memcpy(systimer_us, payload, 8);
    *roll  = r4(payload + 8);
    *pitch = r4(payload + 12);
    *yaw   = r4(payload + 16);
    *qx    = r4(payload + 20);
    *qy    = r4(payload + 24);
    *qz    = r4(payload + 28);
    *qw    = r4(payload + 32);
    *temp  = r4(payload + 36);

    if (fusion_status) {
        uint32_t fs;
        memcpy(&fs, payload + 38, 4);
        *fusion_status = fs;
    }
    if (running_status) {
        memcpy(running_status, payload + 42, 6);
    }
}

void mct7123_parse_cfg(const uint8_t payload[64],
                       uint8_t *sentence_ctrl, uint8_t *output_rate,
                       uint8_t *orientation, uint8_t *fusion_mode,
                       double *latitude, double *longitude, float *height)
{
    if (sentence_ctrl) *sentence_ctrl = payload[0];
    if (output_rate)   *output_rate   = payload[1];
    if (orientation)   *orientation   = payload[2];
    if (fusion_mode)   *fusion_mode   = payload[3];
    if (latitude)      memcpy(latitude,  payload + 4,  8);
    if (longitude)     memcpy(longitude, payload + 12, 8);
    if (height)        memcpy(height,    payload + 20, 4);
}
