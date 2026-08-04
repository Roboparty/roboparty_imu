// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

/*
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

/**
 * @brief Byte-by-byte decoder state for an MCT7123 UART frame.
 *
 * The structure retains synchronization state and the current payload between
 * calls to mct7123_input(). Zero-initialize it before processing the first
 * byte. A complete frame leaves the decoded message ID and payload available
 * to the caller.
 */
typedef struct {
    uint8_t  buf[MCT7123_BUF_SIZE];
    int      idx;

    uint8_t  msg_id;
    uint8_t  payload_len;
    uint8_t  payload[MCT7123_PAYLOAD_LEN];
} mct7123_raw_t;

/**
 * @brief Feed one UART byte into the MCT7123 frame decoder.
 *
 * @param[in,out] raw Decoder state and frame storage. The function updates
 *                    synchronization state, message metadata, and payload.
 * @param[in] data The next byte received from the serial interface.
 *
 * @retval 1 A complete frame was received and passed validation.
 * @retval 0 More bytes are required to complete a frame.
 * @retval -1 The frame length, tail byte, or CRC was invalid.
 *
 * @note The receive index is reset after a complete frame or validation
 *       failure. On success, raw->msg_id and raw->payload remain available.
 */
int mct7123_input(mct7123_raw_t *raw, uint8_t data);

/**
 * @brief Calculate the CRC16-CCITT checksum used by MCT7123 frames.
 *
 * @param[in] data Bytes included in the checksum calculation.
 * @param[in] len Number of bytes in @p data.
 *
 * @return CRC16-CCITT value calculated with polynomial 0x1021 and initial
 *         value 0xFFFF.
 *
 * @note This function does not modify the input buffer or any global state.
 */
uint16_t mct7123_crc16(const uint8_t *data, size_t len);

/**
 * @brief Decode an MCT7123 raw IMU payload.
 *
 * @param[in] payload Validated 64-byte payload from an IMU data frame.
 * @param[out] gyr_x X-axis angular velocity in degrees per second.
 * @param[out] gyr_y Y-axis angular velocity in degrees per second.
 * @param[out] gyr_z Z-axis angular velocity in degrees per second.
 * @param[out] acc_x X-axis linear acceleration reported by the device.
 * @param[out] acc_y Y-axis linear acceleration reported by the device.
 * @param[out] acc_z Z-axis linear acceleration reported by the device.
 * @param[out] mag_x X-axis magnetic-field measurement.
 * @param[out] mag_y Y-axis magnetic-field measurement.
 * @param[out] mag_z Z-axis magnetic-field measurement.
 * @param[out] temp Device temperature.
 * @param[out] systimer_us Device timestamp in microseconds; may be NULL.
 * @param[out] cycle Device cycle counter; may be NULL.
 *
 * @note All scalar output pointers except @p systimer_us and @p cycle must be
 *       valid. Values are decoded in host-native floating-point format.
 */
void mct7123_parse_imu(const uint8_t payload[64],
                       float *gyr_x, float *gyr_y, float *gyr_z,
                       float *acc_x, float *acc_y, float *acc_z,
                       float *mag_x, float *mag_y, float *mag_z,
                       float *temp, uint64_t *systimer_us, uint8_t *cycle);

/**
 * @brief Decode an MCT7123 attitude payload.
 *
 * @param[in] payload Validated 64-byte payload from an attitude frame.
 * @param[out] roll Roll angle reported by the device.
 * @param[out] pitch Pitch angle reported by the device.
 * @param[out] yaw Yaw angle reported by the device.
 * @param[out] qw Quaternion scalar component.
 * @param[out] qx Quaternion X component.
 * @param[out] qy Quaternion Y component.
 * @param[out] qz Quaternion Z component.
 * @param[out] temp Device temperature.
 * @param[out] running_status Buffer receiving the six status bytes at payload
 *                            offset 42; may be NULL.
 * @param[out] fusion_status Fusion status word at payload offset 38; may be
 *                           NULL.
 * @param[out] systimer_us Device timestamp in microseconds; may be NULL.
 * @param[out] cycle Device cycle counter; may be NULL.
 *
 * @note Angle and quaternion outputs, plus @p temp, must point to writable
 *       storage. This function only decodes data and performs no unit
 *       conversion.
 * @note Status offsets 38 and 42 preserve the layout validated by the working
 *       MCT7123 implementation in commit cd94935.
 */
void mct7123_parse_att(const uint8_t payload[64],
                       float *roll, float *pitch, float *yaw,
                       float *qw, float *qx, float *qy, float *qz,
                       float *temp, uint8_t *running_status, uint32_t *fusion_status,
                       uint64_t *systimer_us, uint8_t *cycle);

/**
 * @brief Decode an MCT7123 configuration payload.
 *
 * @param[in] payload Validated 64-byte configuration payload (message ID 0x83
 *                    or the corresponding CAN identifier).
 * @param[out] sentence_ctrl Enabled output-sentence mask; may be NULL.
 * @param[out] output_rate Configured output-rate code; may be NULL.
 * @param[out] orientation Device installation-orientation code; may be NULL.
 * @param[out] fusion_mode Sensor-fusion mode code; may be NULL.
 * @param[out] latitude Configured latitude; may be NULL.
 * @param[out] longitude Configured longitude; may be NULL.
 * @param[out] height Configured height; may be NULL.
 *
 * @note The function does not validate field ranges. Every output pointer may
 *       be NULL when that field is not required.
 */
void mct7123_parse_cfg(const uint8_t payload[64],
                       uint8_t *sentence_ctrl, uint8_t *output_rate,
                       uint8_t *orientation, uint8_t *fusion_mode,
                       double *latitude, double *longitude, float *height);

#ifdef __cplusplus
}
#endif

#endif /* __MCT7123_DEC_H__ */
