#ifndef HIPNUC_J1939_PARSER_H
#define HIPNUC_J1939_PARSER_H

#include "hipnuc_can_common.h"
#include "imu_sensor_data.h"

int hipnuc_j1939_parse_frame(const hipnuc_can_frame_t *frame, imu_sensor_data_t *data);

#endif
