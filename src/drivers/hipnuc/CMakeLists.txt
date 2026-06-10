# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 Luo1imasi
AUX_SOURCE_DIRECTORY(. SOURCE_LIST_HIPNUC_IMU)
add_library(hipnuc_imu
        STATIC
        ${SOURCE_LIST_HIPNUC_IMU}
)
target_include_directories(hipnuc_imu
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hipnuc_imu PUBLIC ${PUBLIC_DEPENDENCIES} imu_protocol)