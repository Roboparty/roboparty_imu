# SPDX-License-Identifier: GPL-3.0
add_library(mct7123_imu STATIC
    mct7123_dec.c
    mct7123_imu_driver.cpp
)
target_include_directories(mct7123_imu
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../hipnuc>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mct7123_imu PUBLIC ${PUBLIC_DEPENDENCIES} imu_protocol)
