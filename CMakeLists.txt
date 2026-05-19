# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 Luo1imasi
cmake_minimum_required(VERSION 3.12)

file(READ "package.xml" PACKAGE_XML_CONTENT)
string(REGEX MATCH "<version>([0-9\\.]+)</version>" _ ${PACKAGE_XML_CONTENT})
project(roboto_imu VERSION ${CMAKE_MATCH_1})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
if(NOT CMAKE_CROSSCOMPILING)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()
set(CMAKE_CXX_COMPILER_LAUNCHER ccache)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake QUIET)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)

set(PUBLIC_DEPENDENCIES
    fmt::fmt spdlog::spdlog pthread)

add_subdirectory(src)

add_library(imu STATIC
  src/imu_driver.cpp
)

target_include_directories(imu
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(imu PUBLIC ${PUBLIC_DEPENDENCIES} hipnuc_imu imu_protocol)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS imu hipnuc_imu imu_protocol
  EXPORT roboto_imu-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(EXPORT roboto_imu-targets
  FILE roboto_imuTargets.cmake
  NAMESPACE roboto_imu::
  DESTINATION lib/cmake/roboto_imu
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/roboto_imuConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/roboto_imuConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/roboto_imuConfig.cmake"
  INSTALL_DESTINATION lib/cmake/roboto_imu
)

# Install auto-generated Config for non-ROS consumers (Deb mode)
install(FILES 
  "${CMAKE_CURRENT_BINARY_DIR}/roboto_imuConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/roboto_imuConfigVersion.cmake"
  DESTINATION lib/cmake/roboto_imu
)

pybind11_add_module(imu_py src/pybind_module.cpp)
target_include_directories(imu_py
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(imu_py PUBLIC imu)

set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")

install(TARGETS imu_py
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
  ARCHIVE DESTINATION ${PYTHON_INSTALL_DIR}
  RUNTIME DESTINATION ${PYTHON_INSTALL_DIR}
)

if(ament_cmake_FOUND)
  ament_export_libraries(imu hipnuc_imu imu_protocol)
  ament_export_include_directories(include)
  ament_package()
endif()
