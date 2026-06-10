file(GLOB_RECURSE PROTOCOL_SRCS *.cpp)

add_library(imu_protocol
        STATIC
        ${PROTOCOL_SRCS}
)
target_include_directories(imu_protocol
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/can>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/serial>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(imu_protocol PUBLIC ${PUBLIC_DEPENDENCIES})