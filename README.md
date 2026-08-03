# roboparty_imu

**English** | [简体中文](README.zh-CN.md)

RoboParty biped robot IMU driver library with C++ APIs and Python bindings.
It supports MCT7123 (MD7123) and HiPNUC devices.

## 1. Features

| Device | Interface | Frame format | Validation |
| --- | --- | --- | --- |
| MCT7123 (MD7123) | Serial, CAN FD | 69-byte wrapped serial frames; 64-byte CAN FD payloads | CRC16-CCITT (polynomial `0x1021`) |
| HiPNUC (HI226, HI229, etc.) | Serial, classic CAN (J1939) | Variable-length frames beginning with `5A A5`, or J1939 frames | CRC16 for serial; no additional application-layer checksum for J1939 |

Main capabilities:

- Connect classic CAN and CAN FD devices through the same CAN interface.
- Distinguish and route MCT7123 raw-data, attitude, and configuration frames.
- Allow multiple drivers to subscribe to the same CAN route with managed callback lifetimes.
- Read angular velocity, linear acceleration, magnetic field, quaternion, Euler angles, temperature, timestamp, and frame counter data.

## 2. Build

### ROS 2 and colcon

The following example targets ROS 2 Humble on Ubuntu. Install the build dependencies first:

```bash
sudo apt-get update
sudo apt-get install \
  build-essential \
  cmake \
  ccache \
  python3-colcon-common-extensions \
  python3-dev \
  pybind11-dev \
  libfmt-dev \
  libspdlog-dev \
  ros-humble-ament-cmake
```

For another ROS 2 distribution, replace `humble` in
`ros-humble-ament-cmake` with the corresponding distribution name.

Place the repository under the `src/` directory of a ROS 2 workspace:

```text
imu_ws/
└── src/
    └── imu/
```

Run the following commands from the workspace root:

```bash
cd ~/imu_ws

# Replace humble with the installed ROS 2 distribution when necessary.
source /opt/ros/humble/setup.bash

# Build this package only.
colcon build --symlink-install --packages-select roboparty_imu

# Source the workspace.
source install/setup.bash
```

To confirm that ROS 2 recognizes the package:

```bash
colcon list
```

The output should include:

```text
roboparty_imu    src/imu    (ros.ament_cmake)
```

After changing the source, run `colcon build` again and source
`install/setup.bash` in each new terminal.

### Standalone CMake

A standalone build requires a C++17 compiler, CMake 3.12 or later, Python 3
development headers, pybind11, fmt, spdlog, and ccache.

Run the following commands from the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

The Python module is generated in the `build/` directory.

## 3. Communication setup

### Default parameters

| Device | Serial baud rate | CAN arbitration rate | CAN FD data rate | Common node ID |
| --- | --- | --- | --- | --- |
| MCT7123 | `921600 bit/s` | `500 kbit/s` | `2 Mbit/s` | `1` |
| HiPNUC | `115200 bit/s` | `500 kbit/s` | Not applicable | `8` |

The node ID is a device parameter. The `imu_id` passed when creating a driver
must match the device's current node ID. The values above are commonly used by
this project and are not written to the device automatically.

### SocketCAN

MCT7123 CAN FD interface:

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 500000 dbitrate 2000000 fd on
sudo ip link set can1 up
```

HiPNUC classic CAN interface:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

The driver only reads from and writes to an already configured SocketCAN
interface. It does not change the bitrate or link state automatically.

### HiPNUC J1939 output configuration

The following example enables pitch/roll, heading, and temperature output for
node ID `8`:

```bash
cansend can0 0CEF0808#3D0106000A000000  # Pitch/roll, 10 ms (100 Hz)
cansend can0 0CEF0808#410106000A000000  # Heading, 10 ms (100 Hz)
cansend can0 0CEF0808#4301060064000000  # Temperature, 100 ms (10 Hz)
cansend can0 0CEF0808#0000060000000000  # Save parameters to flash
cansend can0 0CEF0808#00000600FF000000  # Reset the device
```

The configuration payload format is
`[register low byte][register high byte][command 0x06][reserved][32-bit little-endian value]`.
For example, `3D 01 06 00 0A 00 00 00` writes a `10 ms` period to register
`0x013D`. In configuration CAN ID `0x0CEF0808`, both the destination and source
addresses are `8`; adjust the CAN ID when the device uses another node ID.

## 4. Python API

Create a driver with `IMUDriver.create_imu()`:

```python
import imu_py

imu = imu_py.IMUDriver.create_imu(
    imu_id=1,
    interface_type="serial",   # MCT7123: "serial"/"canfd"; HiPNUC: "serial"/"can"
    interface="/dev/ttyUSB0",  # Serial device path or SocketCAN interface name
    imu_type="MCT7123",        # "MCT7123" or "HIPNUC"
    baudrate=921600,            # Required for serial; ignored for CAN/CAN FD
)

angular_velocity = imu.get_ang_vel()
linear_acceleration = imu.get_lin_acc()
euler = imu.get_euler()
cycle = imu.get_cycle()
```

Supported device and interface combinations:

| Device | `interface_type` | `interface` |
| --- | --- | --- |
| MCT7123 | `serial` | Serial device path, such as `/dev/ttyUSB0` |
| MCT7123 | `canfd` | SocketCAN interface name, such as `can1` |
| HiPNUC | `serial` | Serial device path, such as `/dev/ttyUSB0` |
| HiPNUC | `can` | SocketCAN interface name, such as `can0` |

Public accessors:

| Method | Return value | Unit or description |
| --- | --- | --- |
| `get_imu_id()` | `int` | CAN node ID |
| `get_ang_vel()` | `[x, y, z]` | `rad/s` |
| `get_lin_acc()` | `[x, y, z]` | `m/s²` |
| `get_mag()` | `[x, y, z]` | `µT`; available in 9-axis mode |
| `get_quat()` | `[w, x, y, z]` | Attitude quaternion |
| `get_euler()` | `[roll, pitch, yaw]` | Degrees |
| `get_timestamp()` | `int` | Protocol timestamp converted to microseconds; `0` if unavailable |
| `get_temperature()` | `float` | Degrees Celsius |
| `get_cycle()` | `int` | MCT7123 frame counter from `0` to `255`; always `0` for HiPNUC |

Run `help(imu_py.IMUDriver)` to view the inline Python documentation.

## 5. C++ API

```cpp
#include "imu_driver.hpp"

auto imu = IMUDriver::create_imu(
    1, "serial", "/dev/ttyUSB0", "MCT7123", 921600);

std::vector<float> angular_velocity = imu->get_ang_vel();
std::vector<float> linear_acceleration = imu->get_lin_acc();
std::vector<float> magnetic_field = imu->get_mag();
std::vector<float> quaternion = imu->get_quat();
std::vector<float> euler = imu->get_euler();
uint64_t timestamp = imu->get_timestamp();
float temperature = imu->get_temperature();
uint8_t cycle = imu->get_cycle();
```

## 6. CAN and CAN FD

The following example uses two SocketCAN interfaces. The devices may share one
bus when they use the same arbitration rate and the host controller and bus
support CAN FD. The driver routes frames according to their type and length.

```python
# HiPNUC classic CAN; the J1939 source address is commonly 8.
hipnuc = imu_py.IMUDriver.create_imu(
    8, "can", "can0", "HIPNUC")

# MCT7123 CAN FD connected to can1.
mct7123 = imu_py.IMUDriver.create_imu(
    1, "canfd", "can1", "MCT7123")
```

MCT7123 uses the following CAN identifiers:

| CAN identifier | Type | Frequency |
| --- | --- | --- |
| `0x181` | Raw IMU data: gyroscope, accelerometer, and magnetometer | Determined by the device output configuration |
| `0x182` | Attitude data: Euler angles and quaternion | Determined by the device output configuration |
| `0x183` | Configuration exchange | Event-driven |

## 7. MCT7123 coordinate system

MCT7123 uses coordinate system 9 by default:

- The `+Uz` label corresponds to the positive X-axis.
- The `-Uy` label corresponds to the positive Y-axis.
- The `+Ux` label corresponds to the positive Z-axis.

For the robot installation, the front faces right and the connector faces up.
When stationary, `AccZ` is approximately `+9.81 m/s²`, representing the
reaction to gravity when the Z-axis points upward.
