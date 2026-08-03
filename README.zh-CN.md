# roboparty_imu

[English](README.md) | **简体中文**

RoboParty 双足机器人 IMU 驱动库，提供 C++ 接口和 Python 绑定，支持
MCT7123（MD7123）与 HiPNUC 系列设备。

## 1. 功能概览

| 设备 | 通信接口 | 数据帧 | 校验 |
| --- | --- | --- | --- |
| MCT7123（MD7123） | 串口、CAN FD | 串口为 69 字节封装帧；CAN FD 为 64 字节载荷 | CRC16-CCITT（多项式 `0x1021`） |
| HiPNUC（HI226、HI229 等） | 串口、经典 CAN（J1939） | `5A A5` 开头的变长帧或 J1939 帧 | 串口使用 CRC16；J1939 无额外应用层校验 |

主要能力：

- 同一条 CAN 总线同时接入经典 CAN 和 CAN FD 设备。
- 自动区分并路由 MCT7123 原始数据、姿态数据和配置响应。
- 支持多个驱动订阅同一 CAN 路由，并安全管理回调生命周期。
- 提供角速度、加速度、磁场、四元数、欧拉角、温度和帧计数器等数据。

## 2. 构建

### 使用 ROS 2 和 colcon

以 ROS 2 Humble 和 Ubuntu 为例，先安装构建依赖：

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

使用其他 ROS 2 发行版时，将 `ros-humble-ament-cmake` 中的 `humble`
替换为对应的发行版名称。

推荐将仓库放在 ROS 2 工作区的 `src/` 目录下：

```text
imu_ws/
└── src/
    └── imu/
```

在工作区根目录执行：

```bash
cd ~/imu_ws

# 按实际安装的 ROS 2 发行版修改 humble
source /opt/ros/humble/setup.bash

# 只构建本包
colcon build --symlink-install --packages-select roboparty_imu

# 加载本工作区
source install/setup.bash
```

如果已经加载 ROS 2 环境，也可以确认包是否被正确识别：

```bash
colcon list
```

输出中应包含：

```text
roboparty_imu    src/imu    (ros.ament_cmake)
```

修改代码后重新执行 `colcon build`，并在新终端中重新加载
`install/setup.bash`。

### 使用独立 CMake

不使用 ROS 2 时，需要提前安装支持 C++17 的编译器、CMake 3.12、
Python 3 开发头文件、pybind11、fmt、spdlog 和 ccache。

在仓库根目录执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

独立构建完成后，Python 模块位于 `build/` 目录。

## 3. 通信配置

### 默认参数

| 型号 | 串口波特率 | CAN 仲裁速率 | CAN FD 数据速率 | 常用节点编号 |
| --- | --- | --- | --- | --- |
| MCT7123 | `921600 bit/s` | `500 kbit/s` | `2 Mbit/s` | `1` |
| HiPNUC | `115200 bit/s` | `500 kbit/s` | 不适用 | `8` |

节点编号是设备参数，创建驱动时传入的 `imu_id` 必须与设备当前编号一致。
上表是本项目常用值，不会由驱动自动写入设备。

### SocketCAN

MCT7123 CAN FD 接口：

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 500000 dbitrate 2000000 fd on
sudo ip link set can1 up
```

HiPNUC 经典 CAN 接口：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

驱动只读写已配置的 SocketCAN 接口，不会自动修改接口的波特率或启停状态。

### HiPNUC J1939 输出配置

以节点 ID `8` 为例，以下命令开启俯仰/横滚、航向角和温度输出：

```bash
cansend can0 0CEF0808#3D0106000A000000  # 俯仰/横滚，10 ms（100 Hz）
cansend can0 0CEF0808#410106000A000000  # 航向角，10 ms（100 Hz）
cansend can0 0CEF0808#4301060064000000  # 温度，100 ms（10 Hz）
cansend can0 0CEF0808#0000060000000000  # 保存参数到 Flash
cansend can0 0CEF0808#00000600FF000000  # 复位设备
```

配置载荷格式为
`[寄存器低字节][寄存器高字节][命令 0x06][保留][32 位小端值]`。
例如 `3D 01 06 00 0A 00 00 00` 表示向寄存器 `0x013D` 写入
`10 ms` 周期。配置 CAN ID `0x0CEF0808` 中的目标地址和源地址均为
`8`；设备使用其他节点编号时需要相应修改 CAN ID。

## 4. Python API

通过 `IMUDriver.create_imu()` 创建驱动：

```python
import imu_py

imu = imu_py.IMUDriver.create_imu(
    imu_id=1,
    interface_type="serial",   # MCT7123: "serial"/"canfd"; HiPNUC: "serial"/"can"
    interface="/dev/ttyUSB0",  # 串口路径或 SocketCAN 接口名
    imu_type="MCT7123",        # "MCT7123" 或 "HIPNUC"
    baudrate=921600,           # 串口必填；CAN 和 CAN FD 模式忽略
)

angular_velocity = imu.get_ang_vel()
linear_acceleration = imu.get_lin_acc()
euler = imu.get_euler()
cycle = imu.get_cycle()
```

型号与通信接口的对应关系：

| 型号 | `interface_type` | `interface` |
| --- | --- | --- |
| MCT7123 | `serial` | 串口设备路径，如 `/dev/ttyUSB0` |
| MCT7123 | `canfd` | SocketCAN 接口名，如 `can1` |
| HiPNUC | `serial` | 串口设备路径，如 `/dev/ttyUSB0` |
| HiPNUC | `can` | SocketCAN 接口名，如 `can0` |

公共读取接口：

| 方法 | 返回值 | 单位或说明 |
| --- | --- | --- |
| `get_imu_id()` | `int` | CAN 节点编号 |
| `get_ang_vel()` | `[x, y, z]` | `rad/s` |
| `get_lin_acc()` | `[x, y, z]` | `m/s²` |
| `get_mag()` | `[x, y, z]` | `µT`，九轴模式下有效 |
| `get_quat()` | `[w, x, y, z]` | 姿态四元数 |
| `get_euler()` | `[roll, pitch, yaw]` | 度 |
| `get_timestamp()` | `int` | 协议时间戳换算为微秒；设备未输出时间戳时为 `0` |
| `get_temperature()` | `float` | 摄氏度 |
| `get_cycle()` | `int` | MCT7123 的 `0`–`255` 帧计数器；HiPNUC 固定返回 `0` |

可通过 `help(imu_py.IMUDriver)` 查看 Python 行内文档。

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

## 6. CAN 与 CAN FD

下面示例使用两个 SocketCAN 接口。设备也可以
接在同一条总线上，但必须使用相同仲裁速率，且主机控制器和总线都支持
CAN FD。驱动按照帧类型和长度分别路由数据。

```python
# HiPNUC 经典 CAN；J1939 源地址通常为 8
hipnuc = imu_py.IMUDriver.create_imu(
    8, "can", "can0", "HIPNUC")

# MCT7123 CAN FD；当前示例连接在 can1
mct7123 = imu_py.IMUDriver.create_imu(
    1, "canfd", "can1", "MCT7123")
```

MCT7123 使用以下 CAN 标识：

| CAN 标识 | 类型 | 频率 |
| --- | --- | --- |
| `0x181` | IMU 原始数据：陀螺仪、加速度计、磁力计 | 取决于设备输出配置 |
| `0x182` | 姿态数据：欧拉角、四元数 | 取决于设备输出配置 |
| `0x183` | 配置交互 | 事件触发 |

## 7. MCT7123 坐标系

MCT7123 默认使用坐标系 9：

- 标签 `+Uz` 对应 X 轴正方向。
- 标签 `-Uy` 对应 Y 轴正方向。
- 标签 `+Ux` 对应 Z 轴正方向。

机器人安装方向为正面朝右、接插件朝天。静止时 `AccZ` 约为
`+9.81 m/s²`，表示 Z 轴朝上时测得的重力反作用力。
