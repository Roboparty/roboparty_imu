# roboparty_imu

RoboParty 双足机器人 IMU 驱动库，提供 C++ 接口和 Python 绑定，支持
MCT7123（MD7123）与 HiPNUC 系列设备。

## 1. 功能概览

| 设备 | 通信接口 | 数据帧 | 校验 |
| --- | --- | --- | --- |
| MCT7123（MD7123） | 串口、CAN FD、CANable2 SLCAN-FD | 串口为 69 字节封装帧；CAN FD 为 64 字节载荷 | CRC16-CCITT（多项式 `0x1021`） |
| HiPNUC（HI226、HI229 等） | 串口、经典 CAN（J1939） | `5A A5` 开头的变长帧或 J1939 帧 | 串口使用 CRC16；J1939 无额外应用层校验 |

主要能力：

- 同一条 CAN 总线同时接入经典 CAN 和 CAN FD 设备。
- 自动区分并路由 MCT7123 原始数据、姿态数据和配置响应。
- 支持多个驱动订阅同一 CAN 路由，并安全管理回调生命周期。
- 提供角速度、加速度、磁场、四元数、欧拉角、温度和帧计数器等数据。
- 提供命令行测试工具，可明确选择串口、SocketCAN 或 CANable2。

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

独立构建完成后，Python 模块位于 `build/` 目录。测试脚本默认从该目录加载
`imu_py`，也可以使用 `--build-dir` 指定其他构建目录。

## 3. 测试工具

测试脚本位于 `scripts/test_imu.py`。

### 默认通信参数

| 型号 | 原生串口 | CAN 仲裁速率 | CAN FD 数据速率 | 测试脚本默认节点编号 |
| --- | --- | --- | --- | --- |
| MCT7123 | `921600 bit/s` | `500 kbit/s` | `2 Mbit/s` | `1` |
| HiPNUC | `115200 bit/s` | `500 kbit/s` | 不适用 | `8` |

SocketCAN 接口需要在运行测试脚本前配置并拉起，脚本不会自动修改系统网络接口。
节点编号是设备参数，可能与表中的脚本默认值不同。测试脚本不会修改设备节点
编号；编号不一致时，仅通过 `--id` 传入设备当前编号。

MCT7123（以下使用 `can1`，避免与示例中的 HiPNUC `can0` 冲突）：

```bash
# 500 kbit/s 仲裁段，2 Mbit/s 数据段，启用 CAN FD
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 500000 dbitrate 2000000 fd on
sudo ip link set can1 up
```

HiPNUC：

```bash
# 500 kbit/s 经典 CAN，不启用 CAN FD
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

### 指定通信接口

必须同时指定型号和通信接口：

```bash
# 原生串口
python3 scripts/test_imu.py MCT7123 --serial /dev/ttyUSB0 -d 5

# SocketCAN
python3 scripts/test_imu.py MCT7123 --can can1 -d 5
python3 scripts/test_imu.py HIPNUC --can can0 --id 8 -d 5  # 对应 PGN 未启用时，默认显示项会全为 0

# CANable2 SLCAN-FD（目前仅支持 MCT7123；仅在该设备节点存在时执行）
python3 scripts/test_imu.py MCT7123 --slcan /dev/ttyACM0 -d 5
```

`--slcan` 模式直接通过 Python 读取 CANable2，需要额外安装
[`python-can`](https://python-can.readthedocs.io/)：

```bash
python3 -m pip install --user --no-warn-script-location python-can
```

CANable2 默认使用 `500 kbit/s` 仲裁速率和 `2 Mbit/s` 数据速率。脚本不会自动
选择 SLCAN 设备，需要显式传入 `--slcan`。

### 配置 HiPNUC 数据输出

测试工具的默认界面显示欧拉角和温度。如果设备只启用了加速度、角速度和
四元数 PGN，默认界面会全部显示为 `0`，但这不表示 CAN 通信或解析失败。
使用 `--all` 可以查看当前已经启用的数据：

```bash
python3 scripts/test_imu.py HIPNUC --can can0 --id 8 --all -d 5
```

下面的命令以节点 ID `8` 为例，开启俯仰/横滚、航向角和温度输出：

```bash
cansend can0 0CEF0808#3D0106000A000000  # 寄存器 0x013D：俯仰/横滚，10 ms（100 Hz）
cansend can0 0CEF0808#410106000A000000  # 寄存器 0x0141：航向角，10 ms（100 Hz）
cansend can0 0CEF0808#4301060064000000  # 寄存器 0x0143：温度，100 ms（10 Hz）
cansend can0 0CEF0808#0000060000000000  # 寄存器 0x0000，值 0：保存全部参数到 Flash
cansend can0 0CEF0808#00000600FF000000  # 寄存器 0x0000，值 0xFF：复位设备
```

配置载荷格式为
`[寄存器低字节][寄存器高字节][命令 0x06][保留][32 位小端值]`。
例如 `3D 01 06 00 0A 00 00 00` 表示向寄存器 `0x013D` 写入周期
`10 ms`。

以上命令需要安装 `can-utils`：

```bash
sudo apt-get install can-utils
```

配置成功后，总线上应出现以下 J1939 报文：

| CAN ID | PGN | 数据 | 周期 |
| --- | --- | --- | --- |
| `0x0CFF3D08` | `0xFF3D` | 俯仰角、横滚角 | 10 ms |
| `0x0CFF4108` | `0xFF41` | 航向角 | 10 ms |
| `0x0CFF4308` | `0xFF43` | 温度 | 100 ms |

配置帧 `0x0CEF0808` 中，目标地址和源地址均为 `8`。设备使用其他节点编号时，
需要相应修改配置帧 CAN ID；执行这些命令不会修改节点编号本身。航向角显示
`360°` 与 `0°` 表示同一方向，属于正常的角度回绕。

### 常用操作

```bash
# 列出检测到的原生串口、SLCAN 串口和 SocketCAN 接口
python3 scripts/test_imu.py --list

# 显示全部传感器数据
python3 scripts/test_imu.py HIPNUC --can can0 --id 8 -a -d 5

# 运行 5 秒，只输出读取速率
python3 scripts/test_imu.py MCT7123 --can can1 -d 5 -q

# 每 0.2 秒打印一次
python3 scripts/test_imu.py MCT7123 --can can1 -i 0.2 -d 5
```

默认输出仅包含欧拉角和温度：

```text
  Time         R        P        Y   Temp
   (s)       (°)      (°)      (°)   (°C)
─────────────────────────────────────────
  0.00     -0.17    -2.07   -11.47   36.2
  1.00     -0.16    -2.07   -11.47   36.2
```

当前脚本末尾显示的 `reads/s` 是 getter 轮询速率，不是传感器实际输出帧率。
判断传感器帧率应统计 CAN 报文、Cycle 计数器或时间戳变化。

### 参数

| 参数 | 说明 |
| --- | --- |
| `TYPE` | IMU 型号：`MCT7123` 或 `HIPNUC`，必须与通信接口一起指定 |
| `--serial DEV` | 使用指定的 IMU 原生串口 |
| `--can IFACE` | 使用指定的 SocketCAN 接口 |
| `--slcan DEV` | 使用指定的 CANable2 SLCAN-FD 串口 |
| `--id N` | 覆盖默认 CAN 节点编号；MCT7123 默认为 `1`，HiPNUC 默认为 `8` |
| `--list` | 列出检测到的串口设备 |
| `-d, --duration SEC` | 运行时长；`0` 表示持续运行 |
| `-i, --interval SEC` | 输出间隔，默认为 `0.5` 秒 |
| `-a, --all` | 显示全部传感器数据 |
| `-q, --quiet` | 仅输出 getter 轮询速率 |
| `-b, --build-dir DIR` | 指定包含 `imu_py` 的构建目录 |

## 4. Python API

通过 `IMUDriver.create_imu()` 创建驱动：

```python
import imu_py

imu = imu_py.IMUDriver.create_imu(
    imu_id=1,
    interface_type="serial",   # "serial"、"can" 或 "canfd"
    interface="/dev/ttyUSB0",  # 串口路径或 SocketCAN 接口名
    imu_type="MCT7123",        # "MCT7123" 或 "HIPNUC"
    baudrate=921600,           # 串口必填；CAN 和 CAN FD 模式忽略
)

angular_velocity = imu.get_ang_vel()
linear_acceleration = imu.get_lin_acc()
euler = imu.get_euler()
cycle = imu.get_cycle()
```

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

下面示例使用两个 SocketCAN 接口，和前文 OPI5 测试接线一致。设备也可以
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
