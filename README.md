# roboparty_imu

RPO 双足机器人 IMU 驱动库 — C++ / Python 绑定

## 支持的 IMU

| 类型 | 接口 | 协议 |
|------|------|------|
| **MCT7123** (MD7123) | serial, canfd | EB 90 固定 64B 帧 / CANFD 64B 一体帧 |
| **HIPNUC** (HI226/HI229 等) | serial, can | HiPNUC 私有协议 (5A A5 帧头) + CAN J1939 |

## 构建

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

也可以通过 colcon 在 ROS2 workspace 中构建：`source /opt/ros/humble/setup.bash && colcon build`。

## 快速上手

```bash
# 连接 IMU 后直接运行测试脚本
python3 test_imu.py MCT7123 serial /dev/ttyUSB0 921600 -d 0

# 安静模式, 只打印速率摘要
python3 test_imu.py MCT7123 serial /dev/ttyUSB0 921600 -q

# HIPNUC
python3 test_imu.py HIPNUC serial /dev/ttyUSB0 115200
```

输出示例（默认仅欧拉角+温度）：
```
  Time         R        P        Y   Temp
   (s)       (°)      (°)      (°)   (°C)
─────────────────────────────────────────
  0.00     -0.17    -2.07   -11.47   36.2
  1.00     -0.16    -2.07   -11.47   36.2
```

`-a` 显示完整表格（陀螺/加表/磁力计/四元数+欧拉角）。

| 参数 | 说明 |
|------|------|
| `type` | `MCT7123` / `HIPNUC` |
| `interface` | `serial` / `can` / `canfd` |
| `-d 0` | 无限循环 (Ctrl+C 退出) |
| `-q` | 只打印摘要 |

## Python API

所有方法线程安全, 可通过 `help(imu_py.IMUDriver)` 查看行内文档。

| 方法 | 返回 | 说明 |
|------|------|------|
| `get_ang_vel()` | `[x, y, z]` rad/s | 角速度 |
| `get_lin_acc()` | `[x, y, z]` m/s² | 线加速度 |
| `get_mag()` | `[x, y, z]` uT | 磁场强度 (9 轴模式) |
| `get_quat()` | `[w, x, y, z]` | 姿态四元数 |
| `get_euler()` | `[roll, pitch, yaw]` ° | 欧拉角 (硬件直接输出) |
| `get_timestamp()` | `int` us | IMU 内部时钟 |
| `get_temperature()` | `float` °C | 传感器温度 |

```python
import imu_py

imu = imu_py.IMUDriver.create_imu(
    imu_id=1,
    interface_type="serial",     # "serial" | "can" | "canfd"
    interface="/dev/ttyUSB0",
    imu_type="MCT7123",          # "MCT7123" | "HIPNUC"
    baudrate=921600              # serial 必须, can/canfd 忽略
)

gyr   = imu.get_ang_vel()        # [x, y, z] rad/s
acc   = imu.get_lin_acc()        # [x, y, z] m/s²
euler = imu.get_euler()          # [roll, pitch, yaw] 度
```

## C++ API

```cpp
#include "imu_driver.hpp"

auto imu = IMUDriver::create_imu(1, "serial", "/dev/ttyUSB0", "MCT7123", 921600);

std::vector<float> gyr   = imu->get_ang_vel();       // [x,y,z] rad/s
std::vector<float> acc   = imu->get_lin_acc();       // [x,y,z] m/s²
std::vector<float> mag   = imu->get_mag();           // [x,y,z] uT
std::vector<float> quat  = imu->get_quat();          // [w,x,y,z]
std::vector<float> euler = imu->get_euler();         // [roll,pitch,yaw] 度
uint64_t         ts     = imu->get_timestamp();      // us
float            temp   = imu->get_temperature();    // °C
```

## CAN / CANFD

同一条 CAN 总线可以同时挂载 HIPNUC (经典 CAN) 和 MCT7123 (CANFD)，各自按帧长度分流。

```python
# HIPNUC 经典 CAN (≤8 字节)
imu = imu_py.IMUDriver.create_imu(0, "can", "can0", "HIPNUC")

# MCT7123 CANFD (64 字节)
imu = imu_py.IMUDriver.create_imu(1, "canfd", "can0", "MCT7123")
```

C++ 端可通过 `IMUSocketCAN::get_instance("can0")->send(frame)` 发送 CAN 帧，用于配置/命令下发。

## MCT7123 坐标系

MD7123 使用**坐标系 9**（手册默认值）：

- 标签 +Uz → X 正
- 标签 -Uy → Y 正
- 标签 +Ux → Z 正

机器人安装：正面朝右，接插件朝天。静止时 AccZ ≈ -9.81 m/s²。
