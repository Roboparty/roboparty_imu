# roboparty_imu

RPO 双足机器人 IMU 驱动库 — C++ / Python 绑定

## 支持的 IMU

| 类型 | 接口 | 帧格式 | CRC |
|------|------|--------|-----|
| **MCT7123** (MD7123) | serial, canfd | EB 90 固定 64B + ED | CCITT (poly 0x1021) |
| **HIPNUC** (HI226/HI229 等) | serial, can | 5A A5 变长 + CAN J1939 | 无 |

## 构建

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## 快速上手

```bash
# 连接 IMU 后直接运行 (默认无限循环)
python3 test_imu.py MCT7123 serial /dev/ttyUSB0 921600

# 仅欧拉角 + 温度 (默认输出)
python3 test_imu.py MCT7123 serial /dev/ttyUSB0 921600 -d 5

# 全部数据: 陀螺/加表/磁力计/四元数/欧拉角/温度/Cycle
python3 test_imu.py MCT7123 serial /dev/ttyUSB0 921600 -a

# 安静模式, 只打速率
python3 test_imu.py MCT7123 serial /dev/ttyUSB0 921600 -d 5 -q

# HIPNUC
python3 test_imu.py HIPNUC serial /dev/ttyUSB0 115200
```

输出示例 (默认仅欧拉角+温度):

```
  Time         R        P        Y   Temp
   (s)       (°)      (°)      (°)   (°C)
─────────────────────────────────────────
  0.00     -0.17    -2.07   -11.47   36.2
  1.00     -0.16    -2.07   -11.47   36.2
```

`-a` 完整输出 (含 Cycle 计数器):

```
  Time     GyrX    GyrY    GyrZ     AccX    AccY    AccZ     MagX    MagY    MagZ        R       P       Y       Qw      Qx      Qy      Qz   Temp  Cyc
   (s)    (°/s)   (°/s)   (°/s)   (m/s²)  (m/s²)  (m/s²)     (uT)    (uT)    (uT)      (°)     (°)     (°)                                    (°C)
───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
  0.00     0.00   -0.01    0.04   -0.029  -0.048   9.810      0.0     0.0     0.0    -0.22   -0.12  -51.32    0.901  -0.002  -0.000  -0.433   35.4  215
  1.50     0.03    0.03   -0.00   -0.027  -0.041   9.811      0.0     0.0     0.0    -0.22   -0.12  -51.32    0.901  -0.002  -0.000  -0.433   35.4    2
```

| 参数 | 说明 |
|------|------|
| `type` | `MCT7123` / `HIPNUC` |
| `interface` | `serial` / `can` / `canfd` |
| `-d SECONDS` | 运行时长, 0=无限 (Ctrl+C 退出) |
| `-a` | 显示全部 20 列 (Gyr/Acc/Mag/Quat/Euler/Temp/Cycle) |
| `-q` | 安静模式, 只打速率摘要 |

## Python API

所有方法线程安全 (`shared_mutex`), 可通过 `help(imu_py.IMUDriver)` 查看行内文档。

| 方法 | 返回 | 说明 |
|------|------|------|
| `get_ang_vel()` | `[x, y, z]` rad/s | 角速度 (陀螺仪) |
| `get_lin_acc()` | `[x, y, z]` m/s² | 线加速度 |
| `get_mag()` | `[x, y, z]` uT | 磁场强度 (9 轴模式) |
| `get_quat()` | `[w, x, y, z]` | 姿态四元数 |
| `get_euler()` | `[roll, pitch, yaw]` ° | 欧拉角 (硬件直接输出) |
| `get_timestamp()` | `int` us | IMU 内部时钟 (开机累积) |
| `get_temperature()` | `float` °C | 传感器温度 |
| `get_cycle()` | `int` 0-255 | 帧计数器 (检测丢帧, MCT7123 专用) |

```python
import imu_py

imu = imu_py.IMUDriver.create_imu(
    imu_id=1,                    # 节点 ID (CAN 模式下回调路由)
    interface_type="serial",     # "serial" | "can" | "canfd"
    interface="/dev/ttyUSB0",    # 设备路径
    imu_type="MCT7123",          # "MCT7123" | "HIPNUC"
    baudrate=921600              # serial 必须; can/canfd 忽略
)

# 读取数据
gyr   = imu.get_ang_vel()        # [x, y, z] rad/s
acc   = imu.get_lin_acc()        # [x, y, z] m/s²
euler = imu.get_euler()          # [roll, pitch, yaw] 度
cycle = imu.get_cycle()          # 帧计数 (丢帧检测)
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
uint8_t          cycle  = imu->get_cycle();          // 0-255
```

## CAN / CANFD

同一条 CAN 总线可同时挂载 HIPNUC (经典 CAN, ≤8 字节) 和 MCT7123 (CANFD, 64 字节), 各自按帧长度分流。

```python
# HIPNUC 经典 CAN (J1939)
imu_h = imu_py.IMUDriver.create_imu(0, "can", "can0", "HIPNUC")

# MCT7123 CANFD (0x181=IMU, 0x182=Att, 0x183=Config)
imu_m = imu_py.IMUDriver.create_imu(1, "canfd", "can0", "MCT7123")
```

C++ 端可通过 `IMUSocketCAN::get_instance("can0")->send(frame)` 发送 CAN 帧。

## MCT7123 坐标系

MD7123 使用**坐标系 9**（手册默认值）：

- 标签 +Uz → X 正
- 标签 -Uy → Y 正
- 标签 +Ux → Z 正

机器人安装：正面朝右，接插件朝天。静止时 AccZ ≈ +9.81 m/s² (重力反作用力，Z 轴朝上)。

## 协议参考

| 帧 ID | 类型 | 频率 |
|--------|------|------|
| 0x81 | IMU 原始数据 (陀螺/加表/磁力计) | 100 Hz |
| 0x82 | 姿态数据 (欧拉角/四元数) | 100 Hz |
| 0x83 | 配置交互 | 事件触发 |
