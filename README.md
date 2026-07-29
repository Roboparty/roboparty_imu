# roboparty_imu

RPO 双足机器人 IMU 驱动库 — C++ / Python 绑定

## 支持的 IMU

| 类型 | 接口 | 协议 |
|------|------|------|
| **HIPNUC** (HI226/HI229 等) | serial, can | HiPNUC 私有协议 (5A A5 帧头) + CAN J1939 |
| **MCT7123** (MD7123) | serial, canfd | EB 90 固定 64B 帧 / CANFD 64B 一体帧 |

## Python 测试脚本

```bash
# MCT7123 串口 (默认 921600bps)
python3 test_imu.py MCT7123 serial /dev/ttyUSB0 921600

# MCT7123 CANFD
python3 test_imu.py MCT7123 canfd can0

# 仅打摘要，安静模式
python3 test_imu.py MCT7123 serial /dev/ttyUSB0 921600 -d 5 -q

# 无限循环，Ctrl+C 退出
python3 test_imu.py MCT7123 serial /dev/ttyUSB0 921600 -d 0

# HIPNUC 串口
python3 test_imu.py HIPNUC serial /dev/ttyUSB0 115200

# HIPNUC 经典 CAN
python3 test_imu.py HIPNUC can can0
```

参数:

```
python3 test_imu.py -h
  type            MCT7123 | HIPNUC
  interface       接口: serial | can | canfd
  device          设备路径 (/dev/ttyUSB0, can0 等)
  baudrate        波特率 (serial 必须; can/canfd 忽略)
  -d SECONDS      运行时长 (0=无限循环, Ctrl+C 退出)
  -i SECONDS      打印间隔 (默认 0.5s)
  -q              安静模式, 只打印摘要
```

## Python API

```python
import imu_py

# 创建 IMU 实例
imu = imu_py.IMUDriver.create_imu(
    imu_id=1,                    # 节点 ID (CAN 模式用于多设备路由)
    interface_type="serial",     # "serial" | "can" | "canfd"
    interface="/dev/ttyUSB0",    # 设备路径
    imu_type="MCT7123",          # "MCT7123" | "HIPNUC"
    baudrate=921600              # serial 必须, can/canfd 忽略
)

# 读取数据 (线程安全)
gyr   = imu.get_ang_vel()        # [x, y, z] rad/s   角速度
acc   = imu.get_lin_acc()        # [x, y, z] m/s²   线加速度
mag   = imu.get_mag()            # [x, y, z] uT     磁场强度 (9 轴模式)
quat  = imu.get_quat()           # [w, x, y, z]     姿态四元数
euler = imu.get_euler()          # [roll, pitch, yaw] 度
ts    = imu.get_timestamp()      # us               系统时间戳
temp  = imu.get_temperature()    # °C               传感器温度
```

> `help(imu_py.IMUDriver)` 查看完整文档。

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

## MCT7123 坐标系说明

MD7123 使用**坐标系 9**（手册默认值）：

- 标签 +Uz → X 正
- 标签 -Uy → Y 正
- 标签 +Ux → Z 正

机器人安装：正面朝右，接插件朝天。静止时 AccZ ≈ -9.81 m/s²（重力）。
