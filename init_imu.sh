#!/bin/bash

# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 Luo1imasi
# Copyright (C) 2026 wentywenty
# HiPNUC IMU (J1939) 配置修改脚本
# 使用前会尝试将主机CAN接口临时设置为500Kbps以便与出厂态IMU通信
# 修改完成后IMU变为1M，请随后将主机CAN接口恢复为1M
#
# 使用方法:
#
#   ./init_imu.sh [can_interface]
#
#   参数:
#     can_interface  CAN接口名称，默认 can_imu。
#                   例如: ./init_imu.sh can0
#
#   示例:
#     ./init_imu.sh              # 使用默认接口 can_imu
#     ./init_imu.sh can0          # 使用 can0 接口
#
#   注意事项:
#     1. 确保 IMU 已通过 CAN 总线连接至主机
#     2. 需要 root 权限（内部使用 sudo）
#     3. 需要安装 can-utils: sudo apt-get install can-utils
#     4. 执行完成后，按提示将主机 CAN 接口手动切换回 1Mbps
#
# ============================================================================
# J1939 配置协议说明
# ============================================================================
# 配置帧 CAN ID = 0x0CEF[DA][SA]  (DA=IMU节点ID, SA=主机地址)
# 默认 DA=0x08, SA=0x08 → 0x0CEF0808
#
# 数据载荷 8 字节格式:
#   [addr低8][addr高8][CMD][保留][val低8][val+8][val+16][val+24]
#   CMD: 0x06=写, 0x03=读
#
# 周期值 (uint32 LE, ms): 0=关闭, 5/10/20/50/100/200/500/1000 推荐
#                         0x00008000 = SYNC_IN 触发模式

CAN_IF=${1:-"can_imu"}
DEFAULT_BITRATE=500000

echo "================================================="
echo "  HiPNUC IMU 配置修改脚本"
echo "  当前使用接口: ${CAN_IF}"
echo "================================================="

# 尝试切换总线波特率为 500k
echo "[0/4] 正在将 ${CAN_IF} 设为 ${DEFAULT_BITRATE} bps 以连接出厂态 IMU..."
sudo ip link set ${CAN_IF} down 2>/dev/null || true
sudo ip link set ${CAN_IF} up type can bitrate ${DEFAULT_BITRATE} 2>/dev/null
if [ $? -ne 0 ]; then
    echo "[错误] 无法配置 ${CAN_IF}，请确保设备存在并具有 root 权限。"
    exit 1
fi
echo "[提示] CAN 接口已切换为 ${DEFAULT_BITRATE} bps。"
sleep 1

if ! command -v cansend &> /dev/null; then
    echo "[错误] 未找到 'cansend' 命令。请先安装 can-utils。"
    echo "Ubuntu/Debian 运行: sudo apt-get install can-utils"
    exit 1
fi

echo "[1/4] 配置IMU参数..."

# ── PGN 发送周期配置 ──────────────────────────────────────────────
# PGN 0xFF46 四元数 @ 5ms (200Hz)
cansend ${CAN_IF} 0CEF0808#4601060005000000
if [ $? -ne 0 ]; then
    echo "[错误] CAN 报文发送失败，请检查 ${CAN_IF} 接口是否处于 UP 状态且波特率匹配当前 IMU。"
    exit 1
fi
sleep 0.5

# PGN 0xFF41 航向角 → 关闭 (改用四元数推算)
cansend ${CAN_IF} 0CEF0808#4101060000000000
if [ $? -ne 0 ]; then
    echo "[错误] CAN 报文发送失败，请检查 ${CAN_IF} 接口是否处于 UP 状态且波特率匹配当前 IMU。"
    exit 1
fi
sleep 0.5

# PGN 0xFF3D 俯仰/横滚 → 关闭 (改用四元数推算)
cansend ${CAN_IF} 0CEF0808#3D01060000000000
if [ $? -ne 0 ]; then
    echo "[错误] CAN 报文发送失败，请检查 ${CAN_IF} 接口是否处于 UP 状态且波特率匹配当前 IMU。"
    exit 1
fi
sleep 0.5

echo "[2/4] 发送修改波特率为 1M 指令..."

# 0x009A CAN 波特率: 0=1000K, 1=800K, 2=500K, 3=250K, 4=125K
cansend ${CAN_IF} 0CEF0808#9A00060000000000
if [ $? -ne 0 ]; then
    echo "[错误] CAN 报文发送失败，请检查 ${CAN_IF} 接口是否处于 UP 状态且波特率匹配当前 IMU。"
    exit 1
fi
sleep 0.5

echo "[3/4] 发送保存配置指令..."
# 0x0000 保存所有参数到 Flash
cansend ${CAN_IF} 0CEF0808#0000060000000000
sleep 1.0

echo "[4/4] 发送设备复位指令..."
# 0x0000 复位 (VAL=0xFF000000)
cansend ${CAN_IF} 0CEF0808#00000600FF000000
sleep 1.5

echo "================================================="
echo "[成功] IMU 指令下发完毕！"
echo "IMU 现在应该已经以 1Mbps 的波特率运行。"
echo ""
echo "!!! 请注意 !!!"
echo "你的主机 ${CAN_IF} 接口仍停留在旧的波特率。"
echo "请手动运行以下命令，将主机的 CAN 接口切换到 1M："
echo "  sudo ip link set ${CAN_IF} down"
echo "  sudo ip link set ${CAN_IF} up type can bitrate 1000000"
echo "================================================="

# ============================================================================
# 完整 PGN 周期配置参考 (J1939 配置帧: 0x0CEF[DA][SA])
# ============================================================================
# 格式: cansend <IFACE> 0CEF0808#[addr低8][addr高8][0x06写][0x00][VAL_4字节_LE]
# 写入后需执行 保存(0x0000 VAL=0) + 复位(0x0000 VAL=0xFF) 才生效
# VAL=0 关闭; 5/10/20/50/100/200/500/1000 ms 为推荐周期; 0x00008000=SYNC_IN触发
#
# ═══ PGN 数据输出周期 ═══
#
#   PGN     寄存器   说明                         示例 (10ms=100Hz)
#   ─────────────────────────────────────────────────────────────────
#   0xFF2F  0x012F   时间信息 (UTC/运行时间)       2F0106000A000000
#   0xFF34  0x0134   三轴加速度                    340106000A000000
#   0xFF37  0x0137   三轴角速度                    370106000A000000
#   0xFF3A  0x013A   三轴磁场强度 (仅9轴产品)      3A0106000A000000
#   0xFF3D  0x013D   俯仰/横滚角                   3D0106000A000000
#   0xFF41  0x0141   航向角                        410106000A000000
#   0xFF46  0x0146   四元数                        460106000A000000
#   0xFF4A  0x014A   倾角仪输出 (倾角仪产品)       4A0106000A000000
#   0xFF43  0x0143   温度                          4301060064000000  ← 100ms 足够
#   0xFF5A  0x015A   CANFD 帧 0 (仅 CANFD 产品)    5A0106000A000000
#
# ═══ 系统控制 ═══
#
#   功能                       数据载荷
#   ─────────────────────────────────────────────────────────────────
#   全局使能节点数据输出        9D00060001000000
#   全局关闭节点数据输出        9D00060000000000
#   CAN 波特率 1000K            9A00060000000000
#   CAN 波特率 800K             9A00060001000000
#   CAN 波特率 500K             9A00060002000000
#   CAN 波特率 250K             9A00060003000000
#   CAN 波特率 125K             9A00060004000000
#   J1939 节点 ID (推荐 1-126)  9C000600[ID]000000  (例 ID=8: 9C00060008000000)
#   倾角仪 X 轴方向 默认        9E00060000000000
#   倾角仪 X 轴方向 反向        9E00060001000000
#   倾角仪 Y 轴方向 默认        9F00060000000000
#   倾角仪 Y 轴方向 反向        9F00060001000000
#   姿态控制-航向复位           A500060001000000
#   姿态控制-设置相对零点       A500060002000000
#   姿态控制-自动校平           A500060003000000
#   姿态控制-取消校平           A500060005000000
#   保存所有参数到 Flash        0000060000000000
#   恢复出厂设置 (自动保存复位) 0000060001000000
#   复位                        00000600FF000000
#
# ═══ 快速开启全部核心数据 (粘贴到终端) ═══
#
#   IF=can0
#   sudo ip link set $IF up type can bitrate 500000
#   cansend $IF 0CEF0808#340106000A000000  # 加速度 10ms
#   cansend $IF 0CEF0808#370106000A000000  # 角速度 10ms
#   cansend $IF 0CEF0808#3D0106000A000000  # 俯仰/横滚 10ms
#   cansend $IF 0CEF0808#410106000A000000  # 航向角 10ms
#   cansend $IF 0CEF0808#460106000A000000  # 四元数 10ms
#   cansend $IF 0CEF0808#3A0106000A000000  # 磁场 10ms (9轴产品)
#   cansend $IF 0CEF0808#4301060064000000  # 温度 100ms
#   cansend $IF 0CEF0808#0000060000000000  # 保存
#   cansend $IF 0CEF0808#00000600FF000000  # 复位
