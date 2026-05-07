#!/bin/bash

# HiPNUC IMU (J1939) 配置修改脚本
# 使用前会尝试将主机CAN接口临时设置为500Kbps以便与出厂态IMU通信
# 修改完成后IMU变为1M，请随后将主机CAN接口恢复为1M

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

cansend ${CAN_IF} 0CEF0808#4601060005000000
if [ $? -ne 0 ]; then
    echo "[错误] CAN 报文发送失败，请检查 ${CAN_IF} 接口是否处于 UP 状态且波特率匹配当前 IMU。"
    exit 1
fi
sleep 0.5 

cansend ${CAN_IF} 0CEF0808#4101060000000000
if [ $? -ne 0 ]; then
    echo "[错误] CAN 报文发送失败，请检查 ${CAN_IF} 接口是否处于 UP 状态且波特率匹配当前 IMU。"
    exit 1
fi
sleep 0.5 

cansend ${CAN_IF} 0CEF0808#3D01060000000000
if [ $? -ne 0 ]; then
    echo "[错误] CAN 报文发送失败，请检查 ${CAN_IF} 接口是否处于 UP 状态且波特率匹配当前 IMU。"
    exit 1
fi
sleep 0.5 

echo "[2/4] 发送修改波特率为 1M 指令..."

cansend ${CAN_IF} 0CEF0808#9A00060000000000
if [ $? -ne 0 ]; then
    echo "[错误] CAN 报文发送失败，请检查 ${CAN_IF} 接口是否处于 UP 状态且波特率匹配当前 IMU。"
    exit 1
fi
sleep 0.5 

echo "[3/4] 发送保存配置指令..."
cansend ${CAN_IF} 0CEF0808#0000060000000000
sleep 1.0 

echo "[4/4] 发送设备复位指令..."
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