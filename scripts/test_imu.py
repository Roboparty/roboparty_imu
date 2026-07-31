#!/usr/bin/env python3
"""IMU 测试工具 — 必须指定型号和通信接口。

Usage:
  python3 scripts/test_imu.py MCT7123 --can can1 -d 5
  python3 scripts/test_imu.py MCT7123 --slcan /dev/ttyACM0 -d 5
  python3 scripts/test_imu.py MCT7123 --serial /dev/ttyUSB0 -d 5
  python3 scripts/test_imu.py --list
"""
import sys, os, time, argparse, glob, struct

RAD2DEG = 57.29578
DEG2RAD = 1.0 / RAD2DEG

DEFAULTS = {
    'MCT7123': {'baudrate': 921600, 'id': 1, 'can': 'can0'},
    'HIPNUC':  {'baudrate': 115200,  'id': 8, 'can': 'can0'},
}

def scan_serial():
    """返回可用的串口 (ttyUSB / ttyAMA)."""
    devs = []
    for p in sorted(glob.glob('/dev/ttyUSB*')) + sorted(glob.glob('/dev/ttyAMA*')):
        if os.path.exists(p):
            devs.append(p)
    return devs

def scan_slcan():
    """返回可能的 CANable/SLCAN 串口。"""
    return [p for p in sorted(glob.glob('/dev/ttyACM*')) if os.path.exists(p)]

def scan_can():
    """返回系统中已经注册的 SocketCAN 网络接口。"""
    interfaces = []
    for type_path in sorted(glob.glob('/sys/class/net/*/type')):
        try:
            with open(type_path, encoding='ascii') as type_file:
                if type_file.read().strip() == '280':  # Linux ARPHRD_CAN
                    interfaces.append(type_path.split('/')[-2])
        except OSError:
            continue
    return interfaces

def crc16_ccitt(data):
    crc = 0xffff
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xffff if crc & 0x8000 else (crc << 1) & 0xffff
    return crc

class MCT7123SlcanDriver:
    """通过 CANable2 的 SLCAN-FD 协议读取 MCT7123。"""

    def __init__(self, device):
        try:
            import can
        except ImportError as exc:
            raise RuntimeError('缺少 python-can，请先安装: pip install python-can') from exc

        channel = device if '@' in device else f'{device}@115200'
        self.bus = can.interface.Bus(
            interface='slcan', channel=channel, bitrate=500000,
            sleep_after_open=0.5)
        self.bus.set_bitrate(500000, data_bitrate=2000000)
        self.gyr = [0.0, 0.0, 0.0]
        self.acc = [0.0, 0.0, 0.0]
        self.mag = [0.0, 0.0, 0.0]
        self.quat = [1.0, 0.0, 0.0, 0.0]
        self.euler = [0.0, 0.0, 0.0]
        self.temperature = 0.0
        self.cycle = 0

    def poll(self, timeout=0.05):
        msg = self.bus.recv(timeout)
        if msg is None or not msg.is_fd or len(msg.data) != 64:
            return False
        payload = bytes(msg.data)
        if crc16_ccitt(payload[:62]) != struct.unpack_from('<H', payload, 62)[0]:
            return False

        can_id = msg.arbitration_id & 0x7ff
        if can_id == 0x181:
            values = struct.unpack_from('<10f', payload, 8)
            self.gyr = [v * DEG2RAD for v in values[0:3]]
            self.acc = list(values[3:6])
            self.mag = list(values[6:9])
            self.temperature = values[9]
            self.cycle = payload[61]
        elif can_id == 0x182:
            roll, pitch, yaw, qx, qy, qz, qw, temp = struct.unpack_from('<8f', payload, 8)
            self.euler = [roll, pitch, yaw]
            self.quat = [qw, qx, qy, qz]
            self.temperature = temp
        else:
            return False
        return True

    def shutdown(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass

    def get_ang_vel(self): return self.gyr
    def get_lin_acc(self): return self.acc
    def get_mag(self): return self.mag
    def get_quat(self): return self.quat
    def get_euler(self): return self.euler
    def get_temperature(self): return self.temperature
    def get_cycle(self): return self.cycle

def main():
    ap = argparse.ArgumentParser(
        description='IMU 测试工具 — MCT7123 / HIPNUC（必须指定通信接口）',
        epilog='示例: python3 scripts/test_imu.py MCT7123 --can can1 -d 5',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('type', nargs='?', default=None, metavar='TYPE',
                    help='IMU 型号 (MCT7123 / HIPNUC; 不提供则提示用法)')
    interface = ap.add_mutually_exclusive_group()
    interface.add_argument('--serial', metavar='DEV', help='指定 IMU 原生串口设备')
    interface.add_argument('--can', metavar='IFACE', help='指定 SocketCAN 接口 (如 can0)')
    interface.add_argument('--slcan', metavar='DEV',
                           help='指定 CANable2 SLCAN-FD 串口 (如 /dev/ttyACM0)')
    ap.add_argument('--list', action='store_true', help='列出可用设备')
    ap.add_argument('-d', '--duration', type=float, default=0,
                    help='运行时长 秒 (0=无限循环)')
    ap.add_argument('-i', '--interval', type=float, default=0.5,
                    help='打印间隔 秒 (默认 0.5)')
    ap.add_argument('--id', type=int, default=None,
                    help='CAN 源地址 (默认按型号: HIPNUC=8, MCT7123=1)')
    ap.add_argument('-a', '--all', action='store_true',
                    help='显示全部: Gyr/Acc/Mag/Quat/Euler/Temp/Cycle')
    ap.add_argument('-q', '--quiet', action='store_true',
                    help='安静模式, 只打印速率')
    ap.add_argument('-b', '--build-dir', default=None,
                    help='build 目录 (默认仓库根目录下的 build/)')
    args = ap.parse_args()

    if args.list:
        serials = scan_serial()
        slcans = scan_slcan()
        cans = scan_can()
        print('串口:', ', '.join(serials) if serials else '(无)')
        print('SLCAN:', ', '.join(slcans) if slcans else '(无)')
        print('CAN:  ', ', '.join(cans) if cans else '(无)')
        return

    if args.type is None:
        print('请指定 IMU 型号: MCT7123 或 HIPNUC', file=sys.stderr)
        print('示例: python3 scripts/test_imu.py MCT7123 --can can1 -d 5', file=sys.stderr)
        sys.exit(1)

    if args.type not in ('MCT7123', 'HIPNUC'):
        print(f'无效型号: {args.type}, 可选: MCT7123, HIPNUC', file=sys.stderr)
        sys.exit(1)

    if not (args.serial or args.can or args.slcan):
        print('请用 --serial、--can 或 --slcan 明确指定通信接口', file=sys.stderr)
        sys.exit(1)

    # Determine interface + device + baudrate
    imu_type = args.type
    baudrate = DEFAULTS[imu_type]['baudrate']
    iface_type = 'slcanfd' if args.slcan else \
                 'canfd' if (imu_type == 'MCT7123' and args.can) else \
                 'can'   if (imu_type == 'HIPNUC'  and args.can) else \
                 'serial'

    if args.slcan:
        if imu_type != 'MCT7123':
            print('--slcan 当前只支持 MCT7123', file=sys.stderr)
            sys.exit(1)
        device = args.slcan
        baudrate = 500000
    elif args.serial:
        device = args.serial
    elif args.can:
        device = args.can

    imu_id = args.id if args.id is not None else DEFAULTS[imu_type]['id']

    forever = (args.duration <= 0)
    rate_text = {
        'slcanfd': '500K/2M',
        'canfd': 'CAN FD',
        'can': 'CAN',
        'serial': f'{baudrate}bps',
    }[iface_type]
    print(f'[{args.type}] {iface_type}:{device} {rate_text} '
          f'{"∞" if forever else f"{args.duration}s"}', end=' ', flush=True)
    try:
        if iface_type == 'slcanfd':
            imu = MCT7123SlcanDriver(device)
        else:
            # Silence C++ spdlog noise
            sys.stdout.flush()
            os.dup2(os.open(os.devnull, os.O_WRONLY), 2)
            repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            sys.path.insert(0, args.build_dir or os.path.join(repo_root, 'build'))
            import imu_py
            imu = imu_py.IMUDriver.create_imu(imu_id, iface_type, device,
                                              args.type, baudrate)
    except (ImportError, RuntimeError, OSError) as e:
        print(f'FAIL: {e}', flush=True)
        sys.exit(1)
    time.sleep(0.3)
    print('OK\n')

    if not args.quiet:
        if args.all:
            hdr = (f'{"Time":>6s}  {"GyrX":>7s} {"GyrY":>7s} {"GyrZ":>7s}  '
                   f'{"AccX":>7s} {"AccY":>7s} {"AccZ":>7s}  '
                   f'{"MagX":>7s} {"MagY":>7s} {"MagZ":>7s}  '
                   f'{"R":>7s} {"P":>7s} {"Y":>7s}  '
                   f'{"Qw":>7s} {"Qx":>7s} {"Qy":>7s} {"Qz":>7s}  {"Temp":>5s}  Cyc')
            unt = (f'{"(s)":>6s}  {"(°/s)":>7s} {"(°/s)":>7s} {"(°/s)":>7s}  '
                   f'{"(m/s²)":>7s} {"(m/s²)":>7s} {"(m/s²)":>7s}  '
                   f'{"(uT)":>7s} {"(uT)":>7s} {"(uT)":>7s}  '
                   f'{"(°)":>7s} {"(°)":>7s} {"(°)":>7s}  '
                   f'{"":>7s} {"":>7s} {"":>7s} {"":>7s}  {"(°C)":>5s}')
        else:
            hdr = f'{"Time":>6s}  {"R":>8s} {"P":>8s} {"Y":>8s}  {"Temp":>5s}'
            unt = f'{"(s)":>6s}  {"(°)":>8s} {"(°)":>8s} {"(°)":>8s}  {"(°C)":>5s}'
        print(hdr)
        print(unt)
        print('─' * len(hdr))

    start = time.time()
    last = 0
    cnt = 0

    try:
        while forever or time.time() - start < args.duration:
            if iface_type == 'slcanfd' and not imu.poll(0.05):
                continue
            g = imu.get_ang_vel()
            a = imu.get_lin_acc()
            m = imu.get_mag()
            q = imu.get_quat()
            e = imu.get_euler()
            t = imu.get_temperature()
            c = imu.get_cycle()
            cnt += 1
            now = time.time()
            if not args.quiet and now - last >= args.interval:
                el = now - start
                if args.all:
                    gd = [v * RAD2DEG for v in g]
                    print(f'{el:6.2f}  {gd[0]:7.2f} {gd[1]:7.2f} {gd[2]:7.2f}  '
                          f'{a[0]:7.3f} {a[1]:7.3f} {a[2]:7.3f}  '
                          f'{m[0]:7.1f} {m[1]:7.1f} {m[2]:7.1f}  '
                          f'{e[0]:7.2f} {e[1]:7.2f} {e[2]:7.2f}  '
                          f'{q[0]:7.3f} {q[1]:7.3f} {q[2]:7.3f} {q[3]:7.3f}  {t:5.1f}  {c:3d}')
                else:
                    print(f'{el:6.2f}  {e[0]:8.2f} {e[1]:8.2f} {e[2]:8.2f}  {t:5.1f}')
                last = now
            time.sleep(0.0005)
    except KeyboardInterrupt:
        pass
    finally:
        if iface_type == 'slcanfd':
            imu.shutdown()

    elapsed = time.time() - start
    print(f'\n{args.type} {iface_type}  {elapsed:.1f}s  {cnt} reads  {cnt/elapsed:.0f} Hz  ', end='')
    print('OK' if cnt > 0 else 'FAIL')

if __name__ == '__main__':
    main()
