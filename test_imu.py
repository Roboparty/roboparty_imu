#!/usr/bin/env python3
"""IMU 测试工具 — 只需指定型号，自动探测接口。

Usage:
  python3 test_imu.py MCT7123               # 自动探测
  python3 test_imu.py MCT7123 --can can0    # 指定 CAN 口
  python3 test_imu.py MCT7123 --serial /dev/ttyUSB0  # 指定串口
  python3 test_imu.py --list                # 列出设备
"""
import sys, os, time, argparse, glob

RAD2DEG = 57.29578

DEFAULTS = {
    'MCT7123': {'baudrate': 921600, 'iface': 'serial'},
    'HIPNUC':  {'baudrate': 115200, 'iface': 'serial'},
}

def scan_serial():
    """返回可用的串口 (ttyUSB / ttyAMA)."""
    devs = []
    for p in sorted(glob.glob('/dev/ttyUSB*')) + sorted(glob.glob('/dev/ttyAMA*')):
        if os.path.exists(p):
            devs.append(p)
    return devs

def main():
    ap = argparse.ArgumentParser(
        description='IMU 测试工具 — MCT7123 / HIPNUC',
        epilog='示例: python3 test_imu.py MCT7123',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('type', nargs='?', default=None, metavar='TYPE',
                    help='IMU 型号 (MCT7123 / HIPNUC; 不提供则提示用法)')
    ap.add_argument('--serial', metavar='DEV', help='指定串口设备')
    ap.add_argument('--can', metavar='IFACE', help='指定 CAN/CANFD 接口 (如 can0)')
    ap.add_argument('--list', action='store_true', help='列出可用设备')
    ap.add_argument('-d', '--duration', type=float, default=0,
                    help='运行时长 秒 (0=无限循环)')
    ap.add_argument('-i', '--interval', type=float, default=0.5,
                    help='打印间隔 秒 (默认 0.5)')
    ap.add_argument('-a', '--all', action='store_true',
                    help='显示全部: Gyr/Acc/Mag/Quat/Euler/Temp/Cycle')
    ap.add_argument('-q', '--quiet', action='store_true',
                    help='安静模式, 只打印速率')
    ap.add_argument('-b', '--build-dir', default=None,
                    help='build 目录 (默认 ./build)')
    args = ap.parse_args()

    if args.list:
        serials = scan_serial()
        print('串口:', ', '.join(serials) if serials else '(无)')
        print('CAN:   (使用 --can can0 指定)')
        return

    if args.type is None:
        print('请指定 IMU 型号: MCT7123 或 HIPNUC', file=sys.stderr)
        print('示例: python3 test_imu.py MCT7123', file=sys.stderr)
        sys.exit(1)

    if args.type not in ('MCT7123', 'HIPNUC'):
        print(f'无效型号: {args.type}, 可选: MCT7123, HIPNUC', file=sys.stderr)
        sys.exit(1)

    # Determine interface + device + baudrate
    imu_type = args.type
    baudrate = DEFAULTS[imu_type]['baudrate']
    iface_type = 'canfd' if (imu_type == 'MCT7123' and args.can) else \
                 'can'   if (imu_type == 'HIPNUC'  and args.can) else \
                 'serial'

    if args.serial:
        device = args.serial
    elif args.can:
        device = args.can
    else:
        # Auto-detect: try serial first
        serials = scan_serial()
        if serials:
            device = serials[0]
            iface_type = 'serial'
        else:
            print('未找到串口设备，请用 --can 或 --serial 指定', file=sys.stderr)
            sys.exit(1)

    # Silence C++ spdlog noise
    sys.stdout.flush()
    os.dup2(os.open(os.devnull, os.O_WRONLY), 2)

    sys.path.insert(0, args.build_dir or
                    os.path.join(os.path.dirname(__file__) or '.', 'build'))
    try:
        import imu_py
    except ImportError:
        print('ERROR: imu_py not found. 请先编译', flush=True)
        os._exit(1)

    forever = (args.duration <= 0)
    print(f'[{args.type}] {iface_type}:{device} {baudrate}bps '
          f'{"∞" if forever else f"{args.duration}s"}', end=' ', flush=True)
    try:
        imu = imu_py.IMUDriver.create_imu(1, iface_type, device,
                                          args.type, baudrate)
    except RuntimeError as e:
        print(f'FAIL: {e}', flush=True)
        os._exit(1)
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

    elapsed = time.time() - start
    print(f'\n{args.type} {iface_type}  {elapsed:.1f}s  {cnt} reads  {cnt/elapsed:.0f} Hz  ', end='')
    print('OK' if cnt > 0 else 'FAIL')

if __name__ == '__main__':
    main()
