#!/usr/bin/env python3
"""IMU test reader — MCT7123 / HIPNUC, serial / CAN / CANFD.

Usage:
  python3 test_imu.py MCT7123 serial /dev/ttyUSB0 921600
  python3 test_imu.py MCT7123 canfd  can0
  python3 test_imu.py HIPNUC  serial /dev/ttyUSB0 115200
  python3 test_imu.py HIPNUC  can    can0
"""
import sys, os, time, argparse

sys.stdout.flush()
os.dup2(os.open(os.devnull, os.O_WRONLY), 2)

RAD2DEG = 57.29578

def main():
    ap = argparse.ArgumentParser(description='IMU test reader')
    ap.add_argument('type', choices=['MCT7123', 'HIPNUC'])
    ap.add_argument('interface', choices=['serial', 'can', 'canfd'])
    ap.add_argument('device')
    ap.add_argument('baudrate', type=int, nargs='?', default=0)
    ap.add_argument('-d', '--duration', type=float, default=0,
                    help='seconds (0=forever)')
    ap.add_argument('-i', '--interval', type=float, default=0.5)
    ap.add_argument('-a', '--all', action='store_true',
                    help='show all: Gyr/Acc/Mag/Quat + Euler')
    ap.add_argument('-q', '--quiet', action='store_true',
                    help='summary only')
    ap.add_argument('-b', '--build-dir', default=None)
    args = ap.parse_args()

    sys.path.insert(0, args.build_dir or
                    os.path.join(os.path.dirname(__file__) or '.', 'build'))
    try:
        import imu_py
    except ImportError:
        sys.exit('ERROR: imu_py not found. Build first.')

    forever = (args.duration <= 0)
    print(f'[{args.type}] {args.interface}:{args.device} '
          f'{"∞" if forever else args.duration}s', end=' ', flush=True)
    try:
        imu = imu_py.IMUDriver.create_imu(1, args.interface, args.device,
                                          args.type, args.baudrate)
    except RuntimeError as e:
        sys.exit(f'FAIL {e}')
    time.sleep(0.3)
    print('OK\n')

    if not args.quiet:
        if args.all:
            hdr = (f'{"Time":>6s}  {"GyrX":>7s} {"GyrY":>7s} {"GyrZ":>7s}  '
                   f'{"AccX":>7s} {"AccY":>7s} {"AccZ":>7s}  '
                   f'{"MagX":>7s} {"MagY":>7s} {"MagZ":>7s}  '
                   f'{"R":>7s} {"P":>7s} {"Y":>7s}  '
                   f'{"Qw":>7s} {"Qx":>7s} {"Qy":>7s} {"Qz":>7s}  {"Temp":>5s}')
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
                          f'{q[0]:7.3f} {q[1]:7.3f} {q[2]:7.3f} {q[3]:7.3f}  {t:5.1f}')
                else:
                    print(f'{el:6.2f}  {e[0]:8.2f} {e[1]:8.2f} {e[2]:8.2f}  {t:5.1f}')
                last = now
            time.sleep(0.0005)
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - start
    print(f'\n{args.type} {args.interface}  {elapsed:.1f}s  {cnt} reads  {cnt/elapsed:.0f} Hz  ', end='')
    print('OK' if cnt > 0 else 'FAIL')

if __name__ == '__main__':
    main()
