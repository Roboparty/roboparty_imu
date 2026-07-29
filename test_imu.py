#!/usr/bin/env python3
"""IMU test reader — MCT7123 / HIPNUC, serial / CAN / CANFD.

Usage:
  python3 test_imu.py MCT7123 serial /dev/ttyUSB0 921600
  python3 test_imu.py MCT7123 canfd  can0
  python3 test_imu.py HIPNUC  serial /dev/ttyUSB0 115200
  python3 test_imu.py HIPNUC  can    can0

Options: -d SEC (0=forever)  -i SEC  -q  -b BUILD_DIR
"""
import sys, os, time, argparse

# Silence spdlog noise on stderr
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
                    help='seconds (0=forever, Ctrl+C to stop)')
    ap.add_argument('-i', '--interval', type=float, default=0.5)
    ap.add_argument('-q', '--quiet', action='store_true')
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
            ts = imu.get_timestamp()
            cnt += 1
            now = time.time()
            if not args.quiet and now - last >= args.interval:
                el = now - start
                print(f'── {el:6.1f}s  ts {ts/1000:8.0f}ms')
                print(f'  GyrX {g[0]*RAD2DEG:7.2f}  GyrY {g[1]*RAD2DEG:7.2f}  GyrZ {g[2]*RAD2DEG:7.2f}  °/s')
                print(f'  AccX {a[0]:7.3f}  AccY {a[1]:7.3f}  AccZ {a[2]:7.3f}  m/s²')
                print(f'  MagX {m[0]:7.1f}  MagY {m[1]:7.1f}  MagZ {m[2]:7.1f}  uT')
                print(f'  R {e[0]:7.2f}  P {e[1]:7.2f}  Y {e[2]:7.2f}  °')
                print(f'  Qw {q[0]:7.3f}  Qx {q[1]:7.3f}  Qy {q[2]:7.3f}  Qz {q[3]:7.3f}  Temp {t:5.1f}°C')
                last = now
            time.sleep(0.0005)
    except KeyboardInterrupt:
        pass

    elapsed = time.time() - start
    print(f'\n{args.type} {args.interface}  {elapsed:.1f}s  {cnt} reads  {cnt/elapsed:.0f} Hz  ', end='')
    print('OK' if cnt > 0 else 'FAIL')

if __name__ == '__main__':
    main()
