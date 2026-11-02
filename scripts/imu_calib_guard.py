#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 RoboParty

# IMU 标定矩阵(ACCMIS/ACCB/GYRMIS/GYRB)备份与恢复工具。
#
#   python imu_calib_guard.py backup  --port COM3 [--out DIR]
#   python imu_calib_guard.py restore --file imu_calib_XXXX.json --port COM3
#   python imu_calib_guard.py update  --firmware fw.hex --port COM3 \
#       --interface can0 --node 8
#
# backup:  发送 LOG IMUCONFIG, 解析 4 个矩阵块, 落盘 JSON(临时持久化)并驻留内存。
# restore: 按行优先发 FCAL 写指令 + SAVECONFIG, 读回 LOG IMUCONFIG 比对;
#          不一致的块自动重写, 每轮只重写没对上的块, 最多 --attempts 轮。
# update:  backup -> CAN 刷写(复用 hipnuc_ota) -> 等待上线 -> restore。
#
# 标定指令(FCAL/LOG IMUCONFIG/SAVECONFIG)是设备 ASCII 串口 CLI 语言;
# CAN 侧只承载 HIREG 寄存器读写与 SDO bootloader, 刷写走 CAN(--interface)。
#
# 写入参数名按固件参数表映射: ACCMIS->WMIS, ACCB->WB, GYRMIS->GMIS, GYRB->GB
# (FCAL GMIS 已由指令格式确认, 其余为固件参数注册表推断, 首次运行读回比对即验证)。

"""IMU calibration-matrix backup and restore with rewrite-on-mismatch."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

BLOCK_KEYS = ("ACCMIS", "ACCB", "GYRMIS", "GYRB")
BLOCK_SIZE = {"ACCMIS": 9, "ACCB": 3, "GYRMIS": 9, "GYRB": 3}
# LOG IMUCONFIG 显示名 -> FCAL 写指令参数名(固件参数注册表: GMIS/GB/WMIS/WB)。
WRITE_NAME = {"ACCMIS": "WMIS", "ACCB": "WB", "GYRMIS": "GMIS", "GYRB": "GB"}
DEFAULT_TOLERANCE = 1e-3  # LOG 显示 3 位小数, 同值读回应当逐位一致
DEFAULT_ATTEMPTS = 3


class CalibError(RuntimeError):
    """Device rejected a command, or the backup is unusable."""


class VerifyError(CalibError):
    """Matrices still mismatched after all rewrite attempts."""


# ---------------------------------------------------------------------------
# 解析与生成
# ---------------------------------------------------------------------------


def parse_imu_config(text: str) -> dict[str, list[float]]:
    """解析 LOG IMUCONFIG 回复 -> {KEY: 展平浮点列表}。

    每个 KEY= 行开一个块, 后续纯数值行按行序收齐;
    回显行、OK、其余杂行跳过。数量不符即抛 CalibError。
    """
    rows: dict[str, list[float]] = {}
    key: Optional[str] = None
    for line in text.splitlines():
        line = line.strip()
        if not line or line == "OK":
            continue
        if "=" in line:
            name, _, rest = line.partition("=")
            name = name.strip()
            if name in BLOCK_KEYS:
                key = name
                rows[key] = []
                if rest.strip():
                    rows[key].extend(_numbers(rest))
            else:
                key = None  # 未知 KEY 块(如未来新增磁力计)整块跳过
            continue
        if key is None:
            continue  # 命令回显等杂行
        rows[key].extend(_numbers(line))
    missing = [k for k in BLOCK_KEYS if k not in rows]
    if missing:
        raise CalibError(f"LOG IMUCONFIG 缺少数据块: {missing}")
    for name, values in rows.items():
        if len(values) != BLOCK_SIZE[name]:
            raise CalibError(
                f"{name} 数量不符: 期望 {BLOCK_SIZE[name]}, 实得 {len(values)}({values})"
            )
    return {name: rows[name] for name in BLOCK_KEYS}


def _numbers(line: str) -> list[float]:
    try:
        return [float(p) for p in re.split(r"[\s,]+", line) if p]
    except ValueError as exc:
        raise CalibError(f"无法解析数值行: {line!r}") from exc


def fmt_num(value: float) -> str:
    """写回格式: 最多 6 位小数去尾零(设备 %f 解析; 3 位小数来源时逐位一致)。"""
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return "0" if text in ("", "-", "-0") else text


def write_command(key: str, values: list[float]) -> str:
    """一条 FCAL 写指令, 值按行序展平逗号分隔。"""
    return f"FCAL {WRITE_NAME[key]} " + ",".join(fmt_num(v) for v in values)


def build_write_commands(cfg: dict[str, list[float]]) -> list[str]:
    """全部写回指令(含收尾 SAVECONFIG)。"""
    return [write_command(k, cfg[k]) for k in BLOCK_KEYS] + ["SAVECONFIG"]


def compare(
    a: dict[str, list[float]], b: dict[str, list[float]], tol: float
) -> dict[str, list[tuple[int, float, float]]]:
    """逐元素比对 -> {块名: [(下标, 期望, 实得), ...]}, 全对返回空 dict。"""
    diffs: dict[str, list[tuple[int, float, float]]] = {}
    for name in BLOCK_KEYS:
        va, vb = a[name], b[name]
        if len(va) != len(vb):
            diffs[name] = [(i, x, float("nan")) for i, x in enumerate(va)]
            continue
        bad = [(i, x, y) for i, (x, y) in enumerate(zip(va, vb)) if abs(x - y) > tol]
        if bad:
            diffs[name] = bad
    return diffs


def format_diffs(diffs: dict[str, list[tuple[int, float, float]]]) -> str:
    parts = []
    for name, bad in diffs.items():
        shape = BLOCK_SIZE[name]
        for i, want, got in bad:
            pos = f"[{i // 3 + 1},{i % 3 + 1}]" if shape == 9 else f"[{i + 1}]"
            parts.append(f"{name}{pos}: 期望 {want} 实得 {got}")
    return "; ".join(parts)


# ---------------------------------------------------------------------------
# 备份持久化
# ---------------------------------------------------------------------------


@dataclass
class Backup:
    blocks: dict[str, list[float]]
    path: Optional[Path] = None
    device: dict = field(default_factory=dict)
    raw_text: str = ""


def save_backup(backup: Backup, out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    sn = backup.device.get("serial_number") or "unknown"
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = out_dir / f"imu_calib_{sn}_{stamp}.json"
    path.write_text(
        json.dumps(
            {
                "format": "imu-calib-v1",
                "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "device": backup.device,
                "blocks": backup.blocks,
                "raw_text": backup.raw_text,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    backup.path = path
    return path


def load_backup(path: Path) -> Backup:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("format") != "imu-calib-v1":
        raise CalibError(f"备份格式不识别: {data.get('format')!r} ({path})")
    blocks = parse_imu_config(data.get("raw_text", "")) if data.get("raw_text") else None
    # raw_text 不可用时直接信任 blocks(手工编辑过也要能恢复)
    if not blocks:
        blocks = {}
        for name in BLOCK_KEYS:
            values = [float(v) for v in data["blocks"][name]]
            if len(values) != BLOCK_SIZE[name]:
                raise CalibError(f"备份 {name} 数量不符: {len(values)}")
            blocks[name] = values
    return Backup(blocks=blocks, path=path, device=data.get("device", {}),
                  raw_text=data.get("raw_text", ""))


# ---------------------------------------------------------------------------
# 指令通道: 设备 ASCII 串口 CLI
# ---------------------------------------------------------------------------


class SerialCli:
    """一行指令 -> 文本回复(至 OK)。二进制测量帧字节会污染行, 逐行丢弃。

    语义对齐官方 SDK SerialDevice.command(): 回显行跳过, ERROR/ERR/FAIL 抛错,
    OK 结束; 超时视为指令未确认。
    """

    def __init__(self, port: str, baud: int = 115200, timeout: float = 3.0):
        import serial  # 惰性导入: 只有实际连设备才需要 pyserial

        self.port, self.baud, self.timeout = port, baud, timeout
        self._serial = serial.Serial(port, baud, timeout=0.05, write_timeout=timeout)

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def reopen(self) -> None:
        """刷写后 USB 串口可能重新枚举, 恢复前重开一次。"""
        self.close()
        import serial

        self._serial = serial.Serial(self.port, self.baud, timeout=0.05,
                                     write_timeout=self.timeout)

    def command(self, command: str) -> str:
        if self._serial is None:
            raise CalibError("串口已关闭")
        self._serial.reset_input_buffer()
        self._serial.write((command + "\r\n").encode("ascii"))
        deadline = time.monotonic() + self.timeout
        lines: list[str] = []
        buf = bytearray()
        invalid = False
        while time.monotonic() < deadline:
            for value in self._serial.read(256):
                if value == 10:  # \n
                    if not invalid:
                        line = buf.decode("ascii", errors="replace").strip()
                        if line and line != command:
                            if re.match(r"^(ERROR|ERR|FAIL)(?:$|[\s:])", line, re.I):
                                raise CalibError(f"设备拒绝 {command!r}: {line}")
                            lines.append(line)
                            if line == "OK":
                                return "\n".join(lines)
                    buf.clear()
                    invalid = False
                elif value in (9, 13) or 32 <= value <= 126:
                    if not invalid:
                        if len(buf) < 1024:
                            buf.append(value)
                        else:
                            invalid = True
                else:  # 二进制帧/噪声: 整行作废
                    invalid = True
                    buf.clear()
        raise CalibError(f"{command!r} 在 {self.timeout:g}s 内未收到 OK 回复")


# ---------------------------------------------------------------------------
# 备份 / 恢复(核心: 不一致自动重写)
# ---------------------------------------------------------------------------


def read_config(cli: SerialCli) -> Backup:
    raw = cli.command("LOG IMUCONFIG")
    blocks = parse_imu_config(raw)
    device = {}
    try:
        info = cli.command("LOG VERSION")
        device = dict(
            re.findall(r"^(\w+)=(.*)$", info, re.M)
        ) or {"raw": info}
    except CalibError:
        pass  # 身份查询失败不阻塞备份
    return Backup(blocks=blocks, device=device, raw_text=raw)


def restore(
    cli: SerialCli,
    target: dict[str, list[float]],
    *,
    attempts: int = DEFAULT_ATTEMPTS,
    tolerance: float = DEFAULT_TOLERANCE,
    log: Callable[[str], None] = print,
) -> int:
    """写回 4 块并读回验证; 只重写没对上的块, 返回实际轮数。

    第 1 轮写全部块; 之后每轮只写上一轮比对失败的块, 每轮结束 SAVECONFIG
    后重新 LOG IMUCONFIG 比对。全部一致即成功返回轮数;
    attempts 轮后仍不一致抛 VerifyError(附逐元素差异)。
    """
    pending = list(BLOCK_KEYS)
    diffs: dict[str, list[tuple[int, float, float]]] = {}
    for round_no in range(1, attempts + 1):
        log(f"[restore] 第 {round_no} 轮: 写入 {pending}")
        for key in pending:
            cmd = write_command(key, target[key])
            cli.command(cmd)
            log(f"[restore]   > {cmd}")
        cli.command("SAVECONFIG")
        current = parse_imu_config(cli.command("LOG IMUCONFIG"))
        diffs = compare(current, target, tolerance)
        if not diffs:
            log(f"[restore] 读回比对通过(第 {round_no} 轮)")
            return round_no
        log(f"[restore] 未对上: {format_diffs(diffs)}")
        pending = list(diffs)  # 只重写不一致的块
    raise VerifyError(
        f"{attempts} 轮重写后仍不一致: {format_diffs(diffs)}"
    )


# ---------------------------------------------------------------------------
# 刷写编排(复用 hipnuc_ota 的 CAN bootloader 流程)
# ---------------------------------------------------------------------------


def run_flash(args: argparse.Namespace) -> None:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import hipnuc_ota as ota

    image_path = Path(args.firmware)
    image = ota.load_image(image_path, raw_binary=args.raw_binary)
    ota.log(f"Firmware: {image_path} ({len(image.data)} bytes)")
    with ota.open_bus(args.interface) as bus:
        before = ota.read_info(bus, args.node, timeout=args.timeout)
        ota.log(f"Device: product={before.product_name} fw={before.firmware_version} "
                f"sn={before.serial_number}")
        ota.check_model_compatibility(before, image_path, force=args.force)
        result = ota.update_can(
            bus, args.node, image_path,
            raw_binary=args.raw_binary,
            progress=ota.make_progress_logger(len(image.data)),
        )
        ota.log(f"Transfer done: {result.bytes_written} bytes, "
                f"verified_version={result.verified_version}")
        ota.log("Waiting for application mode ...")
        after = ota.wait_for_node(
            bus, args.node,
            recovery_timeout=args.recovery_timeout, timeout=args.timeout,
        )
        ota.log(f"Device after update: fw={after.firmware_version} sn={after.serial_number}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def cmd_backup(args: argparse.Namespace) -> int:
    cli = SerialCli(args.port, args.baud)
    try:
        backup = read_config(cli)
    finally:
        cli.close()
    path = save_backup(backup, Path(args.out))
    for name in BLOCK_KEYS:
        print(f"{name}: {backup.blocks[name]}")
    print(f"backup_file: {path}")
    return 0


def cmd_restore(args: argparse.Namespace) -> int:
    backup = load_backup(Path(args.file))
    cli = SerialCli(args.port, args.baud)
    try:
        rounds = restore(cli, backup.blocks,
                         attempts=args.attempts, tolerance=args.tolerance)
    except VerifyError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        print(f"[FAIL] 备份保留在 {backup.path}, 可修复后重跑 restore",
              file=sys.stderr)
        return 1
    finally:
        cli.close()
    print(f"restore OK ({rounds} 轮)")
    return 0


def cmd_update(args: argparse.Namespace) -> int:
    cli = SerialCli(args.port, args.baud)
    try:
        backup = read_config(cli)  # 1. 备份: 内存 + 落盘
    finally:
        cli.close()
    path = save_backup(backup, Path(args.out))
    print(f"backup_file: {path}")

    run_flash(args)  # 2. CAN 刷写

    cli = SerialCli(args.port, args.baud)
    try:
        try:
            cli.reopen()  # 刷写可能触发 USB 重新枚举
        except Exception:
            pass
        try:
            rounds = restore(cli, backup.blocks,
                             attempts=args.attempts, tolerance=args.tolerance)
        except VerifyError as exc:
            print(f"[FAIL] 刷写已完成但标定写回未对齐: {exc}", file=sys.stderr)
            print(f"[FAIL] 备份在 {path}, 可单独执行 restore --file {path} 重试",
                  file=sys.stderr)
            return 1
    finally:
        cli.close()
    print(f"update OK, restore OK ({rounds} 轮)")
    return 0


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="IMU 标定矩阵备份/恢复(含不一致重写)")
    sub = parser.add_subparsers(dest="command", required=True)

    def common(p: argparse.ArgumentParser) -> None:
        p.add_argument("--port", required=True, help="设备串口, 如 COM3 或 /dev/ttyUSB0")
        p.add_argument("--baud", type=int, default=115200)
        p.add_argument("--attempts", type=int, default=DEFAULT_ATTEMPTS,
                       help="不一致时的最大重写轮数")
        p.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE,
                       help="读回比对容差(LOG 显示 3 位小数)")

    b = sub.add_parser("backup", help="读 LOG IMUCONFIG 并落盘 JSON")
    b.add_argument("--port", required=True)
    b.add_argument("--baud", type=int, default=115200)
    b.add_argument("--out", default=".", help="备份目录")
    b.set_defaults(func=cmd_backup)

    r = sub.add_parser("restore", help="从备份写回并读回验证(可重复执行)")
    r.add_argument("--file", required=True, help="backup 生成的 JSON")
    r.add_argument("--out", default=".", help=argparse.SUPPRESS)
    common(r)
    r.set_defaults(func=cmd_restore)

    u = sub.add_parser("update", help="备份 -> CAN 刷写 -> 写回验证")
    u.add_argument("--firmware", required=True, help="Intel HEX 镜像")
    u.add_argument("--raw-binary", action="store_true")
    u.add_argument("--force", action="store_true",
                   help="跳过型号安全门(不建议)")
    u.add_argument("--interface", default="can0")
    u.add_argument("--node", type=int, required=True)
    u.add_argument("--timeout", type=float, default=10.0)
    u.add_argument("--recovery-timeout", type=float, default=30.0)
    u.add_argument("--out", default=".", help="备份目录")
    common(u)
    u.set_defaults(func=cmd_update)
    return parser


# ---------------------------------------------------------------------------
# 自检(无硬件可跑)
# ---------------------------------------------------------------------------


def _gen_config(seed: int) -> dict[str, list[float]]:
    """程序生成一组测试矩阵(单位阵/零向量 + 确定性扰动), 不含真实设备数据。"""
    cfg: dict[str, list[float]] = {}
    for block_no, key in enumerate(BLOCK_KEYS):
        n = BLOCK_SIZE[key]
        values = []
        for i in range(n):
            base = 1.0 if n == 9 and i in (0, 4, 8) else 0.0
            # 每块换一组扰动, 避免块间数值雷同
            values.append(base + ((seed * (i + 3) + block_no * 7) % 21 - 10) * 1e-3)
        cfg[key] = values
    return cfg


def _render_config(cfg: dict[str, list[float]]) -> str:
    """把矩阵渲染成 LOG IMUCONFIG 文本(测试桩), 3 位小数与设备显示一致。"""
    out = []
    for name in BLOCK_KEYS:
        out.append(f"{name}=")
        values = cfg[name]
        if BLOCK_SIZE[name] == 9:
            for row in range(3):
                out.append(" ".join(f"{v:.3f}" for v in values[row * 3:row * 3 + 3]))
        else:
            out.extend(f"{v:.3f}" for v in values)
    return "\n".join(out) + "\nOK"


class _MockCli:
    """自检用: 预置读回序列, 记录发出的指令。"""

    def __init__(self, readbacks: list[str]):
        self.readbacks = list(readbacks)
        self.sent: list[str] = []

    def command(self, command: str) -> str:
        self.sent.append(command)
        if command == "LOG IMUCONFIG":
            return self.readbacks.pop(0)
        return "OK"


def run_self_tests() -> None:
    def check(cond: bool, message: str) -> None:
        if not cond:
            raise CalibError(f"self-test failed: {message}")

    cfg = _gen_config(3)
    text = _render_config(cfg)
    parsed = parse_imu_config(text)
    check(not compare(parsed, cfg, 1e-9), "解析往返(行序保持)")
    check(write_command("GYRB", cfg["GYRB"]).startswith("FCAL GB "),
          "GYRB 写指令参数名")
    check(build_write_commands(cfg)[-1] == "SAVECONFIG", "收尾 SAVECONFIG")
    check(fmt_num(1.0) == "1" and fmt_num(-0.0) == "0", "数值格式化")

    # 比对: 仅单元素不一致时精确点名
    mutated = {k: list(v) for k, v in cfg.items()}
    mutated["GYRB"][1] += 0.5
    diffs = compare(mutated, cfg, DEFAULT_TOLERANCE)
    check(list(diffs) == ["GYRB"] and [i for i, _, _ in diffs["GYRB"]] == [1],
          "单元素差异定位")

    # 恢复流程: 第 1 轮某块没写进去, 第 2 轮只重写该块后通过
    stale = {k: list(v) for k, v in cfg.items()}
    stale["GYRB"] = [v + 0.5 for v in cfg["GYRB"]]
    mock = _MockCli([_render_config(stale), _render_config(cfg)])
    rounds = restore(mock, cfg, attempts=3, tolerance=DEFAULT_TOLERANCE,
                     log=lambda _m: None)
    check(rounds == 2, f"两轮通过(实际 {rounds})")
    check(len([c for c in mock.sent if c.startswith("FCAL GMIS ")]) == 1,
          "GYRMIS 未受影响不重写")
    check(len([c for c in mock.sent if c.startswith("FCAL GB ")]) == 2,
          "GYRB 重写一次")
    check(mock.sent.count("SAVECONFIG") == 2, "每轮 SAVECONFIG")

    # 持续不一致 -> VerifyError
    mock2 = _MockCli([_render_config(stale)] * 3)
    try:
        restore(mock2, cfg, attempts=3, tolerance=DEFAULT_TOLERANCE,
                log=lambda _m: None)
    except VerifyError:
        pass
    else:
        check(False, "3 轮不一致必须抛 VerifyError")


def main(argv: Optional[list[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        run_self_tests()
        return args.func(args)
    except VerifyError:
        raise  # 由各子命令处理
    except (CalibError, OSError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
