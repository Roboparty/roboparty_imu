#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 changchuanyong

# Installed command examples:
#   /opt/roboparty/bin/imu_ota.py
#   /opt/roboparty/bin/imu_ota.py --firmware /path/to/firmware.bin
# Each invocation performs a real OTA after all preflight checks pass.
#
# The SocketCAN interface must be configured and up before real execution.
# MD7123 OTA uses Standard CAN-FD with 500000/2000000 bit/s arbitration/data
# rates, request ID 0x7C4, response ID 0x7CC, and 64-byte UCB frames.
# Only one MD7123 device may be connected because the fixed IDs contain no
# device address. This tool does not reconfigure the CAN link or stop services.
#
# Command 0x08 is interpreted according to the vendor Markdown protocol: its
# response identifies the currently running Bank, so the opposite Bank is
# programmed. An NA Bank response always aborts the update.
# Firmware header, size, markers, Bank CRC32 values, and the complete SHA-256
# are validated before the CAN interface is opened.

"""MD7123/MCT7123 CAN-FD OTA tool with conservative safety gates."""

from __future__ import annotations

import argparse
import hashlib
import re
import socket
import struct
import sys
import time
import zlib
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Optional


DEFAULT_IMU_TYPE = "MCT7123"
DEFAULT_INTERFACE = "can_imu"
DEFAULT_FIRMWARE = (
    "/opt/roboparty/lib/firmware/300702-0001_MD7123_App_v01.02.09_SW02.bin"
)

# Optional expected App version.
# Leave this as None to infer the version from a firmware filename containing
# vX.Y.Z. Set it to a value such as "01.02.09" when the filename has no version.
# The --expected-version command-line option overrides this value.
IMU_EXPECTED_VERSION: Optional[str] = None
#IMU_EXPECTED_VERSION: Optional[str] = "01.02.09"

ARBITRATION_BITRATE = 500000
DATA_BITRATE = 2000000
OTA_TX_ID = 0x7C4
OTA_RX_ID = 0x7CC
EXTENDED_ID = False
BRS_ENABLED = True
TIMEOUT_SEC = 3.0
MAX_RETRIES = 3
APP_START_TIMEOUT_SEC = 15.0
VERIFY_BANK_AFTER_REBOOT = True
ACCEPT_FIRST_WRITE_ZERO = False


class ConfigurationError(RuntimeError):
    """The configuration is invalid."""


class FirmwareError(RuntimeError):
    """Firmware preflight validation failed."""


class RetryableOtaError(RuntimeError):
    """A protocol error that may be retried according to the configuration."""


class FatalOtaError(RuntimeError):
    """An OTA error for which blind command-level retry is unsafe."""


class SelfTestError(RuntimeError):
    """An internal OTA protocol self-test failed."""


class OtaState(Enum):
    PRECHECK = "Preflight validation"
    ENTER_BOOTLOADER = "Enter Bootloader"
    NEGOTIATE = "Negotiate update parameters"
    TRANSFER = "Transfer firmware"
    FINALIZE = "Finalize and verify"
    REBOOT = "Reboot device"
    VERIFY = "Post-update verification"


@dataclass(frozen=True)
class OtaConfig:
    imu_type: str
    interface: str
    firmware: Path
    arbitration_bitrate: int
    data_bitrate: int
    tx_id: Optional[int]
    rx_id: Optional[int]
    extended_id: bool
    brs: bool
    timeout_sec: float
    max_retries: int
    expected_version: Optional[str]
    verify_bank_after_reboot: bool
    accept_first_write_zero: bool


@dataclass(frozen=True)
class FirmwareInfo:
    path: Path
    size: int
    sha256: str
    expected_version: Optional[str]
    bank_size: int
    bank_a: bytes
    bank_b: bytes
    crc32_a: int
    crc32_b: int


@dataclass(frozen=True)
class VersionInfo:
    product: str
    version: str
    commit: str
    raw: str


UCB_SYNC = b"\xBD\x64"
UCB_FRAME_SIZE = 64
UCB_PAYLOAD_SIZE = 59
WRITE_DATA_SIZE = 56
BANK_A_MARKER = b"a_bank_start:"
BANK_B_MARKER = b"b_bank_start:"


def log(message: str) -> None:
    print(f"[IMU OTA] {message}", flush=True)


def build_config(
    firmware_override: Optional[str],
    interface_override: Optional[str],
    expected_version_override: Optional[str] = None,
) -> OtaConfig:
    return OtaConfig(
        imu_type=DEFAULT_IMU_TYPE,
        interface=interface_override or DEFAULT_INTERFACE,
        firmware=Path(firmware_override or DEFAULT_FIRMWARE),
        arbitration_bitrate=ARBITRATION_BITRATE,
        data_bitrate=DATA_BITRATE,
        tx_id=OTA_TX_ID,
        rx_id=OTA_RX_ID,
        extended_id=EXTENDED_ID,
        brs=BRS_ENABLED,
        timeout_sec=TIMEOUT_SEC,
        max_retries=MAX_RETRIES,
        expected_version=expected_version_override or IMU_EXPECTED_VERSION,
        verify_bank_after_reboot=VERIFY_BANK_AFTER_REBOOT,
        accept_first_write_zero=ACCEPT_FIRST_WRITE_ZERO,
    )


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_ucb_frame(command: int, payload: bytes = b"") -> bytes:
    if command < 0 or command > 0xFF:
        raise ValueError("UCB command must fit in one byte")
    if len(payload) > UCB_PAYLOAD_SIZE:
        raise ValueError(f"UCB payload cannot exceed {UCB_PAYLOAD_SIZE} bytes")
    frame = bytearray(UCB_FRAME_SIZE)
    frame[0:2] = UCB_SYNC
    frame[2] = command
    frame[3 : 3 + len(payload)] = payload
    crc = crc16_ccitt(frame[:62])
    struct.pack_into("<H", frame, 62, crc)
    return bytes(frame)


def parse_ucb_frame(frame: bytes, expected_command: int) -> bytes:
    if len(frame) != UCB_FRAME_SIZE:
        raise RetryableOtaError(f"Unexpected UCB frame length: {len(frame)}")
    if frame[0:2] != UCB_SYNC:
        raise RetryableOtaError("Invalid UCB sync bytes")
    received_crc = struct.unpack_from("<H", frame, 62)[0]
    calculated_crc = crc16_ccitt(frame[:62])
    if received_crc != calculated_crc:
        raise RetryableOtaError(
            f"UCB CRC mismatch: received 0x{received_crc:04X}, calculated 0x{calculated_crc:04X}"
        )
    if frame[2] != expected_command:
        raise RetryableOtaError(
            f"Unexpected UCB response command 0x{frame[2]:02X}; expected 0x{expected_command:02X}"
        )
    return frame[3:62]


def select_target_bank(running_bank: str) -> str:
    if running_bank == "AA":
        return "BB"
    if running_bank == "BB":
        return "AA"
    raise FatalOtaError(f"Cannot select a target Bank from {running_bank!r}")


def encode_write_offset(offset: int) -> bytes:
    if offset < 0 or offset > 0xFFFFFF:
        raise FatalOtaError(f"Write offset is outside the 24-bit range: {offset}")
    return offset.to_bytes(3, byteorder="big")


def run_internal_self_tests() -> None:
    crc_examples = [
        (0x07, b"", 0x1290),
        (0x40, b"\x00", 0x1472),
        (0x40, b"\x01", 0xB12C),
        (0x30, b"", 0x0CB3),
        (0x06, b"", 0xA9DF),
        (0x20, b"", 0x0928),
        (0x09, b"", 0x338C),
        (0x21, b"\x00\x00\x00" + bytes(range(56)), 0xA856),
        (0x21, b"\x00\x00\x38" + bytes(range(56)), 0xA671),
    ]
    for command, payload, expected_crc in crc_examples:
        frame = build_ucb_frame(command, payload)
        actual_crc = int.from_bytes(frame[62:64], byteorder="little")
        if actual_crc != expected_crc:
            raise SelfTestError(
                f"CRC16 self-test failed for command 0x{command:02X}: "
                f"expected 0x{expected_crc:04X}, received 0x{actual_crc:04X}"
            )

    damaged_frame = bytearray(build_ucb_frame(0x07, b"MD7123;1.2.9"))
    damaged_frame[10] ^= 0x01
    try:
        parse_ucb_frame(bytes(damaged_frame), 0x07)
    except RetryableOtaError:
        pass
    else:
        raise SelfTestError("A UCB frame with a damaged CRC was accepted")

    encoded_offset = encode_write_offset(0x123456)
    if encoded_offset != b"\x12\x34\x56":
        raise SelfTestError(
            f"Write offset self-test failed: received {encoded_offset.hex()}"
        )

    if select_target_bank("AA") != "BB" or select_target_bank("BB") != "AA":
        raise SelfTestError("The current-to-target Bank mapping is invalid")

    log("Internal protocol self-test passed")


def parse_version_payload(payload: bytes) -> VersionInfo:
    try:
        raw = payload.split(b"\x00", 1)[0].decode("ascii")
    except UnicodeDecodeError as exc:
        raise FatalOtaError("Version response is not valid ASCII") from exc
    parts = raw.split(";")
    return VersionInfo(
        product=parts[0] if parts else "",
        version=parts[1] if len(parts) > 1 else "",
        commit=parts[2] if len(parts) > 2 else "",
        raw=raw,
    )


def normalize_version(version: str) -> Optional[tuple[int, int, int]]:
    match = re.search(r"(\d+)\.(\d+)\.(\d+)", version)
    if not match:
        return None
    return tuple(int(part) for part in match.groups())


def infer_expected_version(path: Path, override: Optional[str]) -> Optional[str]:
    if override:
        return override
    match = re.search(r"v(\d+(?:\.\d+){2})", path.name, re.IGNORECASE)
    return match.group(1) if match else None


def inspect_firmware(path: Path, expected_version_override: Optional[str] = None) -> FirmwareInfo:
    if not path.is_file():
        raise FirmwareError(f"Firmware file does not exist: {path}")
    data = path.read_bytes()
    if len(data) < 12 + len(BANK_A_MARKER) + len(BANK_B_MARKER):
        raise FirmwareError("Firmware file is too short for the dual-bank header")

    header_crc_a, header_crc_b, bank_size = struct.unpack_from("<III", data, 0)
    if bank_size <= 0:
        raise FirmwareError("Firmware bank size must be greater than zero")
    if bank_size > 0x1000000:
        raise FirmwareError("Firmware bank exceeds the 24-bit write-offset range")
    bank_a_marker_offset = 12
    bank_a_offset = bank_a_marker_offset + len(BANK_A_MARKER)
    bank_b_marker_offset = bank_a_offset + bank_size
    bank_b_offset = bank_b_marker_offset + len(BANK_B_MARKER)
    expected_size = bank_b_offset + bank_size

    if data[bank_a_marker_offset:bank_a_offset] != BANK_A_MARKER:
        raise FirmwareError("Bank A marker is missing or misplaced")
    if data[bank_b_marker_offset:bank_b_offset] != BANK_B_MARKER:
        raise FirmwareError("Bank B marker is missing or misplaced")
    if len(data) != expected_size:
        raise FirmwareError(
            f"Firmware size mismatch: received {len(data)}, expected {expected_size}"
        )
    if data.count(BANK_A_MARKER) != 1 or data.count(BANK_B_MARKER) != 1:
        raise FirmwareError("Each bank marker must appear exactly once")

    bank_a = data[bank_a_offset:bank_b_marker_offset]
    bank_b = data[bank_b_offset:expected_size]
    crc32_a = zlib.crc32(bank_a) & 0xFFFFFFFF
    crc32_b = zlib.crc32(bank_b) & 0xFFFFFFFF
    if crc32_a != header_crc_a or crc32_b != header_crc_b:
        raise FirmwareError(
            "Firmware bank CRC32 does not match the dual-bank header: "
            f"A 0x{header_crc_a:08X}/0x{crc32_a:08X}, "
            f"B 0x{header_crc_b:08X}/0x{crc32_b:08X}"
        )

    return FirmwareInfo(
        path=path,
        size=len(data),
        sha256=hashlib.sha256(data).hexdigest(),
        expected_version=infer_expected_version(path, expected_version_override),
        bank_size=bank_size,
        bank_a=bank_a,
        bank_b=bank_b,
        crc32_a=crc32_a,
        crc32_b=crc32_b,
    )


class CanFdTransport:
    """Linux SocketCAN-FD transport independent of the vendor protocol."""

    CAN_RAW_FD_FRAMES = 5
    CAN_RAW_FILTER = 1
    CAN_EFF_FLAG = 0x80000000
    CAN_RTR_FLAG = 0x40000000
    CAN_EFF_MASK = 0x1FFFFFFF
    CAN_SFF_MASK = 0x7FF
    CANFD_BRS = 0x01
    FRAME = struct.Struct("=IBBBB64s")

    def __init__(self, interface: str, timeout_sec: float, rx_id: int, extended_id: bool):
        self.interface = interface
        self.timeout_sec = timeout_sec
        self.rx_id = rx_id
        self.extended_id = extended_id
        self.sock: Optional[socket.socket] = None

    def open(self) -> None:
        if not hasattr(socket, "AF_CAN"):
            raise OSError("SocketCAN is not supported on this system")
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sol_can_raw = getattr(socket, "SOL_CAN_RAW", 101)
        sock.setsockopt(sol_can_raw, self.CAN_RAW_FD_FRAMES, 1)
        raw_id = self.rx_id | (self.CAN_EFF_FLAG if self.extended_id else 0)
        id_mask = self.CAN_EFF_MASK if self.extended_id else self.CAN_SFF_MASK
        can_mask = id_mask | self.CAN_EFF_FLAG | self.CAN_RTR_FLAG
        sock.setsockopt(
            sol_can_raw,
            self.CAN_RAW_FILTER,
            struct.pack("=II", raw_id, can_mask),
        )
        sock.settimeout(self.timeout_sec)
        sock.bind((self.interface,))
        self.sock = sock

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def send(self, can_id: int, payload: bytes, *, extended: bool, brs: bool) -> None:
        if self.sock is None:
            raise OSError("The CAN-FD interface is not open")
        if len(payload) > 64:
            raise ValueError("A CAN-FD frame cannot contain more than 64 data bytes")
        mask = self.CAN_EFF_MASK if extended else self.CAN_SFF_MASK
        if can_id < 0 or can_id > mask:
            raise ValueError("CAN ID is outside the allowed range")
        raw_id = can_id | (self.CAN_EFF_FLAG if extended else 0)
        flags = self.CANFD_BRS if brs else 0
        frame = self.FRAME.pack(raw_id, len(payload), flags, 0, 0, payload.ljust(64, b"\x00"))
        sent = self.sock.send(frame)
        if sent != self.FRAME.size:
            raise OSError(f"Incomplete CAN-FD frame write: {sent}/{self.FRAME.size}")

    def drain(self) -> None:
        if self.sock is None:
            raise OSError("The CAN-FD interface is not open")
        previous_timeout = self.sock.gettimeout()
        self.sock.setblocking(False)
        try:
            while True:
                self.sock.recv(self.FRAME.size)
        except BlockingIOError:
            pass
        finally:
            self.sock.settimeout(previous_timeout)

    def receive(self, timeout: Optional[float] = None) -> tuple[int, bytes, bool, int]:
        if self.sock is None:
            raise OSError("The CAN-FD interface is not open")
        self.sock.settimeout(self.timeout_sec if timeout is None else timeout)
        raw = self.sock.recv(self.FRAME.size)
        if len(raw) != self.FRAME.size:
            raise OSError(f"Unexpected CAN-FD frame size: {len(raw)}")
        raw_id, length, flags, _reserved0, _reserved1, data = self.FRAME.unpack(raw)
        if length > 64:
            raise OSError(f"Received invalid CAN-FD payload length: {length}")
        extended = bool(raw_id & self.CAN_EFF_FLAG)
        mask = self.CAN_EFF_MASK if extended else self.CAN_SFF_MASK
        return raw_id & mask, data[:length], extended, flags

    def __enter__(self) -> "CanFdTransport":
        self.open()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()


class Mct7123CanFdOtaProtocol:
    """MD7123/MCT7123 UCB Bootloader protocol over Standard CAN-FD."""

    IMPLEMENTED = True
    GET_BOOTLOADER_VERSION = 0x06
    GET_APP_VERSION = 0x07
    GET_CURRENT_BANK = 0x08
    GET_SERIAL_NUMBER = 0x09
    JUMP_APP = 0x20
    WRITE_APP = 0x21
    JUMP_BOOTLOADER = 0x30
    SET_SAMPLING = 0x40

    def __init__(
        self,
        config: OtaConfig,
        transport: CanFdTransport,
        sleeper: Callable[[float], None] = time.sleep,
    ):
        self.config = config
        self.transport = transport
        self.sleeper = sleeper

    @classmethod
    def ensure_available(cls, config: OtaConfig, firmware: FirmwareInfo) -> None:
        if config.imu_type.upper() not in {"MCT7123", "MD7123"}:
            raise ConfigurationError(f"The current protocol adapter does not support IMU_TYPE={config.imu_type}")
        if config.tx_id is None or config.rx_id is None:
            raise ConfigurationError("IMU_OTA_TX_ID and IMU_OTA_RX_ID are required for an update")
        if config.tx_id != 0x7C4 or config.rx_id != 0x7CC:
            raise ConfigurationError("MD7123 OTA requires fixed CAN IDs 0x7C4 and 0x7CC")
        if config.extended_id:
            raise ConfigurationError("MD7123 OTA uses Standard CAN-FD, not extended CAN IDs")
        if config.arbitration_bitrate != 500000 or config.data_bitrate != 2000000:
            raise ConfigurationError("MD7123 OTA requires 500000/2000000 bit/s CAN-FD rates")
        if firmware.expected_version is None:
            raise ConfigurationError(
                "The target version could not be inferred; set "
                "IMU_EXPECTED_VERSION or use --expected-version"
            )
        if normalize_version(firmware.expected_version) is None:
            raise ConfigurationError(
                f"Expected version is not in X.Y.Z form: {firmware.expected_version}"
            )

    def _receive_response(self, command: int, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        last_error: Optional[BaseException] = None
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RetryableOtaError(
                    f"No valid response for command 0x{command:02X}: {last_error or 'timeout'}"
                ) from last_error
            try:
                can_id, frame, extended, _flags = self.transport.receive(remaining)
            except socket.timeout as exc:
                raise RetryableOtaError(
                    f"Response timeout for command 0x{command:02X}"
                ) from exc
            if can_id != self.config.rx_id or extended != self.config.extended_id:
                continue
            try:
                return parse_ucb_frame(frame, command)
            except RetryableOtaError as exc:
                last_error = exc
                log(f"Ignoring invalid response for command 0x{command:02X}: {exc}")

    def _exchange_once(
        self,
        command: int,
        payload: bytes = b"",
        *,
        delay_before_receive: float = 0.0,
        timeout: Optional[float] = None,
    ) -> bytes:
        if self.config.tx_id is None:
            raise ConfigurationError("IMU_OTA_TX_ID is not configured")
        self.transport.drain()
        self.transport.send(
            self.config.tx_id,
            build_ucb_frame(command, payload),
            extended=self.config.extended_id,
            brs=self.config.brs,
        )
        if delay_before_receive > 0:
            self.sleeper(delay_before_receive)
        return self._receive_response(command, timeout or self.config.timeout_sec)

    def _query(self, command: int, payload: bytes = b"") -> bytes:
        last_error: Optional[BaseException] = None
        for attempt in range(1, self.config.max_retries + 1):
            try:
                return self._exchange_once(command, payload)
            except RetryableOtaError as exc:
                last_error = exc
                log(
                    f"Command 0x{command:02X} failed, attempt "
                    f"{attempt}/{self.config.max_retries}: {exc}"
                )
        raise RetryableOtaError(
            f"Command 0x{command:02X} exceeded the maximum retry count: {last_error}"
        ) from last_error

    def get_app_version(self) -> VersionInfo:
        return parse_version_payload(self._query(self.GET_APP_VERSION))

    def get_bootloader_version(self) -> VersionInfo:
        return parse_version_payload(self._query(self.GET_BOOTLOADER_VERSION))

    def wait_for_app_mode(self, timeout_sec: float) -> VersionInfo:
        deadline = time.monotonic() + timeout_sec
        last_result = "no response"
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RetryableOtaError(
                    f"App mode was not confirmed within {timeout_sec:.1f}s: "
                    f"{last_result}"
                )
            try:
                payload = self._exchange_once(
                    self.GET_BOOTLOADER_VERSION,
                    timeout=min(self.config.timeout_sec, remaining),
                )
                version = parse_version_payload(payload)
                last_result = version.raw or "empty version response"
                if (
                    version.product.startswith("MD7123")
                    and "bootloader" not in version.raw.lower()
                ):
                    return version
            except RetryableOtaError as exc:
                last_result = str(exc)
            remaining = deadline - time.monotonic()
            if remaining > 0:
                self.sleeper(min(0.5, remaining))

    def get_serial_number(self) -> str:
        payload = self._query(self.GET_SERIAL_NUMBER)
        mcu_sn = struct.unpack_from("<Q", payload, 0)[0]
        pcba = payload[8:18].rstrip(b"\x00").decode("ascii", errors="replace")
        product = payload[18:28].rstrip(b"\x00").decode("ascii", errors="replace")
        hardware = payload[28:33].rstrip(b"\x00").decode("ascii", errors="replace")
        return f"MCU=0x{mcu_sn:016X}, PCBA={pcba or '-'}, product={product or '-'}, HW={hardware or '-'}"

    def set_sampling(self, enabled: bool) -> None:
        expected = 1 if enabled else 0
        payload = self._query(self.SET_SAMPLING, bytes([expected]))
        if payload[0] != expected:
            raise FatalOtaError(
                f"Sampling state mismatch: requested {expected}, received {payload[0]}"
            )

    def jump_bootloader(self) -> None:
        payload = self._exchange_once(self.JUMP_BOOTLOADER)
        if payload[0] != 0:
            raise FatalOtaError(f"Bootloader jump rejected with condition 0x{payload[0]:02X}")

    def get_current_bank(self, firmware: FirmwareInfo) -> str:
        request = struct.pack(
            "<III", firmware.bank_size, firmware.crc32_a, firmware.crc32_b
        )
        payload = self._query(self.GET_CURRENT_BANK, request)
        try:
            bank = payload[0:2].decode("ascii")
        except UnicodeDecodeError as exc:
            raise FatalOtaError("Bank response is not valid ASCII") from exc
        if bank not in {"AA", "BB", "NA"}:
            raise FatalOtaError(f"Unexpected bank response: {bank!r}")
        return bank

    def write_block(self, offset: int, data: bytes, *, first: bool, last: bool) -> None:
        if not data or len(data) > WRITE_DATA_SIZE:
            raise FatalOtaError(f"Invalid write block length: {len(data)}")
        address = encode_write_offset(offset)
        request = address + data.ljust(WRITE_DATA_SIZE, b"\x00")
        delay = 8.0 if first or last else 0.01
        try:
            payload = self._exchange_once(
                self.WRITE_APP,
                request,
                delay_before_receive=delay,
                timeout=max(self.config.timeout_sec, 2.0),
            )
        except RetryableOtaError as exc:
            raise FatalOtaError(
                f"Write response failed at offset 0x{offset:06X}; restart the full OTA"
            ) from exc
        if payload[0] == 1:
            return
        if first and payload[0] == 0 and self.config.accept_first_write_zero:
            log("Warning: accepting write_result=0 for the first block by explicit configuration")
            return
        raise FatalOtaError(
            f"Flash write failed at offset 0x{offset:06X} with result {payload[0]}"
        )

    def jump_app(self) -> str:
        payload = self._exchange_once(self.JUMP_APP)
        return payload[0:2].rstrip(b"\x00").decode("ascii", errors="replace")


class OtaRunner:
    def __init__(self, config: OtaConfig, firmware: FirmwareInfo):
        self.config = config
        self.firmware = firmware

    def execute(self) -> None:
        Mct7123CanFdOtaProtocol.ensure_available(self.config, self.firmware)
        if self.config.rx_id is None:
            raise ConfigurationError("IMU_OTA_RX_ID is not configured")
        with CanFdTransport(
            self.config.interface,
            self.config.timeout_sec,
            self.config.rx_id,
            self.config.extended_id,
        ) as transport:
            protocol = Mct7123CanFdOtaProtocol(self.config, transport)
            mode = "app"
            sampling_stopped = False
            try:
                try:
                    log(f"Device identity: {protocol.get_serial_number()}")
                except RetryableOtaError as exc:
                    log(f"Warning: serial-number query failed: {exc}")

                old_version = protocol.get_app_version()
                if not old_version.product.startswith("MD7123"):
                    raise FatalOtaError(
                        f"Unexpected product in version response: {old_version.product}"
                    )
                log(f"Current App version: {old_version.raw}")

                protocol.set_sampling(False)
                sampling_stopped = True
                time.sleep(0.1)

                log(OtaState.ENTER_BOOTLOADER.value)
                mode = "unknown"
                protocol.jump_bootloader()
                time.sleep(2.0)
                mode = "bootloader"

                bootloader_version = protocol.get_bootloader_version()
                if not bootloader_version.product.startswith("MD7123"):
                    raise FatalOtaError(
                        "Unexpected product in Bootloader response: "
                        f"{bootloader_version.product or '<empty>'}"
                    )
                if "bootloader" not in bootloader_version.raw.lower():
                    raise FatalOtaError(
                        "Command 0x06 did not confirm Bootloader mode: "
                        f"{bootloader_version.raw!r}"
                    )
                log(f"Bootloader version: {bootloader_version.raw}")
                sampling_stopped = False

                log(OtaState.NEGOTIATE.value)
                running_bank = protocol.get_current_bank(self.firmware)
                if running_bank == "NA":
                    raise FatalOtaError(
                        "Bank response is NA; refusing to infer a target bank"
                    )
                target_bank = select_target_bank(running_bank)
                bank_data = self.firmware.bank_b if target_bank == "BB" else self.firmware.bank_a
                log(f"Running bank: {running_bank}; target bank: {target_bank}")
                time.sleep(1.0)

                log(OtaState.TRANSFER.value)
                total = len(bank_data)
                block_count = (total + WRITE_DATA_SIZE - 1) // WRITE_DATA_SIZE
                last_percent = -1
                for index, offset in enumerate(range(0, total, WRITE_DATA_SIZE)):
                    block = bank_data[offset : offset + WRITE_DATA_SIZE]
                    protocol.write_block(
                        offset,
                        block,
                        first=index == 0,
                        last=index == block_count - 1,
                    )
                    completed = offset + len(block)
                    percent = completed * 100 // total
                    if percent != last_percent:
                        log(
                            f"Transfer progress: {percent}% "
                            f"({index + 1}/{block_count} blocks)"
                        )
                        last_percent = percent

                log(OtaState.FINALIZE.value)
                time.sleep(2.0)
                log(OtaState.REBOOT.value)
                mode = "unknown"
                jump_status = protocol.jump_app()
                log(f"Jump App response: {jump_status or '<empty>'}")
                app_mode_version = protocol.wait_for_app_mode(APP_START_TIMEOUT_SEC)
                mode = "app"
                sampling_stopped = True
                log(f"App mode confirmed: {app_mode_version.raw}")

                log(OtaState.VERIFY.value)
                new_version = protocol.get_app_version()
                log(f"Updated App version: {new_version.raw}")
                if not new_version.product.startswith("MD7123"):
                    raise FatalOtaError(
                        f"Unexpected product after update: {new_version.product or '<empty>'}"
                    )
                expected = normalize_version(self.firmware.expected_version or "")
                received = normalize_version(new_version.version)
                if expected is None or received != expected:
                    raise FatalOtaError(
                        f"Version mismatch: expected {self.firmware.expected_version}, "
                        f"received {new_version.version or new_version.raw}"
                    )

                verified_bank: Optional[str] = None
                if self.config.verify_bank_after_reboot:
                    verified_bank = protocol.get_current_bank(self.firmware)
                    if verified_bank != target_bank:
                        raise FatalOtaError(
                            f"Bank verification failed: expected {target_bank}, got {verified_bank}"
                        )
                    log(f"Updated Bank: {verified_bank}")

                protocol.set_sampling(True)
                sampling_stopped = False
                bank_result = verified_bank or target_bank
                log(
                    "OTA update completed successfully: "
                    f"Bank {running_bank} -> {bank_result}, version {new_version.raw}"
                )
            finally:
                if mode == "app" and sampling_stopped:
                    try:
                        protocol.set_sampling(True)
                        log("Sampling restored during cleanup")
                    except Exception as exc:
                        log(f"Warning: failed to restore sampling during cleanup: {exc}")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Safe IMU CAN-FD OTA framework")
    parser.add_argument("--firmware", help="Override the built-in firmware path")
    parser.add_argument("--interface", help="Override the built-in SocketCAN interface")
    parser.add_argument(
        "--expected-version",
        help="Override IMU_EXPECTED_VERSION and firmware filename inference",
    )
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        run_internal_self_tests()
        config = build_config(
            args.firmware,
            args.interface,
            args.expected_version,
        )
        log(OtaState.PRECHECK.value)
        firmware = inspect_firmware(config.firmware, config.expected_version)
        log(f"Firmware: {firmware.path} ({firmware.size} bytes)")
        log(f"SHA-256: {firmware.sha256}")
        log(
            f"Bank size: {firmware.bank_size}; CRC32 A/B: "
            f"0x{firmware.crc32_a:08X}/0x{firmware.crc32_b:08X}"
        )
        log(
            f"Device: {config.imu_type}, interface: {config.interface}, "
            f"arbitration/data bitrate: {config.arbitration_bitrate}/{config.data_bitrate}"
        )

        OtaRunner(config, firmware).execute()
        return 0
    except (ConfigurationError, FirmwareError) as exc:
        log(f"Configuration or firmware error: {exc}")
        return 2
    except RetryableOtaError as exc:
        log(f"Update retries exhausted: {exc}")
        return 5
    except FatalOtaError as exc:
        log(f"OTA aborted: {exc}")
        return 6
    except SelfTestError as exc:
        log(f"Internal self-test failed: {exc}")
        return 3
    except OSError as exc:
        log(f"System interface error: {exc}")
        return 4
    except KeyboardInterrupt:
        log("Update interrupted by the user")
        return 130


if __name__ == "__main__":
    sys.exit(main())
