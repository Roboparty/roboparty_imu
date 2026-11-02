#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 RoboParty

# Installed command examples:
#   /opt/roboparty/bin/hipnuc_ota.py info --interface can0 --node 8
#   /opt/roboparty/bin/hipnuc_ota.py update /path/to/HIPNUC_x.y.z_HIxxxx.hex \
#       --interface can0 --node 8
# Each update invocation performs a real flash write after all preflight checks.
#
# The SocketCAN interface must be configured and up before execution, e.g.
#   sudo ip link set can0 up type can bitrate 500000
# HiPNUC CAN version queries use the vendor HIREG register protocol (29-bit
# extended frames 0x0CEF....) and only work while the application is running.
# Firmware updates use the CANopen-SDO-style bootloader (standard frames
# 0x600+node / 0x580+node, vendor object 0x1F51) and are destructive: use an
# image made for the exact product. This tool verifies the product name from
# the identity registers against the firmware filename before erasing, and
# re-reads the firmware version after the update to confirm the new
# application started. It does not reconfigure the CAN link or stop services.
# Aligned with hipnuc SDK commit cfbc5c0 (2026-11-02).

"""HiPNUC IMU CAN version query and firmware update tool.

Protocol logic ported from the HiPNUC Python SDK (hipnuc.can / hipnuc.update /
hipnuc._firmware_image / hipnuc.models) with conservative safety gates:
model check before any destructive request, and a post-update identity
re-read (the bootloader acknowledgements alone do not prove the new
application runs).
"""

from __future__ import annotations

import argparse
from binascii import crc32
from contextlib import contextmanager
import errno
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys
import tempfile
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Iterator, Optional

if TYPE_CHECKING:
    import can

DEFAULT_INTERFACE = "can0"
DEFAULT_INFO_TIMEOUT = 10.0
DEFAULT_RECOVERY_TIMEOUT = 30.0

# Host address used by product firmware for configuration replies.
HOST_ADDRESS = 0x55
CONFIG_ID = 0x0CEF0000
# Identity registers: product 0x70-0x77, fw/bl versions 0x78/0x79,
# reserved 0x7A-0x7E (0x7C-0x7E are unmapped), serial 0x7F-0x82.
IDENTITY_SLOTS = (*range(0x70, 0x7C), *range(0x7F, 0x83))
IDENTITY_WORDS = 19
MAX_IMAGE_SIZE = 64 * 1024 * 1024

# HiPNUC application-image layout (aligned with the SDK's _firmware_image).
_MARKER = 0x0FFEB6B6
_MARKER_SCAN = 100 * 1024  # The bootloader looks for the marker this far.
_HEADER_MAGIC = 0xFEEDA5A5
_META_MAGIC = 0x41504D44
_DEBUG_CRC = 0x55AA1234

# SDO bootloader (vendor object 0x1F51) request codes.
SDO_DOWNLOAD_INITIATE = 0x21
SDO_EXPEDITED_WRITE = 0x23
SUB_ENTER_BOOTLOADER = 5
SUB_HANDSHAKE = 6
SUB_DOWNLOAD = 1
SUB_START_APP = 9


# ---------------------------------------------------------------------------
# Exceptions (ported from the SDK error tree; CommandTimeout is not needed).
# ---------------------------------------------------------------------------


class HipnucError(Exception):
    """Base class for device communication and configuration errors."""


class TransportError(HipnucError):
    """The bus could not be opened, read, or written."""


class ResponseTimeout(TransportError):
    """The requested response was not received before the deadline."""


class DeviceError(HipnucError):
    """The device rejected a request; retain its original response."""

    def __init__(self, message: str, *, code: Optional[int] = None, response: str = ""):
        super().__init__(message)
        self.code = code
        self.response = response


class VerificationError(DeviceError):
    """The response differs from the requested value or is malformed."""


class ConfigurationError(RuntimeError):
    """CLI, image or model-gate error (never sent to the device)."""


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class DeviceInfo:
    """Identity reported by a device; unavailable fields remain None."""

    product_name: Optional[str] = None
    firmware_version: Optional[str] = None
    bootloader_version: Optional[str] = None
    serial_number: Optional[str] = None
    build: Optional[str] = None  # only from image metadata; registers carry none
    raw_response: str = ""

    def to_dict(self) -> dict:
        return {
            "product_name": self.product_name,
            "firmware_version": self.firmware_version,
            "bootloader_version": self.bootloader_version,
            "serial_number": self.serial_number,
            "build": self.build,
            "raw_response": self.raw_response,
        }


@dataclass(frozen=True)
class FirmwareImage:
    """``version``/``build``/``debug`` are None/False when the image does not record them."""

    address: int  # unused on the CAN path (sequential write at a fixed address)
    data: bytes
    version: Optional[str] = None
    build: Optional[str] = None
    debug: bool = False


@dataclass(frozen=True)
class UpdateResult:
    """Versions are None when unknown: an image without version data, a
    device that did not answer before the update (for example one left in the
    bootloader), or verification turned off."""

    bytes_written: int
    image_version: Optional[str]
    previous_version: Optional[str]
    verified_version: Optional[str]

    def to_dict(self) -> dict:
        return {
            "bytes_written": self.bytes_written,
            "image_version": self.image_version,
            "previous_version": self.previous_version,
            "verified_version": self.verified_version,
        }


def _version_key(version: Optional[str]) -> Optional[tuple]:
    try:
        return tuple(int(part) for part in version.split(".")) if version else None
    except ValueError:
        return None


def _check_wait(wait: float) -> None:
    if isinstance(wait, bool) or not isinstance(wait, (int, float)) or not 0 <= wait < math.inf:
        raise ValueError("power_cycle_wait must be a finite number of seconds, 0 or more")


def _stopped(error: HipnucError) -> HipnucError:
    """The same failure, saying how to finish an update the erase already started."""
    message = f"{str(error).rstrip('.')}. The update stopped part-way; run the update again."
    if isinstance(error, DeviceError):
        return type(error)(message, code=error.code, response=error.response)
    return type(error)(message)


def _check_started(image: FirmwareImage, info: DeviceInfo) -> Optional[str]:
    running = info.firmware_version
    if image.version and _version_key(running) != _version_key(image.version):
        raise VerificationError(
            f"The device runs version {running} after the update, not the image's {image.version}"
        )
    return running


@dataclass(frozen=True)
class UpdateReport:
    before: Optional[DeviceInfo]
    after: Optional[DeviceInfo]
    result: UpdateResult
    model_expected: Optional[str]
    model_matched: Optional[bool]
    expected_version: Optional[str]
    version_changed: Optional[bool]
    image_build: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "before": self.before.to_dict() if self.before else None,
            "after": self.after.to_dict() if self.after else None,
            "result": self.result.to_dict(),
            "model_expected": self.model_expected,
            "model_matched": self.model_matched,
            "expected_version": self.expected_version,
            "version_changed": self.version_changed,
            "image_build": self.image_build,
        }


# ---------------------------------------------------------------------------
# Logging and output
# ---------------------------------------------------------------------------


def log(message: str) -> None:
    print(f"[HIPNUC OTA] {message}", file=sys.stderr, flush=True)


def emit_json(payload: dict) -> None:
    print(json.dumps(payload, ensure_ascii=False, allow_nan=False))


def emit_info(info: DeviceInfo) -> None:
    for key, value in info.to_dict().items():
        if key == "raw_response" or value is None:
            continue
        print(f"{key}: {value}")


def emit_report(report: UpdateReport) -> None:
    before = report.before.firmware_version if report.before else None
    after = report.after.firmware_version if report.after else None
    print(f"before_version: {before}")
    print(f"after_version: {after}")
    print(f"bytes_written: {report.result.bytes_written}")
    print(f"image_version: {report.result.image_version}")
    print(f"image_build: {report.image_build}")
    print(f"previous_version: {report.result.previous_version}")
    print(f"verified_version: {report.result.verified_version}")
    print(f"model_expected: {report.model_expected}")
    print(f"model_matched: {report.model_matched}")
    print(f"expected_version: {report.expected_version}")
    print(f"version_changed: {report.version_changed}")


# ---------------------------------------------------------------------------
# Firmware image (ported from _firmware_image.py)
# ---------------------------------------------------------------------------


def load_image(path: str | Path, *, raw_binary: bool = False) -> FirmwareImage:
    """Load strict Intel HEX, or explicitly requested binary; fill HEX gaps with FF.

    HEX images additionally carry a HiPNUC check: application marker, image
    header and CRC32 (any failure raises before the device is touched), and the
    embedded version/build are extracted. The raw_binary escape hatch skips
    all of that.
    """
    path = Path(path)
    if raw_binary:
        with path.open("rb") as source:
            data = source.read(MAX_IMAGE_SIZE + 1)
        if not data or len(data) > MAX_IMAGE_SIZE:
            raise ValueError("Firmware must contain 1 byte to 64 MiB of data")
        return FirmwareImage(0, data)

    origin = None
    upper = 0
    data = bytearray()
    ended = False
    try:
        text = path.read_bytes().decode("ascii")
    except UnicodeDecodeError:
        raise ValueError(f"{path.name} is not an Intel HEX file") from None
    for line_number, line in enumerate(text.splitlines(), 1):
        line = line.rstrip("\r\n")
        if not line:
            continue
        error = f"Invalid Intel HEX at line {line_number}"
        if ended:
            raise ValueError(f"{error}: data after end-of-file record")
        if not line.startswith(":") or len(line) > 521:
            raise ValueError(error)
        # bytes.fromhex alone also accepts embedded whitespace.
        digits = line[1:]
        if len(digits) % 2 or any(c not in "0123456789abcdefABCDEF" for c in digits):
            raise ValueError(error)
        record = bytes.fromhex(digits)
        if len(record) < 5 or len(record) != record[0] + 5:
            raise ValueError(f"{error}: record length mismatch")
        if sum(record) & 0xFF:
            raise ValueError(f"{error}: checksum mismatch")
        count, kind = record[0], record[3]
        address = int.from_bytes(record[1:3], "big")
        payload = record[4:-1]
        if kind == 0:
            if not count:
                continue
            absolute = upper + address
            if absolute + count > 0x100000000:
                raise ValueError(f"{error}: address overflow")
            if origin is None:
                origin = absolute
            offset = absolute - origin
            if offset < len(data):
                raise ValueError(f"{error}: records must be ascending and not overlap")
            if offset + count > MAX_IMAGE_SIZE:
                raise ValueError("Firmware address span exceeds 64 MiB")
            data.extend(b"\xff" * (offset - len(data)))
            data.extend(payload)
        elif kind == 1:
            if count or address:
                raise ValueError(f"{error}: malformed end-of-file record")
            ended = True
        elif kind in (2, 4):
            if count != 2 or address:
                raise ValueError(f"{error}: malformed extended address")
            upper = int.from_bytes(payload, "big") << (4 if kind == 2 else 16)
        elif kind in (3, 5):
            if count != 4 or address:
                raise ValueError(f"{error}: malformed start address")
            # The bootloader starts the application using its vector table.
        else:
            raise ValueError(f"{error}: unsupported record type {kind:02X}")
    if not ended:
        raise ValueError("Intel HEX is missing its end-of-file record")
    if origin is None or not data:
        raise ValueError("Firmware contains no data")
    # HiPNUC application-image validation, verbatim the SDK's checks.
    data = bytes(data)
    for marker in range(0, min(len(data), _MARKER_SCAN) - 7, 4):
        if struct.unpack_from("<I", data, marker)[0] == _MARKER:
            break
    else:
        raise ValueError("Not a HiPNUC application image: image marker not found")
    header = struct.unpack_from("<I", data, marker + 4)[0] - origin
    if not 0 <= header <= len(data) - 24 or struct.unpack_from("<I", data, header)[0] != (
        _HEADER_MAGIC
    ):
        raise ValueError("Not a HiPNUC application image: invalid image header")
    legacy_version, length, checksum = struct.unpack_from("<III", data, header + 8)
    debug = checksum == _DEBUG_CRC
    if not debug:
        if not header + 20 <= length <= len(data):
            raise ValueError("Image header length does not match the file")
        # The checksum covers the image except its own four bytes.
        if crc32(data[header + 20 : length], crc32(data[: header + 16])) != checksum:
            raise ValueError("Image CRC mismatch: the file is damaged or incomplete")
    version = build = None
    meta = header + 24
    if len(data) >= meta + 24 and struct.unpack_from("<I", data, meta)[0] == _META_MAGIC:
        raw_version, date = struct.unpack_from("<II", data, meta + 8)
        version = _version(raw_version)
        if date:
            build = f"20{date // 10000:02d}-{date // 100 % 100:02d}-{date % 100:02d}"
    elif 0 < legacy_version <= 999:
        version = _version(legacy_version)
    return FirmwareImage(origin, data, version, build, debug)


def _version(raw: int) -> Optional[str]:
    return f"{raw // 100}.{raw // 10 % 10}.{raw % 10}" if raw else None


# ---------------------------------------------------------------------------
# Model / version extraction from the firmware filename (safety gate)
# ---------------------------------------------------------------------------


def extract_model_from_filename(path: str | Path) -> Optional[str]:
    match = re.search(r"HI\d+", Path(path).name, re.IGNORECASE)
    return match.group(0).upper() if match else None


def extract_version_from_filename(path: str | Path) -> Optional[str]:
    match = re.search(r"(\d+)\.(\d+)\.(\d+)", Path(path).name)
    return ".".join(match.groups()) if match else None


# ---------------------------------------------------------------------------
# Identity decoding (ported from models.identity_from_registers)
# ---------------------------------------------------------------------------


def identity_from_registers(words: list) -> DeviceInfo:
    """Decode the 19-word identity block at register 0x70 (Modbus and J1939).

    Words are big-endian 16-bit values: product name 0x70-0x77, firmware and
    bootloader versions 0x78-0x79, reserved 0x7A-0x7E and serial number
    0x7F-0x82. Unavailable words may be zero.
    """
    if len(words) != IDENTITY_WORDS or not all(
        isinstance(word, int) and not isinstance(word, bool) and 0 <= word <= 0xFFFF
        for word in words
    ):
        raise ValueError("identity block must contain 19 unsigned 16-bit words")
    raw = b"".join(word.to_bytes(2, "big") for word in words)

    def version(value: int) -> Optional[str]:
        return f"{value // 100}.{value // 10 % 10}.{value % 10}" if value else None

    product_name = raw[:16].split(b"\0", 1)[0].decode("ascii", errors="replace").strip()
    serial = raw[30:38]
    return DeviceInfo(
        product_name=product_name or None,
        firmware_version=version(int.from_bytes(raw[16:18], "big")),
        bootloader_version=version(int.from_bytes(raw[18:20], "big")),
        serial_number=serial.hex().upper() if any(serial) else None,
        raw_response=raw.hex(),
    )


# ---------------------------------------------------------------------------
# HIREG register protocol (ported from hipnuc.can)
# ---------------------------------------------------------------------------

_ERROR_FRAME_HINTS = (
    (
        0x40,
        "the CAN controller is bus-off: check the bitrate and wiring, then restart the interface",
    ),
    (0x20, "no node acknowledges the frames: check the bitrate, wiring and termination"),
)
_ERRNO_HINTS = {
    errno.ENETDOWN: "the CAN interface is down",
    errno.ENOBUFS: "the transmit queue is full because no node acknowledges the frames: "
    "check the bitrate, wiring and termination",
    errno.ENODEV: "the CAN interface does not exist",
}


def _bus_hint(error_class: int = 0, code: Optional[int] = None) -> str:
    """Explain error frames seen (SocketCAN error classes) or a send errno."""
    for bit, reason in _ERROR_FRAME_HINTS:
        if error_class & bit:
            return reason
    if error_class:
        return "the CAN bus reports errors: check the bitrate, wiring and termination"
    return _ERRNO_HINTS.get(code, "")


def _validate_node(node_id: int) -> int:
    if (
        not isinstance(node_id, int)
        or isinstance(node_id, bool)
        or not 1 <= node_id <= 127
        or node_id == HOST_ADDRESS
    ):
        raise ValueError("node_id must be 1..127 and must not be 85 (0x55 host address)")
    return node_id


def _validate_timeout(value: float) -> float:
    if isinstance(value, bool) or value != value or value in (float("inf"), float("-inf")) or value <= 0:
        raise ValueError("timeout must be finite and greater than zero")
    return value


def _config_message(node_id: int, address: int, command: int, value: int) -> "can.Message":
    import can

    return can.Message(
        arbitration_id=CONFIG_ID | (node_id << 8) | HOST_ADDRESS,
        is_extended_id=True,
        data=struct.pack("<HBBI", address, command, 0, value),
        check=True,
    )


def _register_transaction(
    bus: "can.BusABC", node_id: int, address: int, command: int, value: int, timeout: float
) -> int:
    """One register read/write; the protocol has no transaction counter."""
    import can

    node_id = _validate_node(node_id)
    _validate_timeout(timeout)
    if not 0 <= address <= 0xFFFF or not 0 <= value <= 0xFFFFFFFF:
        raise ValueError("register address or value out of range")
    request = _config_message(node_id, address, command, value)
    deadline = time.monotonic() + timeout
    sent_time = time.time()
    expected_id = CONFIG_ID | (HOST_ADDRESS << 8) | node_id
    error_class = 0
    try:
        bus.send(request, timeout=timeout)
        while (remaining := deadline - time.monotonic()) > 0:
            reply = bus.recv(timeout=remaining)
            if reply is None or time.monotonic() >= deadline:
                break
            if reply.is_error_frame:
                error_class |= reply.arbitration_id
                continue
            # A reply stamped on the host clock before this request was queued
            # earlier. Adapter-relative timestamps (seconds since power-up) are
            # far below the epoch and are not judged. (The SDK keeps this filter
            # here only; _can_exchange has none.)
            stale = sent_time - 86400 < reply.timestamp < sent_time
            if (
                reply.arbitration_id != expected_id
                or not reply.is_extended_id
                or not reply.is_rx
                or reply.is_remote_frame
                or reply.is_error_frame
                or reply.is_fd
                or reply.dlc != 8
                or len(reply.data) != 8
                or stale
            ):
                continue
            reply_address, reply_command, status, result = struct.unpack("<HBBI", reply.data)
            if reply_address != address or reply_command != command:
                continue
            if status:
                raise DeviceError(
                    f"CAN node {node_id} rejected register 0x{address:04X} (status {status})",
                    code=status,
                    response=bytes(reply.data).hex(),
                )
            return result
    except can.CanError as exc:
        hint = _bus_hint(error_class, getattr(exc, "error_code", None))
        raise TransportError(
            f"CAN register transaction failed: {exc}" + (f"; {hint}" if hint else "")
        ) from exc
    hint = _bus_hint(error_class)
    raise ResponseTimeout(
        f"No matching CAN response from node {node_id} for register 0x{address:04X} "
        f"within {timeout:.3g}s; "
        + (
            hint
            # Many adapters report no error frames for a bitrate mismatch.
            or "check the bitrate and node ID, which the device may have changed; "
            "or the device does not support the address"
        )
    )


def read_register(bus: "can.BusABC", node_id: int, address: int, timeout: float = 2.0) -> int:
    return _register_transaction(bus, node_id, address, 3, 0, timeout)


def read_info(bus: "can.BusABC", node_id: int, timeout: float = DEFAULT_INFO_TIMEOUT) -> DeviceInfo:
    """Read product, firmware and serial number through the identity registers.

    Sixteen register reads share one identity layout; ``timeout`` is the total
    elapsed budget for all reads. Only valid while the application runs.
    """
    _validate_node(node_id)
    deadline = time.monotonic() + _validate_timeout(timeout)
    expired = f"CAN node {node_id} identity was not read within {timeout:g}s"
    words = [0] * IDENTITY_WORDS
    for address in IDENTITY_SLOTS:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ResponseTimeout(expired)
        words[address - 0x70] = (
            _register_transaction(bus, node_id, address, 3, 0, remaining) & 0xFFFF
        )
    if time.monotonic() >= deadline:
        raise ResponseTimeout(expired)
    return identity_from_registers(words)


def wait_for_node(
    bus: "can.BusABC",
    node_id: int,
    recovery_timeout: float = DEFAULT_RECOVERY_TIMEOUT,
    timeout: float = DEFAULT_INFO_TIMEOUT,
) -> DeviceInfo:
    """Poll the identity registers until ``node_id`` answers or the wait expires.

    Used after an update to confirm the new application reached run mode.
    """
    _validate_node(node_id)
    _validate_timeout(timeout)
    deadline = time.monotonic() + _validate_timeout(recovery_timeout)
    last_error: Optional[Exception] = None
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ResponseTimeout(
                f"CAN node {node_id} did not answer within {recovery_timeout:g}s"
            ) from last_error
        try:
            info = read_info(bus, node_id, timeout=min(timeout, remaining))
            if time.monotonic() < deadline:
                return info
        except (ResponseTimeout, DeviceError) as exc:
            # Differs from SDK cfbc5c0 on purpose: a booting device may briefly
            # reject registers; keep polling instead of aborting.
            last_error = exc
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(min(0.05, remaining))


# ---------------------------------------------------------------------------
# SDO bootloader layer (ported from update.py, CAN path only)
# ---------------------------------------------------------------------------


def _drain_can(bus: "can.BusABC") -> None:
    """Do not accept an acknowledgement queued before this update started."""
    import can

    deadline = time.monotonic() + 0.1
    try:
        while bus.recv(0) is not None:
            if time.monotonic() >= deadline:
                raise ResponseTimeout("CAN receive queue did not drain; use a newly opened bus")
    except (can.CanError, OSError) as error:
        raise TransportError(f"CAN firmware update failed: {error}") from error


def _assert_bus_clear(
    bus: "can.BusABC", node_id: int, *, window: float = 2.0, force: bool = False
) -> None:
    """Refuse to flash while another master addresses the node.

    Passive listen only: SDO requests (0x600+node) or configuration requests
    (0x0CEF[DA][SA], DA = node) from anyone else mean a second master is active;
    two masters fighting over the bootloader can corrupt an update. Measuring
    frames and replies pass through silently.
    """
    if window <= 0:
        return
    seen = []
    deadline = time.monotonic() + window
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        reply = bus.recv(remaining)
        if reply is None or reply.is_error_frame:
            continue
        identifier = reply.arbitration_id
        if identifier == 0x600 + node_id or (
            identifier >> 16 == 0x0CEF and (identifier >> 8) & 0xFF == node_id
        ):
            seen.append(identifier)
    if not seen:
        return
    message = (
        f"another master is addressing CAN node {node_id}: {len(seen)} request(s) "
        f"seen (e.g. 0x{seen[0]:08X}). Stop other tools (CHCenter, services, "
        "scripts) before flashing"
    )
    if not force:
        raise TransportError(message)
    log(f"Warning: {message}")


def _can_send(bus: "can.BusABC", message: "can.Message", timeout: float) -> None:
    import can

    try:
        bus.send(message, timeout=timeout)
    except can.CanError as error:
        if getattr(error, "error_code", None) != errno.ENOBUFS:
            raise
        # The kernel refused the frame, so nothing reached the bus; try once more.
        time.sleep(0.05)
        bus.send(message, timeout=timeout)


def _can_exchange(bus: "can.BusABC", node_id: int, request: bytes, timeout: float) -> bytes:
    import can

    error_class = 0
    try:
        deadline = time.monotonic() + timeout
        _can_send(
            bus,
            can.Message(arbitration_id=0x600 + node_id, is_extended_id=False, data=request),
            timeout,
        )
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                hint = _bus_hint(error_class)
                raise ResponseTimeout(
                    f"CAN node {node_id} did not reply" + (f": {hint}" if hint else "")
                )
            reply = bus.recv(min(remaining, 0.05))
            if reply is None or time.monotonic() >= deadline:
                continue
            if reply.is_error_frame:
                error_class |= reply.arbitration_id
                continue
            if reply.arbitration_id != 0x580 + node_id or reply.is_extended_id:
                continue
            if reply.is_remote_frame or reply.is_fd or reply.dlc != 8 or len(reply.data) != 8:
                raise VerificationError("Invalid CAN bootloader reply frame")
            payload = bytes(reply.data)
            # Delayed/repeated handshake ACKs must not derail an erase already
            # requested. The deadline still bounds a stream of stale replies.
            if payload[:3] == b"\x60\x51\x1f" and payload[3] in (5, 6):
                if request[:4] != b"\x23" + payload[1:4]:
                    continue
            if payload[0] == 0x80:
                expected = request[1:4] if request[0] in (0x21, 0x23) else b"\x51\x1f\x01"
                if payload[1:4] != expected:
                    raise VerificationError("Unmatched CAN bootloader abort")
                code = int.from_bytes(payload[4:], "little")
                raise DeviceError(f"CAN bootloader aborted: 0x{code:08X}", code=code)
            return payload
    except (can.CanError, OSError) as error:
        code = getattr(error, "error_code", None) or getattr(error, "errno", None)
        hint = _bus_hint(error_class, code)
        raise TransportError(
            f"CAN firmware update failed: {error}" + (f"; {hint}" if hint else "")
        ) from error


def _sdo_write(bus: "can.BusABC", node_id: int, subindex: int, timeout: float = 0.1) -> None:
    request = bytes((SDO_EXPEDITED_WRITE, 0x51, 0x1F, subindex, 0, 0, 0, 0))
    reply = _can_exchange(bus, node_id, request, timeout)
    if reply[:4] != b"\x60" + request[1:4]:
        raise VerificationError("CAN bootloader index or subindex echo mismatch")


def _can_confirm(bus: "can.BusABC", node_id: int) -> bool:
    """Whether the bootloader acknowledges :06 within ten short tries.

    A full transmit queue counts as no answer: a node that is off acknowledges
    nothing. A link that is down has already failed the identity read.
    """
    for _ in range(10):
        try:
            _sdo_write(bus, node_id, SUB_HANDSHAKE)
            return True
        except (TransportError, VerificationError):  # See _catch_can_bootloader.
            time.sleep(0.05)
    return False


def _catch_can_bootloader(bus: "can.BusABC", node_id: int, wait: float) -> None:
    """Repeat :06 through a power cycle.

    While the device is off nothing acknowledges the frames, so a full transmit
    queue is expected; the queued requests reach the bootloader once it starts.
    """
    if not wait:
        raise ResponseTimeout(
            f"CAN node {node_id} answered neither as application nor bootloader"
        )
    log(
        f"Power-cycle the device at CAN node {node_id} now; "
        f"waiting up to {wait:g} s for its bootloader."
    )
    deadline = time.monotonic() + wait
    last = None
    while time.monotonic() < deadline:
        try:
            _sdo_write(bus, node_id, SUB_HANDSHAKE, timeout=0.05)
            log(f"Found the CAN bootloader at node {node_id}")
            return
        except ResponseTimeout:
            pass
        except VerificationError:
            pass  # The 1.1.0 bootloader answers its first frame after power-up with command 0.
        except TransportError as error:
            last = error
            time.sleep(0.05)
    raise ResponseTimeout(
        f"CAN node {node_id} did not answer"
        + (f" ({last})" if last else "")
        + ". Check the bus at 500 kbit/s, then run the update again and power-cycle "
        "the device when asked."
    )


def update_can(
    bus: "can.BusABC",
    node_id: int,
    image_path: str | Path,
    *,
    raw_binary: bool = False,
    verify: bool = True,
    progress: Optional[Callable[[int, int], None]] = None,
    power_cycle_wait: float = 60.0,
    recovery_timeout: float = DEFAULT_RECOVERY_TIMEOUT,
    timeout: float = DEFAULT_INFO_TIMEOUT,
    force: bool = False,
    bus_check: float = 2.0,
) -> UpdateResult:
    """Update one CAN node (1..127); the caller exclusively owns and closes bus.

    CAN writes sequentially at the bootloader's fixed application address; HEX
    addresses are not transmitted. Use an image made for this exact product.
    A reply to the start request means the bootloader refused the image and
    raises DeviceError; no reply is the normal jump. Erase, data and explicit
    rejections always raise; no destructive request is retried.
    progress(written, total) exceptions cancel without a device reset.
    power_cycle_wait>0 waits for a node-8 bootloader through a power cycle when
    the node does not answer at all. recovery_timeout bounds the post-update
    identity wait and timeout its per-read budget (tool extensions: exposed as
    --recovery-timeout/--timeout; SDK cfbc5c0 uses fixed values). verify is
    always on from the CLI: unlike SDK there is no --no-verify switch, keeping
    verification mandatory here. raw_binary is likewise a tool extension.
    bus_check seconds of passive listening refuse to start while another
    master addresses the node; force downgrades that refusal to a warning.
    """
    _validate_node(node_id)
    _check_wait(power_cycle_wait)
    image = load_image(image_path, raw_binary=raw_binary)
    # Single master only: abort before any device write when another tool is
    # addressing the node (force downgrades this to a warning).
    _assert_bus_clear(bus, node_id, window=bus_check, force=force)
    try:
        previous = read_info(bus, node_id, timeout=1.0).firmware_version
    except (ResponseTimeout, DeviceError, ValueError):
        previous = None  # Possibly already in the bootloader.
    except TransportError as error:
        # Frames left unacknowledged, also by an earlier run, fill the transmit
        # queue while the node is off; that is no answer, not a broken link.
        if getattr(error.__cause__, "error_code", None) != errno.ENOBUFS:
            raise
        previous = None
    _drain_can(bus)
    log(f"Entering CAN bootloader at node {node_id}")
    # Both application and bootloader acknowledge :05. Even if its ACK is lost,
    # try :06; resending :05 could reboot an application that has already moved.
    try:
        _sdo_write(bus, node_id, SUB_ENTER_BOOTLOADER)
    except (TransportError, VerificationError):
        pass  # Includes no reply; the confirm below decides.
    time.sleep(0.02)
    if not _can_confirm(bus, node_id):
        if node_id != 8:
            raise ResponseTimeout(
                f"CAN node {node_id} did not answer. After power-up the bootloader answers "
                "at 500 kbit/s as node 8; run the update again with those settings."
            )
        _catch_can_bootloader(bus, node_id, power_cycle_wait)
    if progress:
        progress(0, len(image.data))
    request = bytes((SDO_DOWNLOAD_INITIATE, 0x51, 0x1F, SUB_DOWNLOAD)) + struct.pack(
        "<I", len(image.data)
    )
    log("Erasing CAN application flash")
    try:
        reply = _can_exchange(bus, node_id, request, 8)
        if reply[:4] != b"\x60\x51\x1f\x01":
            raise VerificationError("CAN bootloader download-initiate echo mismatch")
        toggle = 0
        for offset in range(0, len(image.data), 7):
            chunk = image.data[offset : offset + 7]
            written = offset + len(chunk)
            command = toggle
            if written == len(image.data):
                # HiPNUC's final-byte convention differs from CiA 301: seven -> 03.
                command |= 17 - 2 * len(chunk)
            request = bytes((command,)) + chunk.ljust(7, b"\x00")
            reply = _can_exchange(bus, node_id, request, 4)
            if reply[0] != (0x20 | toggle) or reply[1:] != request[1:]:
                raise VerificationError("CAN bootloader segment toggle or data echo mismatch")
            toggle ^= 0x10
            if progress:
                progress(written, len(image.data))
    except HipnucError as error:
        raise _stopped(error) from error
    log("Image transfer acknowledged; starting the application")
    # The bootloader jumps before it can reply; it replies only when it refuses.
    try:
        _sdo_write(bus, node_id, SUB_START_APP, timeout=0.2)
    except ResponseTimeout:
        pass
    else:
        raise DeviceError(
            "The bootloader rejected the image and stays in the bootloader; "
            "check that the image is made for this product"
        )
    if not verify:
        return UpdateResult(len(image.data), image.version, previous, None)
    try:
        restarted = wait_for_node(
            bus, node_id, recovery_timeout=recovery_timeout, timeout=timeout
        )
    except ResponseTimeout as error:
        raise ResponseTimeout(
            f"CAN node {node_id} did not answer after the update. It may still be in the "
            "bootloader or have restarted with default CAN settings; run the update again "
            "at 500 kbit/s with node 8 and an image made for this product, and "
            "power-cycle the device when asked."
        ) from error
    return UpdateResult(
        len(image.data), image.version, previous, _check_started(image, restarted)
    )


# ---------------------------------------------------------------------------
# Bus management and safety gates
# ---------------------------------------------------------------------------


@contextmanager
def open_bus(interface: str) -> Iterator["can.BusABC"]:
    try:
        import can
    except ImportError as exc:
        raise ConfigurationError(
            "python-can is required for CAN access; install it with: pip install python-can"
        ) from exc
    if sys.platform != "linux":
        raise ConfigurationError("SocketCAN requires Linux")
    try:
        bus = can.Bus(
            interface="socketcan", channel=interface, ignore_config=True, fd=False
        )
    except (can.CanError, OSError) as exc:
        raise TransportError(
            f"Cannot open SocketCAN interface {interface!r}: {exc}. "
            f"Check it with: ip -details link show {interface}"
        ) from exc
    try:
        yield bus
    finally:
        try:
            bus.shutdown()
        except (can.CanError, OSError):
            pass


def check_model_compatibility(
    info: DeviceInfo, image_path: str | Path, *, force: bool
) -> Optional[str]:
    """Gate destructive updates on the product name vs the firmware filename."""
    token = extract_model_from_filename(image_path)
    if token is None:
        log("Warning: no model token (HIxxxx) in the firmware filename; model check skipped")
        return None
    product = (info.product_name or "").upper()
    matched = bool(product) and (token in product or product in Path(image_path).name.upper())
    if not matched:
        if not force:
            raise ConfigurationError(
                f"Device product {info.product_name!r} does not match firmware model "
                f"{token!r}. Refusing to flash; pass --force to override."
            )
        log(f"Warning: model mismatch ({info.product_name!r} vs {token!r}) ignored by --force")
    else:
        log(f"Model check passed: {token}")
    return token


def make_progress_logger(total: int) -> Callable[[int, int], None]:
    state = {"last_percent": -10}

    def progress(written: int, image_total: int) -> None:
        percent = int(written * 100 / image_total) if image_total else 100
        if percent - state["last_percent"] >= 10 or percent == 100:
            state["last_percent"] = percent - percent % 10
            log(f"Transfer progress: {percent}% ({written}/{total} bytes)")

    return progress


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------


def cmd_info(args: argparse.Namespace) -> int:
    with open_bus(args.interface) as bus:
        info = read_info(bus, args.node, timeout=args.timeout)
    if args.json:
        emit_json(info.to_dict())
    else:
        emit_info(info)
    return 0


def cmd_update(args: argparse.Namespace) -> int:
    image_path = Path(args.firmware)
    # Preflight: parse and hash the image before the bus is opened at all.
    image = load_image(image_path, raw_binary=args.raw_binary)
    digest = hashlib.sha256(image_path.read_bytes()).hexdigest()
    log(f"Firmware: {image_path} ({len(image.data)} bytes)")
    log(f"SHA-256: {digest}")
    if image.version:
        log(f"Image version: {image.version}" + (f" build {image.build}" if image.build else ""))
    filename_version = extract_version_from_filename(image_path)

    with open_bus(args.interface) as bus:
        before = None
        try:
            before = read_info(bus, args.node, timeout=args.timeout)
        except (ResponseTimeout, DeviceError) as error:
            # No identity. Stop first if another master is addressing the node;
            # then a bootloader acknowledging :06 marks an interrupted update,
            # and flashing it IS the recovery, so continue automatically (the
            # model gate cannot run without an identity). Anything else keeps
            # the strict abort unless --force asks for the power-cycle path.
            _assert_bus_clear(bus, args.node, window=args.bus_check, force=args.force)
            if _can_confirm(bus, args.node):
                log(
                    "Device is in the bootloader (likely an interrupted update); "
                    "recovering automatically. Model check skipped: no identity."
                )
            elif not args.force:
                log(
                    f"Hint: {error}. The device may be stuck in the bootloader; "
                    "retry with --force to skip the model check."
                )
                raise
            else:
                log(
                    f"Warning: {error}. Continuing without identity (--force); "
                    "model check skipped."
                )
        if before is not None:
            log(
                f"Device: product={before.product_name} firmware={before.firmware_version} "
                f"bl={before.bootloader_version} sn={before.serial_number}"
            )
        # Model gate runs before any destructive request is sent.
        if before is None:
            model_expected = model_matched = None
        else:
            model_expected = check_model_compatibility(before, image_path, force=args.force)
            model_matched = (
                None
                if model_expected is None
                else bool((before.product_name or "").upper())
                and (
                    model_expected in (before.product_name or "").upper()
                    or (before.product_name or "").upper() in image_path.name.upper()
                )
            )
        result = update_can(
            bus,
            args.node,
            image_path,
            raw_binary=args.raw_binary,
            progress=make_progress_logger(len(image.data)),
            power_cycle_wait=args.power_cycle_wait,
            recovery_timeout=args.recovery_timeout,
            timeout=args.timeout,
            force=args.force,
            bus_check=args.bus_check,
        )
        log(f"Transfer done: {result.bytes_written} bytes")
        after = read_info(bus, args.node, timeout=args.timeout)
        log(
            f"Device after update: product={after.product_name} "
            f"firmware={after.firmware_version} bl={after.bootloader_version} "
            f"sn={after.serial_number}"
        )

    # The image's embedded version (CRC-protected) is the primary expectation;
    # the filename is only a fallback for images without version data.
    expected_version = image.version or filename_version
    version_changed = (
        None
        if before is None
        or (before.firmware_version is None and after.firmware_version is None)
        else before.firmware_version != after.firmware_version
    )
    if image.version is None and expected_version and after.firmware_version != expected_version:
        raise VerificationError(
            f"Firmware version after update is {after.firmware_version!r}, "
            f"expected {expected_version!r} from the filename"
        )
    report = UpdateReport(
        before=before,
        after=after,
        result=result,
        model_expected=model_expected,
        model_matched=model_matched,
        expected_version=expected_version,
        version_changed=version_changed,
        image_build=image.build,
    )
    if version_changed is False:
        log("Warning: firmware version did not change (same version re-flashed?)")
    elif version_changed:
        log(f"Version: {before.firmware_version} -> {after.firmware_version}")
    if args.json:
        emit_json(report.to_dict())
    else:
        emit_report(report)
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _node_id_arg(value: str) -> int:
    try:
        node_id = int(value, 0)
    except ValueError:
        raise argparse.ArgumentTypeError(f"invalid node id: {value!r}")
    if node_id == HOST_ADDRESS:
        raise argparse.ArgumentTypeError("85 (0x55) is the reserved host address")
    if not 1 <= node_id <= 127:
        raise argparse.ArgumentTypeError("node id must be 1..127")
    return node_id


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="HiPNUC IMU CAN version query and firmware update"
    )
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--interface", default=DEFAULT_INTERFACE,
        help=f"SocketCAN interface (default: {DEFAULT_INTERFACE})",
    )
    common.add_argument(
        "--node", type=_node_id_arg, required=True,
        help="target node id 1..127 (85/0x55 is reserved)",
    )
    common.add_argument(
        "--timeout", type=float, default=DEFAULT_INFO_TIMEOUT,
        help=f"identity read budget in seconds (default: {DEFAULT_INFO_TIMEOUT:g})",
    )
    common.add_argument(
        "--json", action="store_true", help="print machine-readable JSON to stdout",
    )

    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("info", parents=[common], help="read product, firmware and serial number")

    update = sub.add_parser("update", parents=[common], help="update the application firmware")
    update.add_argument("firmware", help="Intel HEX image (.hex) or raw binary with --raw-binary")
    update.add_argument(
        "--raw-binary", action="store_true",
        help="load the image as raw binary instead of Intel HEX",
    )
    update.add_argument(
        "--force", action="store_true",
        help="flash even if the product name does not match the firmware model",
    )
    update.add_argument(
        "--recovery-timeout", type=float, default=DEFAULT_RECOVERY_TIMEOUT,
        help=f"seconds to wait for the new application (default: {DEFAULT_RECOVERY_TIMEOUT:g})",
    )
    update.add_argument(
        "--power-cycle-wait", type=float, default=60.0,
        help="seconds to wait for a node-8 bootloader through a power cycle "
        "when the node does not answer (0: fail immediately)",
    )
    update.add_argument(
        "--bus-check", type=float, default=2.0,
        help="seconds to watch for another master addressing the node before "
        "flashing (0 disables)",
    )
    return parser


def _hex_record(address: int, payload: bytes) -> str:
    record = bytes((len(payload), address >> 8, address & 0xFF, 0)) + payload
    # Doc task 10 wrote bytes(x), which makes x zero bytes; one byte needs [x].
    return ":" + (record + bytes([(-sum(record)) & 0xFF])).hex().upper()


def _synthetic_image_bytes() -> bytes:
    """A minimal valid HiPNUC image body (origin 0): marker, header, CRC32."""
    header = 8
    payload = b"\xA5" * 16
    length = header + 20 + len(payload)
    data = bytearray(length)
    struct.pack_into("<II", data, 0, _MARKER, header)  # marker + header pointer
    struct.pack_into("<IIIII", data, header, _HEADER_MAGIC, 0, 0, length, 0)
    data[header + 20 :] = payload
    checksum = crc32(bytes(data[header + 20 :]), crc32(bytes(data[: header + 16])))
    struct.pack_into("<I", data, header + 16, checksum)
    return bytes(data)


def run_internal_self_tests() -> None:
    def check(condition: bool, message: str) -> None:
        if not condition:
            raise ConfigurationError(f"self-test failed: {message}")

    # Version decoding: 171 -> "1.7.1", 129 -> "1.2.9", 0 -> None.
    raw = bytearray(b"HI1314".ljust(16, b"\0"))
    raw += (171).to_bytes(2, "big") + (129).to_bytes(2, "big")
    raw += b"\0" * 10
    raw += bytes.fromhex("AABBCCDDEEFF0011")
    words = [int.from_bytes(raw[i : i + 2], "big") for i in range(0, 38, 2)]
    info = identity_from_registers(words)
    check(info.product_name == "HI1314", "product name decode")
    check(info.firmware_version == "1.7.1", f"fw version decode ({info.firmware_version})")
    check(info.bootloader_version == "1.2.9", "bl version decode")
    check(info.serial_number == "AABBCCDDEEFF0011", "serial decode")
    check(identity_from_registers([0] * 19).firmware_version is None, "zero version is None")

    # Final segment command bytes: len 7->3, 6->5, ..., 1->15 (HiPNUC convention).
    check([17 - 2 * length for length in range(1, 8)] == [15, 13, 11, 9, 7, 5, 3], "final byte")

    # Model / version extraction.
    check(extract_model_from_filename("HIPNUC_RPO_1.7.1_HI1314.hex") == "HI1314", "model token")
    check(extract_version_from_filename("HIPNUC_RPO_1.7.1_HI1314.hex") == "1.7.1", "version token")

    # Image validation: a synthetic HiPNUC image loads, damaged ones raise.
    with tempfile.TemporaryDirectory() as tmp:
        good = Path(tmp) / "good.hex"
        good.write_text(
            _hex_record(0, _synthetic_image_bytes()) + "\n:00000001FF\n", encoding="ascii"
        )
        parsed = load_image(good)
        check(parsed.address == 0 and len(parsed.data) == 44, "HEX parse")
        check(not parsed.debug and parsed.version is None, "synthetic image fields")
        corrupt_bytes = bytearray(_synthetic_image_bytes())
        corrupt_bytes[28] ^= 0x5A  # first payload byte; the checksum stays stale
        corrupt = Path(tmp) / "corrupt.hex"
        corrupt.write_text(
            _hex_record(0, bytes(corrupt_bytes)) + "\n:00000001FF\n", encoding="ascii"
        )
        try:
            load_image(corrupt)
        except ValueError:
            pass
        else:
            check(False, "CRC corruption must raise")
        plain = Path(tmp) / "plain.hex"
        plain.write_text(":04000000AABBCCDDEE\n:00000001FF\n", encoding="ascii")
        try:
            load_image(plain)
        except ValueError:
            pass
        else:
            check(False, "non-HiPNUC data must raise")
        bad = Path(tmp) / "bad.hex"
        bad.write_text(":04000000AABBCCDDE0\n:00000001FF\n", encoding="ascii")
        try:
            load_image(bad)
        except ValueError:
            pass
        else:
            check(False, "bad checksum must raise")
        raw_file = Path(tmp) / "raw.bin"
        raw_file.write_bytes(b"\x01\x02\x03\x04")
        check(load_image(raw_file, raw_binary=True).data == b"\x01\x02\x03\x04", "raw binary")

    # Version keys compare as integer tuples and reject garbage.
    check(_version_key("1.10.0") == (1, 10, 0), "version key")
    check(_version_key(None) is None and _version_key("x.y.z") is None, "version key invalid")

    # Bus-clear guard: requests aimed at the node block, other traffic passes.
    # The guard logs warnings to stderr; silence them here so self-tests never
    # look like real warnings on the console.
    class _FakeFrame:
        def __init__(self, arbitration_id: int):
            self.arbitration_id = arbitration_id
            self.is_error_frame = False

    class _FakeBus:
        def __init__(self, frames):
            self._frames = list(frames)

        def recv(self, _timeout):
            return self._frames.pop(0) if self._frames else None

    import contextlib
    import io

    with contextlib.redirect_stderr(io.StringIO()):
        quiet = [_FakeFrame(0x0CFF3408), _FakeFrame(0x0CEF5508)]
        _assert_bus_clear(_FakeBus(quiet), 8, window=0.01)  # measurements/replies pass
        for foreign in (0x608, 0x0CEF08AA):  # SDO request / config request to node 8
            try:
                _assert_bus_clear(_FakeBus([_FakeFrame(foreign)]), 8, window=0.01)
            except TransportError:
                pass
            else:
                check(False, f"foreign request 0x{foreign:08X} must abort")
        _assert_bus_clear(_FakeBus([_FakeFrame(0x608)]), 8, window=0.01, force=True)


def main(argv: Optional[list[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        run_internal_self_tests()
        if args.command == "info":
            return cmd_info(args)
        return cmd_update(args)
    except VerificationError as exc:
        log(f"Verification failed: {exc}")
        return 7
    except DeviceError as exc:
        log(f"Device rejected the request: {exc}")
        return 6
    except ResponseTimeout as exc:
        log(f"Timed out: {exc}")
        return 5
    except TransportError as exc:
        log(f"Transport error: {exc}")
        return 4
    except (ConfigurationError, ValueError) as exc:
        log(f"Configuration or firmware error: {exc}")
        return 2
    except OSError as exc:
        log(f"System interface error: {exc}")
        return 4
    except KeyboardInterrupt:
        log("Interrupted by the user; no automatic reset was sent")
        return 130


if __name__ == "__main__":
    sys.exit(main())
