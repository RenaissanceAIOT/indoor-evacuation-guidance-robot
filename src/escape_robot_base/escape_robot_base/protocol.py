"""Pure-Python implementation of the laboratory base serial protocol.

The implementation is intentionally independent from ROS so it can be verified
with recorded byte streams and unit tests. Multi-byte values are signed int16
in big-endian order.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi
from struct import pack, unpack
from typing import Iterable

HEADER = bytes((0xAA, 0x55))
MIN_FRAME_LENGTH = 5
MAX_FRAME_LENGTH = 255

TELEMETRY_CODE = 0x10
VELOCITY_CODE = 0x50
IMU_CALIBRATION_CODE = 0x51
LIGHT_CODE = 0x52
LIGHT_SAVE_CODE = 0x53
BEEP_CODE = 0x54
MODEL_CODE = 0x5A
ARM_CODE = 0x60
ARM_CALIBRATION_CODE = 0x6F

STANDARD_GRAVITY = 9.80665


class ProtocolError(ValueError):
    """Raised when a complete frame violates the protocol contract."""


@dataclass(frozen=True, slots=True)
class Frame:
    code: int
    payload: bytes
    raw: bytes


@dataclass(frozen=True, slots=True)
class Telemetry:
    """Converted telemetry values in SI units."""

    acceleration_mps2: tuple[float, float, float]
    angular_velocity_rps: tuple[float, float, float]
    velocity_mps: tuple[float, float, float]
    battery_voltage: float


def _checksum(data: bytes | bytearray) -> int:
    return sum(data) & 0xFF


def _int16(value: float, scale: float, *, minimum: float, maximum: float) -> bytes:
    if not all(isfinite(float(item)) for item in (value, scale, minimum, maximum)):
        raise ProtocolError("command fields must be finite")
    if minimum > maximum:
        raise ProtocolError("invalid command limits")
    bounded = max(minimum, min(maximum, float(value)))
    encoded = int(round(bounded * scale))
    if not -32768 <= encoded <= 32767:
        raise ProtocolError(f"scaled int16 out of range: {encoded}")
    return pack(">h", encoded)


def build_frame(code: int, payload: bytes = b"") -> bytes:
    if not 0 <= code <= 0xFF:
        raise ProtocolError(f"frame code must be uint8, got {code}")
    length = len(HEADER) + 1 + 1 + len(payload) + 1
    if not MIN_FRAME_LENGTH <= length <= MAX_FRAME_LENGTH:
        raise ProtocolError(f"invalid frame length: {length}")
    body = HEADER + bytes((length, code)) + payload
    return body + bytes((_checksum(body),))


def build_velocity_command(
    vx_mps: float,
    vy_mps: float,
    wz_rps: float,
    *,
    max_vx_mps: float = 0.6,
    max_vy_mps: float = 0.6,
    max_wz_rps: float = 1.5,
) -> bytes:
    """Build the 11-byte chassis command frame (code 0x50)."""

    payload = b"".join(
        (
            _int16(vx_mps, 1000.0, minimum=-max_vx_mps, maximum=max_vx_mps),
            _int16(vy_mps, 1000.0, minimum=-max_vy_mps, maximum=max_vy_mps),
            _int16(wz_rps, 1000.0, minimum=-max_wz_rps, maximum=max_wz_rps),
        )
    )
    return build_frame(VELOCITY_CODE, payload)


def build_arm_command(joint_radians: Iterable[float]) -> bytes:
    """Build a six-servo command (code 0x60).

    Six int16 angles produce a 17-byte frame. Confirm the actual controller
    firmware with a hardware capture before commanding a loaded arm.
    """

    positions = list(joint_radians)
    if len(positions) != 6:
        raise ProtocolError(f"expected 6 arm joints, got {len(positions)}")
    payload = b"".join(
        _int16(angle * 180.0 / pi, 10.0, minimum=-90.0, maximum=90.0)
        for angle in positions
    )
    return build_frame(ARM_CODE, payload)


def build_imu_calibration_command() -> bytes:
    return build_frame(IMU_CALIBRATION_CODE, b"\x55")


def build_light_command(
    mode: int,
    submode: int = 1,
    period: int = 0,
    red: int = 255,
    green: int = 0,
    blue: int = 0,
) -> bytes:
    values = (mode, submode, period, red, green, blue)
    if not (1 <= mode <= 10 and 1 <= submode <= 10):
        raise ProtocolError("light mode and submode must be in [1, 10]")
    if any(not 0 <= value <= 255 for value in values[2:]):
        raise ProtocolError("light period/RGB values must be uint8")
    return build_frame(LIGHT_CODE, bytes(values))


def build_beep_command(enabled: bool) -> bytes:
    return build_frame(BEEP_CODE, bytes((int(bool(enabled)),)))


def build_model_command(mecanum: bool = True) -> bytes:
    return build_frame(MODEL_CODE, bytes((0x01 if mecanum else 0x02,)))


def decode_telemetry(frame: Frame | bytes) -> Telemetry:
    if isinstance(frame, bytes):
        frames = FrameParser().feed(frame)
        if len(frames) != 1:
            raise ProtocolError("expected exactly one complete telemetry frame")
        frame = frames[0]
    if frame.code != TELEMETRY_CODE:
        raise ProtocolError(f"expected telemetry code 0x10, got 0x{frame.code:02X}")
    if len(frame.payload) != 20:
        raise ProtocolError(f"telemetry payload must be 20 bytes, got {len(frame.payload)}")

    ax, ay, az, gx, gy, gz, vx, vy, wz, voltage = unpack(">hhhhhhhhhh", frame.payload)
    acceleration = tuple(raw / 32768.0 * 2.0 * STANDARD_GRAVITY for raw in (ax, ay, az))
    angular_velocity = tuple(raw / 32768.0 * 500.0 * pi / 180.0 for raw in (gx, gy, gz))
    velocity = (vx / 1000.0, vy / 1000.0, wz / 1000.0)
    return Telemetry(
        acceleration_mps2=acceleration,
        angular_velocity_rps=angular_velocity,
        velocity_mps=velocity,
        battery_voltage=voltage / 100.0,
    )


class FrameParser:
    """Incremental, noise-tolerant frame parser."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bad_checksums = 0
        self.bad_lengths = 0
        self.discarded_bytes = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def feed(self, data: bytes | bytearray) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []

        while True:
            header_at = self._buffer.find(HEADER)
            if header_at < 0:
                keep = 1 if self._buffer[-1:] == HEADER[:1] else 0
                discarded = len(self._buffer) - keep
                self.discarded_bytes += discarded
                if discarded:
                    del self._buffer[:discarded]
                break
            if header_at:
                self.discarded_bytes += header_at
                del self._buffer[:header_at]
            if len(self._buffer) < 4:
                break

            length = self._buffer[2]
            if not MIN_FRAME_LENGTH <= length <= MAX_FRAME_LENGTH:
                self.bad_lengths += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            if len(self._buffer) < length:
                break

            candidate = bytes(self._buffer[:length])
            if _checksum(candidate[:-1]) != candidate[-1]:
                self.bad_checksums += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue

            frames.append(Frame(code=candidate[3], payload=candidate[4:-1], raw=candidate))
            del self._buffer[:length]

        return frames
