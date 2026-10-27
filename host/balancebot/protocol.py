"""Frame format shared with firmware/core/inc/protocol.h.

Frame: AA 55 | type | len | payload | crc16 (LE)
CRC-16/CCITT-FALSE over type, len and payload.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterator

SYNC = b"\xAA\x55"
MAX_PAYLOAD = 64


class Msg(IntEnum):
    TELEMETRY = 0x01
    GAINS = 0x02
    ACK = 0x03
    SET_GAINS = 0x10
    GET_GAINS = 0x11
    SAVE_GAINS = 0x12
    ARM = 0x13
    DRIVE = 0x14
    SET_TRIM = 0x15


class State(IntEnum):
    BOOT = 0
    DISARMED = 1
    ARMED = 2
    FAULT = 3


FAULT_BITS = {0x01: "TILT", 0x02: "IMU", 0x04: "BATT", 0x08: "DRIVER", 0x10: "TASK"}
ACK_TEXT = {0: "ok", 1: "bad argument", 2: "busy", 3: "refused (disarm first)", 4: "failed"}


def fault_names(bits: int) -> list[str]:
    return [name for bit, name in FAULT_BITS.items() if bits & bit]


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(msg: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes([msg, len(payload)]) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt(body))


# ---- payload layouts (must match the packed C structs) ----
TELEMETRY_FMT = struct.Struct("<I12fBB")     # 54 bytes
GAINS_FMT = struct.Struct("<10f")            # 40 bytes
SET_GAINS_FMT = struct.Struct("<B3f")        # 13 bytes
ACK_FMT = struct.Struct("<BB")
DRIVE_FMT = struct.Struct("<2f")
TRIM_FMT = struct.Struct("<f")

TELEMETRY_FIELDS = ("t_ms", "angle", "rate", "angle_sp", "p", "i", "d", "v_sp",
                    "v_left", "v_right", "duty_left", "duty_right", "vbatt", "state", "faults")


@dataclass
class Gains:
    kp: list[float]
    ki: list[float]
    kd: list[float]
    trim: float


def decode_telemetry(payload: bytes) -> dict:
    return dict(zip(TELEMETRY_FIELDS, TELEMETRY_FMT.unpack(payload)))


def encode_telemetry(t: dict) -> bytes:
    return encode(Msg.TELEMETRY, TELEMETRY_FMT.pack(*(t[k] for k in TELEMETRY_FIELDS)))


def decode_gains(payload: bytes) -> Gains:
    v = GAINS_FMT.unpack(payload)
    return Gains(list(v[0:3]), list(v[3:6]), list(v[6:9]), v[9])


def encode_gains(g: Gains) -> bytes:
    return encode(Msg.GAINS, GAINS_FMT.pack(*g.kp, *g.ki, *g.kd, g.trim))


# ---- host -> robot commands ----
def cmd_set_gains(loop: int, kp: float, ki: float, kd: float) -> bytes:
    return encode(Msg.SET_GAINS, SET_GAINS_FMT.pack(loop, kp, ki, kd))


def cmd_get_gains() -> bytes:
    return encode(Msg.GET_GAINS)


def cmd_save_gains() -> bytes:
    return encode(Msg.SAVE_GAINS)


def cmd_arm(on: bool) -> bytes:
    return encode(Msg.ARM, bytes([1 if on else 0]))


def cmd_drive(v_ref: float, yaw_rate: float) -> bytes:
    return encode(Msg.DRIVE, DRIVE_FMT.pack(v_ref, yaw_rate))


def cmd_set_trim(trim_rad: float) -> bytes:
    return encode(Msg.SET_TRIM, TRIM_FMT.pack(trim_rad))


class FrameParser:
    """Byte-stream parser. Feed it whatever BLE hands you, get whole frames out."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.crc_errors = 0
        self.frames = 0

    def feed(self, data: bytes) -> Iterator[tuple[int, bytes]]:
        self._buf += data
        buf = self._buf
        while True:
            start = buf.find(SYNC)
            if start < 0:
                # keep a trailing 0xAA in case the 0x55 is in the next packet
                del buf[: max(0, len(buf) - 1)]
                return
            if start:
                del buf[:start]
            if len(buf) < 4:
                return
            length = buf[3]
            if length > MAX_PAYLOAD:
                del buf[:1]           # bad length: resync one byte later
                continue
            total = 4 + length + 2
            if len(buf) < total:
                return
            body = bytes(buf[2 : 4 + length])
            (crc,) = struct.unpack_from("<H", buf, 4 + length)
            if crc == crc16_ccitt(body):
                self.frames += 1
                del buf[:total]
                yield body[0], body[2:]
            else:
                self.crc_errors += 1
                del buf[:1]
