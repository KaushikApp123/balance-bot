"""ctypes wrapper around the firmware control core (libbbcore).

Build the library first:
    cmake -S firmware -B build-host && cmake --build build-host
"""
from __future__ import annotations

import ctypes
import os
import sys
from pathlib import Path

ANGLE, SPEED, VELOCITY = 0, 1, 2
LOOP_NAMES = ("angle", "speed", "velocity")

_c_float_p = ctypes.POINTER(ctypes.c_float)


def _lib_name() -> str:
    if sys.platform.startswith("win"):
        return "bbcore.dll"
    if sys.platform == "darwin":
        return "libbbcore.dylib"
    return "libbbcore.so"


def find_library() -> Path:
    env = os.environ.get("BBCORE_LIB")
    if env:
        return Path(env)
    root = Path(__file__).resolve().parents[2]
    name = _lib_name()
    for build in ("build-host", "build-host/Release", "build-host/Debug", "build"):
        p = root / build / name
        if p.exists():
            return p
    raise FileNotFoundError(
        f"{name} not found. Build it with:\n"
        "  cmake -S firmware -B build-host && cmake --build build-host\n"
        "or point BBCORE_LIB at it."
    )


class Core:
    """One instance of the robot's control core (Kalman + cascaded PID)."""

    def __init__(self, lib_path: Path | None = None):
        lib = ctypes.CDLL(str(lib_path or find_library()))
        f, i, vp = ctypes.c_float, ctypes.c_int, ctypes.c_void_p
        sig = {
            "sil_create": (vp, []),
            "sil_destroy": (None, [vp]),
            "sil_set_gains": (None, [vp, i, f, f, f]),
            "sil_get_gains": (None, [vp, i, _c_float_p]),
            "sil_set_trim": (None, [vp, f]),
            "sil_set_kalman": (None, [vp, f, f, f]),
            "sil_get_trim": (f, [vp]),
            "sil_reset": (None, [vp, f, f]),
            "sil_reset_ctrl": (None, [vp]),
            "sil_speed": (f, [vp, i, ctypes.c_uint, f]),
            "sil_imu": (f, [vp, f, f, f, f, f]),
            "sil_outer": (f, [vp, f, f, f]),
            "sil_inner": (f, [vp, i, f, f, f, f]),
            "sil_state": (None, [vp, _c_float_p]),
        }
        for name, (res, args) in sig.items():
            fn = getattr(lib, name)
            fn.restype = res
            fn.argtypes = args
        self._lib = lib
        self._h = lib.sil_create()
        if not self._h:
            raise MemoryError("sil_create failed")

    def __del__(self):
        h = getattr(self, "_h", None)
        if h:
            self._lib.sil_destroy(h)
            self._h = None

    # gains
    def set_gains(self, loop: int, kp: float, ki: float, kd: float) -> None:
        self._lib.sil_set_gains(self._h, loop, kp, ki, kd)

    def get_gains(self, loop: int) -> tuple[float, float, float]:
        buf = (ctypes.c_float * 3)()
        self._lib.sil_get_gains(self._h, loop, buf)
        return tuple(buf)  # type: ignore[return-value]

    def set_kalman(self, q_angle: float, q_bias: float, r: float) -> None:
        self._lib.sil_set_kalman(self._h, q_angle, q_bias, r)

    def set_trim(self, trim: float) -> None:
        self._lib.sil_set_trim(self._h, trim)

    def get_trim(self) -> float:
        return self._lib.sil_get_trim(self._h)

    # task steps
    def reset(self, angle: float = 0.0, bias: float = 0.0) -> None:
        self._lib.sil_reset(self._h, angle, bias)

    def reset_controllers(self) -> None:
        self._lib.sil_reset_ctrl(self._h)

    def speed(self, wheel: int, raw: int, dt: float) -> float:
        return self._lib.sil_speed(self._h, wheel, raw & 0xFFFF, dt)

    def imu(self, a_fwd: float, a_side: float, a_up: float, gyro: float, dt: float) -> float:
        return self._lib.sil_imu(self._h, a_fwd, a_side, a_up, gyro, dt)

    def outer(self, v_meas: float, v_ref: float, dt: float) -> float:
        return self._lib.sil_outer(self._h, v_meas, v_ref, dt)

    def inner(self, wheel: int, v_sp: float, v_meas: float, vbatt: float, dt: float) -> float:
        return self._lib.sil_inner(self._h, wheel, v_sp, v_meas, vbatt, dt)

    def state(self) -> dict[str, float]:
        buf = (ctypes.c_float * 8)()
        self._lib.sil_state(self._h, buf)
        keys = ("angle", "rate", "bias", "angle_sp", "v_sp", "p", "i", "d")
        return dict(zip(keys, buf))
