"""Transport links for the dashboard.

Both links have the same tiny interface:
    start(on_bytes)   begin delivering raw robot->host bytes to on_bytes(bytes)
    send(bytes)       queue host->robot bytes
    stop()

BleLink talks to the ESP32-C3 bridge (Nordic UART Service) through bleak.
SimLink runs the SIL sim in a thread and speaks the same byte protocol, so the
dashboard cannot tell the difference. Good for UI work without the robot.
"""
from __future__ import annotations

import asyncio
import math
import threading
import time
from typing import Callable

from .. import protocol as P

NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # host writes here
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # robot notifies here

OnBytes = Callable[[bytes], None]
OnStatus = Callable[[str], None]


class BleLink:
    def __init__(self, name: str = "BalanceBot", address: str | None = None,
                 on_status: OnStatus | None = None):
        self.name = name
        self.address = address
        self.on_status = on_status or (lambda s: None)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._client = None
        self._stop: asyncio.Event | None = None
        self._tx: asyncio.Queue[bytes] | None = None

    def start(self, on_bytes: OnBytes) -> None:
        self._thread = threading.Thread(target=self._run, args=(on_bytes,), daemon=True)
        self._thread.start()

    def send(self, data: bytes) -> None:
        if self._loop and self._tx:
            self._loop.call_soon_threadsafe(self._tx.put_nowait, data)

    def stop(self) -> None:
        if self._loop and self._stop:
            self._loop.call_soon_threadsafe(self._stop.set)
        if self._thread:
            self._thread.join(timeout=3)

    def _run(self, on_bytes: OnBytes) -> None:
        asyncio.run(self._main(on_bytes))

    async def _main(self, on_bytes: OnBytes) -> None:
        from bleak import BleakClient, BleakScanner

        self._loop = asyncio.get_running_loop()
        self._stop = asyncio.Event()
        self._tx = asyncio.Queue()

        while not self._stop.is_set():
            try:
                addr = self.address
                if addr is None:
                    self.on_status(f"scanning for {self.name}...")
                    dev = await BleakScanner.find_device_by_name(self.name, timeout=8.0)
                    if dev is None:
                        self.on_status("not found, retrying")
                        continue
                    addr = dev.address
                self.on_status(f"connecting to {addr}...")
                async with BleakClient(addr) as client:
                    self._client = client
                    await client.start_notify(NUS_TX, lambda _h, data: on_bytes(bytes(data)))
                    self.on_status(f"connected ({addr})")
                    while not self._stop.is_set() and client.is_connected:
                        try:
                            data = await asyncio.wait_for(self._tx.get(), timeout=0.2)
                        except asyncio.TimeoutError:
                            continue
                        # frames are <= 70 bytes; split for small-MTU links
                        mtu = max(20, (client.mtu_size or 23) - 3)
                        for i in range(0, len(data), mtu):
                            await client.write_gatt_char(NUS_RX, data[i:i + mtu], response=False)
                self.on_status("disconnected")
            except Exception as e:  # noqa: BLE001 - keep the link alive whatever bleak throws
                self.on_status(f"link error: {e}")
                await asyncio.sleep(1.0)


class SimLink:
    """Runs the SIL robot in real time and exchanges protocol frames with it."""

    def __init__(self, on_status: OnStatus | None = None, start_tilt_deg: float = 3.0):
        self.on_status = on_status or (lambda s: None)
        self.start_tilt = math.radians(start_tilt_deg)
        self._thread: threading.Thread | None = None
        self._running = False
        self._inbox: list[bytes] = []
        self._lock = threading.Lock()

    def start(self, on_bytes: OnBytes) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._run, args=(on_bytes,), daemon=True)
        self._thread.start()

    def send(self, data: bytes) -> None:
        with self._lock:
            self._inbox.append(data)

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)

    def push(self, newtons: float = 0.6, seconds: float = 0.1) -> None:
        """Shove the simulated robot (dashboard 'push' button)."""
        self._push_until = time.monotonic() + seconds
        self._push_force = newtons

    def _gains_frame(self, robot) -> bytes:
        g = [robot.core.get_gains(i) for i in range(3)]
        return P.encode_gains(P.Gains([x[0] for x in g], [x[1] for x in g], [x[2] for x in g],
                                      robot.core.get_trim()))

    def _ack(self, msg: int, status: int = 0) -> bytes:
        return P.encode(P.Msg.ACK, P.ACK_FMT.pack(msg, status))

    def _run(self, on_bytes: OnBytes) -> None:
        from ..sim import PHYS_HZ, SilRobot

        robot = SilRobot(hold_when_disarmed=True)
        robot.place(self.start_tilt)
        self._push_until = 0.0
        self._push_force = 0.0
        parser = P.FrameParser()
        drive_t = 0.0
        self.on_status("sim running: robot held at 3 deg, press Arm")
        steps_per_frame = PHYS_HZ // 50
        t_next = time.monotonic()
        while self._running:
            with self._lock:
                inbox, self._inbox = self._inbox, []
            replies = b""
            for chunk in inbox:
                for msg, payload in parser.feed(chunk):
                    if msg == P.Msg.SET_GAINS:
                        loop, kp, ki, kd = P.SET_GAINS_FMT.unpack(payload)
                        robot.core.set_gains(loop, kp, ki, kd)
                        replies += self._ack(msg) + self._gains_frame(robot)
                    elif msg == P.Msg.GET_GAINS:
                        replies += self._gains_frame(robot)
                    elif msg == P.Msg.SET_TRIM:
                        robot.core.set_trim(P.TRIM_FMT.unpack(payload)[0])
                        replies += self._ack(msg) + self._gains_frame(robot)
                    elif msg == P.Msg.SAVE_GAINS:
                        replies += self._ack(msg, 3 if robot.armed else 0)
                    elif msg == P.Msg.ARM:
                        if payload[0]:
                            if robot.fallen or abs(robot.plant.s.th) > math.radians(5):
                                robot.place(self.start_tilt)    # stand it back up
                            robot.arm(True)
                        else:
                            robot.arm(False)
                        replies += self._ack(msg)
                    elif msg == P.Msg.DRIVE:
                        robot.v_ref, _yaw = P.DRIVE_FMT.unpack(payload)
                        drive_t = time.monotonic()
            if time.monotonic() - drive_t > 0.5:
                robot.v_ref = 0.0

            now = time.monotonic()
            robot.plant.push_force = self._push_force if now < self._push_until else 0.0
            for _ in range(steps_per_frame):
                robot.step()

            tel = robot.telemetry()
            state = P.State.ARMED if robot.armed else (P.State.FAULT if robot.fallen else P.State.DISARMED)
            frame = P.encode_telemetry({
                "t_ms": int(tel["t"] * 1000) & 0xFFFFFFFF,
                "angle": tel["angle"], "rate": tel["rate"], "angle_sp": tel["angle_sp"],
                "p": tel["p"], "i": tel["i"], "d": tel["d"], "v_sp": tel["v_sp"],
                "v_left": tel["v_left"], "v_right": tel["v_right"],
                "duty_left": tel["duty_left"], "duty_right": tel["duty_right"],
                "vbatt": tel["vbatt"], "state": int(state),
                "faults": 0x01 if robot.fallen else 0,
            })
            data = frame + replies
            # deliver in BLE-sized pieces to exercise the parser like the real link
            for i in range(0, len(data), 20):
                on_bytes(data[i:i + 20])

            t_next += 0.02
            sleep = t_next - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                t_next = time.monotonic()
