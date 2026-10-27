"""Live telemetry and PID tuning dashboard.

    python -m balancebot.dashboard              # connect to the robot over BLE
    python -m balancebot.dashboard --sim        # drive the SIL sim instead
    python -m balancebot.dashboard --address AA:BB:CC:DD:EE:FF

Keys while the window has focus: W/S drive forward/back, A/D turn, Space = disarm.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from collections import deque
from pathlib import Path

import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from .. import protocol as P
from .links import BleLink, SimLink

HISTORY_S = 10.0
RATE_HZ = 50
N = int(HISTORY_S * RATE_HZ)
LOOPS = ("Angle", "Speed", "Velocity")
# spinbox (decimals, step, max) per loop, per gain
SPIN = {
    0: ((1, 5.0, 1000.0), (2, 0.5, 200.0), (2, 0.1, 50.0)),
    1: ((2, 0.5, 200.0), (1, 5.0, 2000.0), (3, 0.01, 10.0)),
    2: ((3, 0.01, 5.0), (3, 0.01, 5.0), (3, 0.01, 5.0)),
}
DRIVE_SPEED = 0.25    # m/s
DRIVE_YAW = 1.5       # rad/s


class Bridge(QtCore.QObject):
    """Moves bytes from the link thread onto the Qt thread."""
    bytes_in = QtCore.Signal(bytes)
    status = QtCore.Signal(str)


class Dashboard(QtWidgets.QMainWindow):
    def __init__(self, link, is_sim: bool):
        super().__init__()
        self.setWindowTitle("Balance Bot")
        self.resize(1280, 820)
        self.link = link
        self.is_sim = is_sim
        self.parser = P.FrameParser()
        self.hist = {k: deque(maxlen=N) for k in
                     ("t", "angle", "angle_sp", "p", "i", "d", "v_sp", "v_l", "v_r", "duty_l", "duty_r")}
        self.last_tel: dict | None = None
        self.frames_window: deque[float] = deque(maxlen=200)
        self.csv_writer = None
        self.csv_file = None
        self.keys: set[int] = set()

        self.bridge = Bridge()
        self.bridge.bytes_in.connect(self.on_bytes)
        self.bridge.status.connect(self.on_status)

        self._build_ui()

        self.plot_timer = QtCore.QTimer(self, interval=33, timeout=self.refresh_plots)
        self.plot_timer.start()
        self.drive_timer = QtCore.QTimer(self, interval=100, timeout=self.send_drive)
        self.drive_timer.start()
        self.autosend_timers = [QtCore.QTimer(self, singleShot=True, interval=150) for _ in LOOPS]
        for i, t in enumerate(self.autosend_timers):
            t.timeout.connect(lambda i=i: self.send_loop(i))

        link.on_status = self.bridge.status.emit
        link.start(self.bridge.bytes_in.emit)
        QtCore.QTimer.singleShot(1500, lambda: self.link.send(P.cmd_get_gains()))

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        pg.setConfigOptions(antialias=True)
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        lay = QtWidgets.QHBoxLayout(root)

        # plots
        plots = pg.GraphicsLayoutWidget()
        lay.addWidget(plots, 3)
        self.p_angle = plots.addPlot(row=0, col=0, title="Tilt (deg)")
        self.p_pid = plots.addPlot(row=1, col=0, title="Angle loop terms (m/s²)")
        self.p_wheel = plots.addPlot(row=2, col=0, title="Wheels")
        for p in (self.p_angle, self.p_pid, self.p_wheel):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.addLegend(offset=(10, 5))
        self.p_pid.setXLink(self.p_angle)
        self.p_wheel.setXLink(self.p_angle)
        self.c = {
            "angle": self.p_angle.plot(pen=pg.mkPen("#4cc9f0", width=2), name="angle"),
            "angle_sp": self.p_angle.plot(pen=pg.mkPen("#f72585", width=1, style=QtCore.Qt.DashLine), name="setpoint"),
            "p": self.p_pid.plot(pen=pg.mkPen("#4cc9f0", width=1.5), name="P"),
            "i": self.p_pid.plot(pen=pg.mkPen("#ffd166", width=1.5), name="I"),
            "d": self.p_pid.plot(pen=pg.mkPen("#ef476f", width=1.5), name="D"),
            "v_sp": self.p_wheel.plot(pen=pg.mkPen("#ffffff", width=1, style=QtCore.Qt.DashLine), name="v_sp m/s"),
            "v_l": self.p_wheel.plot(pen=pg.mkPen("#06d6a0", width=1.5), name="v_left m/s"),
            "v_r": self.p_wheel.plot(pen=pg.mkPen("#118ab2", width=1.5), name="v_right m/s"),
            "duty_l": self.p_wheel.plot(pen=pg.mkPen("#ff9f1c", width=1), name="duty_left"),
        }

        # side panel
        side = QtWidgets.QVBoxLayout()
        lay.addLayout(side, 1)

        self.lbl_link = QtWidgets.QLabel("starting...")
        self.lbl_state = QtWidgets.QLabel("state: -")
        self.lbl_state.setStyleSheet("font-size: 18px; font-weight: bold;")
        self.lbl_stats = QtWidgets.QLabel("-")
        side.addWidget(self.lbl_link)
        side.addWidget(self.lbl_state)
        side.addWidget(self.lbl_stats)

        row = QtWidgets.QHBoxLayout()
        b_arm = QtWidgets.QPushButton("Arm")
        b_arm.clicked.connect(lambda: self.link.send(P.cmd_arm(True)))
        b_dis = QtWidgets.QPushButton("Disarm")
        b_dis.clicked.connect(lambda: self.link.send(P.cmd_arm(False)))
        row.addWidget(b_arm)
        row.addWidget(b_dis)
        if self.is_sim:
            b_push = QtWidgets.QPushButton("Push")
            b_push.clicked.connect(lambda: self.link.push())
            row.addWidget(b_push)
        side.addLayout(row)

        # gains grid
        box = QtWidgets.QGroupBox("PID gains (live, no reflash)")
        grid = QtWidgets.QGridLayout(box)
        for c, h in enumerate(("", "Kp", "Ki", "Kd", "")):
            grid.addWidget(QtWidgets.QLabel(h), 0, c)
        self.spins: list[list[QtWidgets.QDoubleSpinBox]] = []
        for r, name in enumerate(LOOPS):
            grid.addWidget(QtWidgets.QLabel(name), r + 1, 0)
            row_spins = []
            for c in range(3):
                dec, step, mx = SPIN[r][c]
                s = QtWidgets.QDoubleSpinBox(decimals=dec, singleStep=step, maximum=mx, minimum=0.0)
                s.valueChanged.connect(lambda _v, r=r: self.on_gain_edit(r))
                grid.addWidget(s, r + 1, c + 1)
                row_spins.append(s)
            b = QtWidgets.QPushButton("Send")
            b.clicked.connect(lambda _c=False, r=r: self.send_loop(r))
            grid.addWidget(b, r + 1, 4)
            self.spins.append(row_spins)
        self.chk_auto = QtWidgets.QCheckBox("Send while editing")
        self.chk_auto.setChecked(True)
        grid.addWidget(self.chk_auto, 4, 0, 1, 3)

        grid.addWidget(QtWidgets.QLabel("Trim (deg)"), 5, 0)
        self.spin_trim = QtWidgets.QDoubleSpinBox(decimals=2, singleStep=0.1, minimum=-10, maximum=10)
        grid.addWidget(self.spin_trim, 5, 1)
        b_trim = QtWidgets.QPushButton("Send")
        b_trim.clicked.connect(lambda: self.link.send(P.cmd_set_trim(math.radians(self.spin_trim.value()))))
        grid.addWidget(b_trim, 5, 4)

        row2 = QtWidgets.QHBoxLayout()
        b_read = QtWidgets.QPushButton("Read from robot")
        b_read.clicked.connect(lambda: self.link.send(P.cmd_get_gains()))
        b_save = QtWidgets.QPushButton("Save to flash")
        b_save.clicked.connect(lambda: self.link.send(P.cmd_save_gains()))
        row2.addWidget(b_read)
        row2.addWidget(b_save)
        grid.addLayout(row2, 6, 0, 1, 5)
        side.addWidget(box)

        self.btn_rec = QtWidgets.QPushButton("Record CSV")
        self.btn_rec.setCheckable(True)
        self.btn_rec.toggled.connect(self.toggle_record)
        side.addWidget(self.btn_rec)

        side.addWidget(QtWidgets.QLabel("Drive: W/S forward/back, A/D turn, Space disarm"))
        self.log = QtWidgets.QPlainTextEdit(readOnly=True)
        self.log.setMaximumBlockCount(300)
        side.addWidget(self.log, 1)

    # ------------------------------------------------------------ incoming
    @QtCore.Slot(bytes)
    def on_bytes(self, data: bytes) -> None:
        for msg, payload in self.parser.feed(data):
            if msg == P.Msg.TELEMETRY and len(payload) == P.TELEMETRY_FMT.size:
                self.on_telemetry(P.decode_telemetry(payload))
            elif msg == P.Msg.GAINS and len(payload) == P.GAINS_FMT.size:
                self.on_gains(P.decode_gains(payload))
            elif msg == P.Msg.ACK and len(payload) == 2:
                m, st = P.ACK_FMT.unpack(payload)
                name = P.Msg(m).name if m in P.Msg._value2member_map_ else hex(m)
                self.say(f"{name}: {P.ACK_TEXT.get(st, st)}")

    def on_telemetry(self, t: dict) -> None:
        now = time.monotonic()
        self.frames_window.append(now)
        self.last_tel = t
        h = self.hist
        h["t"].append(t["t_ms"] / 1000.0)
        h["angle"].append(math.degrees(t["angle"]))
        h["angle_sp"].append(math.degrees(t["angle_sp"]))
        h["p"].append(t["p"]); h["i"].append(t["i"]); h["d"].append(t["d"])
        h["v_sp"].append(t["v_sp"]); h["v_l"].append(t["v_left"]); h["v_r"].append(t["v_right"])
        h["duty_l"].append(t["duty_left"]); h["duty_r"].append(t["duty_right"])
        if self.csv_writer:
            self.csv_writer.writerow([t[k] for k in P.TELEMETRY_FIELDS])

    def on_gains(self, g: P.Gains) -> None:
        for r in range(3):
            for c, v in enumerate((g.kp[r], g.ki[r], g.kd[r])):
                s = self.spins[r][c]
                s.blockSignals(True)
                s.setValue(v)
                s.blockSignals(False)
        self.spin_trim.setValue(math.degrees(g.trim))
        self.say("gains read from robot")

    @QtCore.Slot(str)
    def on_status(self, s: str) -> None:
        self.lbl_link.setText(s)
        self.say(s)

    # ------------------------------------------------------------ outgoing
    def on_gain_edit(self, loop: int) -> None:
        if self.chk_auto.isChecked():
            self.autosend_timers[loop].start()

    def send_loop(self, loop: int) -> None:
        kp, ki, kd = (s.value() for s in self.spins[loop])
        self.link.send(P.cmd_set_gains(loop, kp, ki, kd))

    def send_drive(self) -> None:
        k = self.keys
        if not k:
            return
        v = DRIVE_SPEED * ((QtCore.Qt.Key_W in k) - (QtCore.Qt.Key_S in k))
        w = DRIVE_YAW * ((QtCore.Qt.Key_A in k) - (QtCore.Qt.Key_D in k))
        self.link.send(P.cmd_drive(v, w))

    def keyPressEvent(self, e: QtGui.QKeyEvent) -> None:
        if e.isAutoRepeat():
            return
        if e.key() == QtCore.Qt.Key_Space:
            self.link.send(P.cmd_arm(False))
        elif e.key() in (QtCore.Qt.Key_W, QtCore.Qt.Key_A, QtCore.Qt.Key_S, QtCore.Qt.Key_D):
            self.keys.add(e.key())
            self.send_drive()
        else:
            super().keyPressEvent(e)

    def keyReleaseEvent(self, e: QtGui.QKeyEvent) -> None:
        if e.isAutoRepeat():
            return
        self.keys.discard(e.key())
        if not self.keys:
            self.link.send(P.cmd_drive(0.0, 0.0))

    # ------------------------------------------------------------ misc
    def toggle_record(self, on: bool) -> None:
        if on:
            path = Path(time.strftime("telemetry_%Y%m%d_%H%M%S.csv"))
            self.csv_file = path.open("w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(P.TELEMETRY_FIELDS)
            self.say(f"recording to {path}")
        else:
            if self.csv_file:
                self.csv_file.close()
                self.say("recording stopped")
            self.csv_file = None
            self.csv_writer = None

    def say(self, s: str) -> None:
        self.log.appendPlainText(time.strftime("%H:%M:%S ") + s)

    def refresh_plots(self) -> None:
        h = self.hist
        if not h["t"]:
            return
        t = list(h["t"])
        for k, curve in self.c.items():
            curve.setData(t, list(h[k]))
        tel = self.last_tel
        if tel:
            st = P.State(tel["state"]).name if tel["state"] in P.State._value2member_map_ else "?"
            faults = ",".join(P.fault_names(tel["faults"])) or "none"
            color = {"ARMED": "#06d6a0", "FAULT": "#ef476f"}.get(st, "#ffd166")
            self.lbl_state.setText(f"<span style='color:{color}'>{st}</span>  faults: {faults}")
            fw = self.frames_window
            rate = (len(fw) - 1) / (fw[-1] - fw[0]) if len(fw) > 1 and fw[-1] > fw[0] else 0.0
            self.lbl_stats.setText(
                f"tilt {math.degrees(tel['angle']):+.2f}°   batt {tel['vbatt']:.2f} V   "
                f"{rate:.0f} frames/s   crc err {self.parser.crc_errors}")

    def closeEvent(self, e: QtGui.QCloseEvent) -> None:
        self.toggle_record(False)
        self.link.stop()
        super().closeEvent(e)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="run against the SIL simulation")
    ap.add_argument("--name", default="BalanceBot", help="BLE name to scan for")
    ap.add_argument("--address", help="connect to this BLE address directly")
    ap.add_argument("--quit-after", type=float, help=argparse.SUPPRESS)  # used by the smoke test
    args = ap.parse_args(argv)

    app = QtWidgets.QApplication(sys.argv[:1])
    link = SimLink() if args.sim else BleLink(args.name, args.address)
    win = Dashboard(link, is_sim=args.sim)
    win.show()
    if args.quit_after:
        QtCore.QTimer.singleShot(int(args.quit_after * 1000), win.close)
    return app.exec()
