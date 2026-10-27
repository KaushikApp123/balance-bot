"""Software-in-the-loop sim: the firmware control core driving the plant model.

Task timing matches the firmware:
    physics 10 kHz | motor task 1 kHz | IMU + Kalman 500 Hz | balance 200 Hz

Usage:
    python -m balancebot.sim                 # run the default scenario, print metrics
    python -m balancebot.sim --plot          # also plot (needs matplotlib)
    python -m balancebot.sim --save ../docs/sim_response.png
"""
from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass, field

from .core import Core
from .model import Plant

PHYS_HZ = 10_000
MOTOR_HZ = 1_000
IMU_HZ = 500
BALANCE_HZ = 200
TILT_CUTOFF = math.radians(40)


@dataclass
class SilRobot:
    core: Core = field(default_factory=Core)
    plant: Plant = field(default_factory=Plant)
    gyro_bias: float = math.radians(1.5)   # rad/s, unknown to the filter
    accel_noise: float = 0.01              # g
    gyro_noise: float = 0.004              # rad/s
    seed: int = 1
    hold_when_disarmed: bool = False       # dashboard sim: someone is holding it upright

    def __post_init__(self):
        self.rng = random.Random(self.seed)
        self.t = 0.0
        self.tick = 0
        self.armed = False
        self.fallen = False
        self.v_ref = 0.0
        self.v_sp = 0.0
        self.v_meas = 0.0
        self.duty = 0.0
        self.core.reset(0.0, 0.0)

    def place(self, angle: float) -> None:
        """Put the robot down at a tilt, filter pre-converged like after boot."""
        self.plant.place(angle)
        self.core.reset(angle, self.gyro_bias)
        # Let the filter settle for 2 s with the robot held still, the same
        # way the firmware filter runs from boot before you arm it.
        a_f, a_s, a_u, _ = self.plant.imu()
        n = self.rng.gauss
        for _ in range(2 * IMU_HZ):
            self.core.imu(a_f + n(0, self.accel_noise), a_s + n(0, self.accel_noise),
                          a_u + n(0, self.accel_noise),
                          self.gyro_bias + n(0, self.gyro_noise), 1.0 / IMU_HZ)
        self.fallen = False

    def arm(self, on: bool = True) -> None:
        if on and abs(self.plant.s.th) < math.radians(5):
            self.core.reset_controllers()
            self.armed = True
            self.fallen = False
        elif not on:
            self.armed = False

    def step(self) -> None:
        """Advance one physics tick (100 us) and run any tasks that are due."""
        n = self.tick
        if n % (PHYS_HZ // MOTOR_HZ) == 0:
            dt = 1.0 / MOTOR_HZ
            self.v_meas = self.core.speed(0, self.plant.encoder_counts(), dt)
            if self.armed:
                self.duty = self.core.inner(0, self.v_sp, self.v_meas, self.plant.p.vbatt, dt)
            else:
                self.duty = 0.0
        if n % (PHYS_HZ // IMU_HZ) == 0:
            a_f, a_s, a_u, g = self.plant.imu()
            g_noise = self.rng.gauss
            self.core.imu(a_f + g_noise(0, self.accel_noise), a_s + g_noise(0, self.accel_noise),
                          a_u + g_noise(0, self.accel_noise),
                          g + self.gyro_bias + g_noise(0, self.gyro_noise), 1.0 / IMU_HZ)
        if n % (PHYS_HZ // BALANCE_HZ) == 0:
            if self.armed:
                self.v_sp = self.core.outer(self.v_meas, self.v_ref, 1.0 / BALANCE_HZ)
                if abs(self.core.state()["angle"]) > TILT_CUTOFF:
                    self.armed = False          # safety task
                    self.fallen = True
            else:
                self.v_sp = 0.0
        if self.armed or not self.hold_when_disarmed or self.fallen:
            self.plant.step(self.duty, 1.0 / PHYS_HZ)
        self.tick += 1
        self.t = self.tick / PHYS_HZ

    def run_for(self, seconds: float) -> None:
        for _ in range(int(seconds * PHYS_HZ)):
            self.step()

    def telemetry(self) -> dict[str, float]:
        st = self.core.state()
        return {
            "t": self.t,
            "angle": st["angle"],
            "true_angle": self.plant.s.th,
            "rate": st["rate"],
            "angle_sp": st["angle_sp"],
            "p": st["p"], "i": st["i"], "d": st["d"],
            "v_sp": self.v_sp,
            "v_left": self.v_meas, "v_right": self.v_meas,
            "duty_left": self.duty, "duty_right": self.duty,
            "vbatt": self.plant.p.vbatt,
            "x": self.plant.s.x,
            "armed": self.armed,
        }


def default_scenario(robot: SilRobot, log_every: int = 50) -> list[dict]:
    """Start at 4 deg, 0.5 N*0.1 s shove at 3 s, drive 0.3 m/s from 5 to 7 s."""
    log: list[dict] = []
    robot.place(math.radians(4.0))
    robot.arm(True)
    total = 10.0
    for k in range(int(total * PHYS_HZ)):
        t = k / PHYS_HZ
        robot.plant.push_force = 0.5 if 3.0 <= t < 3.1 else 0.0
        robot.v_ref = 0.3 if 5.0 <= t < 7.0 else 0.0
        robot.step()
        if k % log_every == 0:
            log.append(robot.telemetry())
        if robot.fallen:
            break
    return log


def metrics(log: list[dict]) -> dict[str, float]:
    ang = [abs(r["true_angle"]) for r in log]
    est_err = [abs(r["angle"] - r["true_angle"]) for r in log]
    tail = [r for r in log if 8.5 <= r["t"] <= 10.0]
    return {
        "survived_s": log[-1]["t"],
        "max_tilt_deg": math.degrees(max(ang)),
        "max_est_err_deg": math.degrees(max(est_err)),
        "final_tilt_deg": math.degrees(abs(log[-1]["true_angle"])),
        "tail_speed_mps": max((abs(r["v_left"]) for r in tail), default=float("nan")),
        "max_duty": max(abs(r["duty_left"]) for r in log),
        "drift_m": log[-1]["x"],
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plot", action="store_true", help="show plots (needs matplotlib)")
    ap.add_argument("--save", metavar="PNG", help="save the plot to a file")
    ap.add_argument("--kp", type=float), ap.add_argument("--kd", type=float)
    args = ap.parse_args()

    robot = SilRobot()
    if args.kp is not None or args.kd is not None:
        kp, ki, kd = robot.core.get_gains(0)
        robot.core.set_gains(0, args.kp if args.kp is not None else kp, ki,
                             args.kd if args.kd is not None else kd)
    log = default_scenario(robot)
    for k, v in metrics(log).items():
        print(f"{k:>16}: {v:8.3f}")

    if args.plot or args.save:
        import matplotlib
        if args.save and not args.plot:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        t = [r["t"] for r in log]
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 6.5))
        ax[0].plot(t, [math.degrees(r["true_angle"]) for r in log], label="true tilt")
        ax[0].plot(t, [math.degrees(r["angle"]) for r in log], label="Kalman estimate", alpha=0.85)
        ax[0].plot(t, [math.degrees(r["angle_sp"]) for r in log], "--", label="lean setpoint")
        ax[0].set_ylabel("deg")
        ax[1].plot(t, [r["v_sp"] for r in log], label="speed setpoint")
        ax[1].plot(t, [r["v_left"] for r in log], label="measured wheel speed", alpha=0.85)
        ax[1].plot(t, [0.3 if 5.0 <= x < 7.0 else 0.0 for x in t], ":", label="drive command")
        ax[1].set_ylabel("m/s")
        ax[2].plot(t, [r["duty_left"] for r in log])
        ax[2].set_ylabel("PWM duty")
        ax[2].set_xlabel("time (s)")
        for a in ax:
            a.axvspan(3.0, 3.1, color="red", alpha=0.15)
            a.grid(alpha=0.3)
        ax[0].legend(loc="upper right", fontsize=8)
        ax[1].legend(loc="upper right", fontsize=8)
        ax[0].set_title("SIL sim: start at 4 deg, 0.5 N shove at 3 s (red), "
                        "drive 0.3 m/s from 5 to 7 s", fontsize=10)
        fig.tight_layout()
        if args.save:
            fig.savefig(args.save, dpi=110)
        if args.plot:
            plt.show()


if __name__ == "__main__":
    main()
