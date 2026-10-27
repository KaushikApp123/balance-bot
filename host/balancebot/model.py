"""Planar wheeled-inverted-pendulum model with DC gear motors.

State: x (m), x_dot, theta (rad, + = leaning forward), theta_dot.
Both wheels are lumped into one (the sim drives them with the same duty
unless a yaw command is given, which this planar model ignores).

Numbers are estimates for the build in hardware/README.md. Weigh your robot
and measure the COM height before trusting the gains this produces.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

G = 9.81


@dataclass
class Params:
    body_mass: float = 0.90          # kg, chassis + PCB + 3S battery
    com_height: float = 0.09         # m, axle to body COM
    body_inertia: float = 0.0035     # kg m^2 about the COM
    wheel_mass: float = 0.05         # kg each
    wheel_radius: float = 0.0325     # m
    # wheel + reflected rotor inertia (rotor ~5e-7 kg m^2 x 30^2), each
    wheel_inertia: float = 0.5 * 0.05 * 0.0325**2 + 5e-7 * 30**2
    motor_k: float = 0.347           # V/(rad/s) and N m/A at the output shaft
    motor_r: float = 5.0             # ohm
    coulomb: float = 0.02            # N m static/kinetic friction per motor
    viscous: float = 0.002           # N m/(rad/s) per motor
    imu_height: float = 0.06         # m, axle to IMU
    enc_cpr: float = 1320.0
    vbatt: float = 11.4


@dataclass
class State:
    x: float = 0.0
    xd: float = 0.0
    th: float = 0.0
    thd: float = 0.0
    # last accelerations, needed to synthesize the accelerometer
    xdd: float = 0.0
    thdd: float = 0.0


@dataclass
class Plant:
    p: Params = field(default_factory=Params)
    s: State = field(default_factory=State)
    push_force: float = 0.0          # N, horizontal force on the body COM
    enc_offset: float = 0.0          # rad, so counts start at 0 where we place the robot

    def place(self, angle: float) -> None:
        self.s = State(th=angle)
        self.enc_offset = -angle

    def wheel_rel_speed(self) -> float:
        """Wheel angular speed relative to the body (what the encoder sees)."""
        return self.s.xd / self.p.wheel_radius - self.s.thd

    def motor_torque(self, duty: float) -> float:
        p = self.p
        w = self.wheel_rel_speed()
        v = max(-1.0, min(1.0, duty)) * p.vbatt
        i = (v - p.motor_k * w) / p.motor_r
        tau = p.motor_k * i - p.viscous * w
        # Coulomb friction, smoothed so the integrator does not chatter
        tau -= p.coulomb * math.tanh(w / 0.5)
        return tau

    def step(self, duty: float, dt: float) -> None:
        p, s = self.p, self.s
        tau = 2.0 * self.motor_torque(duty)          # two motors
        M, l, r = p.body_mass, p.com_height, p.wheel_radius
        m_eff = M + 2 * p.wheel_mass + 2 * p.wheel_inertia / r**2
        Ib = p.body_inertia + M * l * l
        c, sn = math.cos(s.th), math.sin(s.th)

        # [m_eff   M l c] [xdd ]   [ tau/r + M l sn thd^2 + F       ]
        # [M l c   Ib   ] [thdd] = [ M g l sn - tau + F l c         ]
        a11, a12, a21, a22 = m_eff, M * l * c, M * l * c, Ib
        b1 = tau / r + M * l * sn * s.thd**2 + self.push_force
        b2 = M * G * l * sn - tau + self.push_force * l * c
        det = a11 * a22 - a12 * a21
        xdd = (b1 * a22 - a12 * b2) / det
        thdd = (a11 * b2 - a21 * b1) / det

        # semi-implicit Euler is plenty at a 10 kHz physics step
        s.xd += xdd * dt
        s.thd += thdd * dt
        s.x += s.xd * dt
        s.th += s.thd * dt
        s.xdd, s.thdd = xdd, thdd

        # wheels on the ground cannot go below the floor: if it falls, stop
        if abs(s.th) > math.radians(80):
            s.th = math.copysign(math.radians(80), s.th)
            s.thd = 0.0

    def imu(self) -> tuple[float, float, float, float]:
        """Ideal accel (g; forward, side, up) and pitch gyro (rad/s) at the IMU."""
        s, h = self.s, self.p.imu_height
        c, sn = math.cos(s.th), math.sin(s.th)
        ax = s.xdd + h * c * s.thdd - h * sn * s.thd**2
        az = -h * sn * s.thdd - h * c * s.thd**2 + G
        a_fwd = (ax * c - az * sn) / G
        a_up = (ax * sn + az * c) / G
        return a_fwd, 0.0, a_up, s.thd

    def encoder_counts(self) -> int:
        p, s = self.p, self.s
        rel_angle = s.x / p.wheel_radius - s.th - self.enc_offset
        return int(math.floor(rel_angle / (2 * math.pi) * p.enc_cpr))
