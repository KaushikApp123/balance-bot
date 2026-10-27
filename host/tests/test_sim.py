"""Closed-loop checks using the real firmware control core (needs libbbcore)."""
import math

import pytest

try:
    from balancebot.core import Core, find_library
    find_library()
    HAVE_LIB = True
except (FileNotFoundError, OSError):
    HAVE_LIB = False

pytestmark = pytest.mark.skipif(not HAVE_LIB, reason="build libbbcore first (cmake -S firmware -B build-host)")


def test_default_gains_balance_and_survive_push():
    from balancebot.sim import SilRobot, default_scenario, metrics
    m = metrics(default_scenario(SilRobot()))
    assert m["survived_s"] > 9.9
    assert m["max_tilt_deg"] < 8.0
    assert m["max_est_err_deg"] < 2.0


@pytest.mark.parametrize("override", [
    {"com_height": 0.06}, {"com_height": 0.13}, {"body_mass": 1.3}, {"vbatt": 10.0},
])
def test_gains_are_robust_to_model_error(override):
    from balancebot.model import Params
    from balancebot.sim import SilRobot, default_scenario, metrics
    r = SilRobot()
    r.plant.p = Params(**override)
    assert metrics(default_scenario(r))["survived_s"] > 9.9


def test_safety_cutoff_when_it_cannot_recover():
    from balancebot.sim import SilRobot, PHYS_HZ
    r = SilRobot()
    r.place(math.radians(4))
    r.arm(True)
    r.core.set_gains(0, 5.0, 0.0, 0.0)   # far too soft to hold it up
    for _ in range(3 * PHYS_HZ):
        r.step()
        if r.fallen:
            break
    assert r.fallen and not r.armed


def test_arming_refused_when_tilted():
    from balancebot.sim import SilRobot
    r = SilRobot()
    r.place(math.radians(20))
    r.arm(True)
    assert not r.armed


def test_core_gain_roundtrip():
    c = Core()
    c.set_gains(1, 1.0, 2.0, 3.0)
    assert c.get_gains(1) == pytest.approx((1.0, 2.0, 3.0))
