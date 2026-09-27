# Batch-mode tests of the core_rtheta G-code layer (run with pytest).
import math
import re

import pytest
import radial_gcode
import sim
from test_kinematics import CFG, START, machine_pos, run_ok


def active_span(trace):
    """(first step time, last step time) of a stepper trace."""
    return trace.times[1], trace.times[-1]


def test_inverse_time_radial_ring():
    gcode, final, total_t = radial_gcode.generate(rings=1,
                                                  moves_per_ring=200)
    res = run_ok("THETA_SET_POSITION X=0 C=0 Z=20 B=0\n" + gcode + "M400\n")
    p = machine_pos(res, 1e9, (0.0, 0.0, 20.0, 0.0))
    assert p[0] == pytest.approx(final[0], abs=0.02)
    assert p[1] == pytest.approx(final[1], abs=0.02)
    assert p[2] == pytest.approx(final[2], abs=0.005)
    assert p[3] == pytest.approx(final[3], abs=0.06)
    t0, t1 = active_span(res.steppers["extruder"])
    # Acceleration at the start and end of the ring adds a little time
    assert total_t * 0.98 < t1 - t0 < total_t * 1.10, (t1 - t0, total_t)


def test_inverse_time_multi_ring():
    gcode, final, total_t = radial_gcode.generate(rings=4,
                                                  moves_per_ring=120)
    res = run_ok("THETA_SET_POSITION X=0 C=0 Z=20 B=0\n" + gcode + "M400\n")
    p = machine_pos(res, 1e9, (0.0, 0.0, 20.0, 0.0))
    for i, tol in enumerate((0.02, 0.02, 0.005, 0.06)):
        assert p[i] == pytest.approx(final[i], abs=tol)


def move_duration(res, gcode_prefix_moves=None):
    spans = [active_span(s) for s in res.steppers.values()
             if len(s.times) > 1]
    return max(t for _, t in spans) - min(t for t, _ in spans)


def test_g94_feed_uses_all_axes():
    # 4 axis feed rate: F applies to the length of (X, C, Z, B) with all
    # axes treated as linear.  Use low accel so the cruise dominates.
    pre = START + "SET_VELOCITY_LIMIT ACCEL=100000\n"
    res = run_ok(pre + "G1 B30 F600\nM400\n")          # 30 deg at 10/s
    assert move_duration(res) == pytest.approx(3.0, rel=0.03)
    res = run_ok(pre + "G1 X80 B30 F600\nM400\n")      # |(30,30)|=42.4
    assert move_duration(res) == pytest.approx(math.hypot(30, 30) / 10,
                                               rel=0.03)
    res = run_ok(pre + "G1 C90 X60 F1200\nM400\n")     # |(10,90)|=90.6
    assert move_duration(res) == pytest.approx(math.hypot(10, 90) / 20,
                                               rel=0.03)


def test_g93_duration():
    pre = START + "SET_VELOCITY_LIMIT ACCEL=100000\n"
    res = run_ok(pre + "G93\nG1 X80 C120 B-20 F30\nG94\nM400\n")
    assert move_duration(res) == pytest.approx(2.0, rel=0.03)
    res = run_ok(pre + "G93\nG1 B-20 F60\nG94\nM400\n")   # tilt only
    assert move_duration(res) == pytest.approx(1.0, rel=0.03)


def test_g93_requires_feed():
    res = sim.run(CFG, START + "G93\nG1 X60\nM400\n")
    assert "require an F value" in res.log


def test_g92_and_relative_moves():
    gcode = START + (
        "G1 C720 F20000\n"
        "G92 C0 B5\n"          # C=720 -> 0, B=0 -> 5
        "G1 C10 B15 F3000\n"   # +10 deg C, +10 deg B
        "G91\n"
        "G1 C-5 X-5 B-2\n"     # relative
        "G90\n"
        "M114\n"
        "THETA_STATUS\n"
        "M400\n"
    )
    res = run_ok(gcode)
    p = machine_pos(res, 1e9, (50.0, 0.0, 10.0, 0.0))
    assert p[1] == pytest.approx(725.0, abs=0.02)
    assert p[3] == pytest.approx(8.0, abs=0.06)
    assert p[0] == pytest.approx(45.0, abs=0.02)
    m = re.search(r"gcode: X=(\S+) C=(\S+) Z=(\S+) E=(\S+) B=(\S+)", res.log)
    assert m, res.log[-2000:]
    assert [float(v) for v in m.groups()] == pytest.approx(
        [45.0, 5.0, 10.0, 0.0, 13.0])
