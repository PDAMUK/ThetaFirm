# Batch-mode tests of core_rtheta polar (Cartesian G-code) mode.
import math

import pytest
import sim
from test_kinematics import CFG, machine_pos, motor_deltas, run_ok

START = "THETA_SET_POSITION X=50 C=0 Z=10 B=0\nTHETA_MODE MODE=POLAR\n"
M0 = (50.0, 0.0, 10.0, 0.0)

# Step resolution: X 0.01mm, C 0.01125deg, B via both core motors
STEP_TOL = 0.035


def cart_at(res, t):
    x, c, z, b = machine_pos(res, t, M0)
    return x * math.cos(math.radians(c)), x * math.sin(math.radians(c))


def seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    l2 = ax * ax + ay * ay
    t = 0.0 if not l2 else max(0.0, min(1.0, ((p[0] - a[0]) * ax
                                              + (p[1] - a[1]) * ay) / l2))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def max_path_error(res, a, b):
    times = res.all_step_times()
    return max(seg_dist(cart_at(res, t), a, b) for t in times)


def end_cart(res):
    return cart_at(res, 1e9)


def test_cartesian_line():
    res = run_ok(START + "G1 X50 Y50 F3000\nM400\n")
    x, c, z, b = machine_pos(res, 1e9, M0)
    assert x == pytest.approx(math.hypot(50, 50), abs=0.02)
    assert c == pytest.approx(45.0, abs=0.02)
    assert max_path_error(res, (50, 0), (50, 50)) < 0.01 + STEP_TOL


def test_line_through_centre():
    res = run_ok(START + "G1 X-50 Y0 F3000\nM400\n")
    x, c, z, b = machine_pos(res, 1e9, M0)
    assert x == pytest.approx(50.0, abs=0.02)
    assert abs(abs(c) - 180.0) < 0.02
    assert max_path_error(res, (50, 0), (-50, 0)) < 0.01 + STEP_TOL


def test_line_near_centre():
    gcode = ("THETA_SET_POSITION X=50.04 C=2.2906 Z=10 B=0\n"
             "THETA_MODE MODE=POLAR\nG1 X-50 Y2 F3000\nM400\n")
    res = run_ok(gcode)
    start = (50.04, 2.2906, 10.0, 0.0)
    x, c, z, b = machine_pos(res, 1e9, start)
    ex = x * math.cos(math.radians(c))
    ey = x * math.sin(math.radians(c))
    assert (ex, ey) == pytest.approx((-50.0, 2.0), abs=0.03)

    def cart(t):
        p = machine_pos(res, t, start)
        return (p[0] * math.cos(math.radians(p[1])),
                p[0] * math.sin(math.radians(p[1])))
    a = (50.04 * math.cos(math.radians(2.2906)),
         50.04 * math.sin(math.radians(2.2906)))
    err = max(seg_dist(cart(t), a, (-50, 2)) for t in res.all_step_times())
    assert err < 0.01 + STEP_TOL


def test_cartesian_feed_rate():
    # Far from the centre the bed turns slowly, so a Cartesian feed
    # rate of 50mm/s moves 20mm in ~0.4s
    gcode = ("THETA_SET_POSITION X=60 C=0 Z=10 B=0\nTHETA_MODE MODE=POLAR\n"
             "SET_VELOCITY_LIMIT ACCEL=20000\nG1 X60 Y20 F3000\nM400\n")
    res = run_ok(gcode)
    spans = [(s.times[1], s.times[-1]) for s in res.steppers.values()
             if len(s.times) > 1]
    duration = max(t for _, t in spans) - min(t for t, _ in spans)
    assert duration == pytest.approx(0.4, rel=0.05)


def test_tilt_and_extrusion_in_polar_mode():
    res = run_ok(START + "M83\nG1 X40 Y30 B-20 E2 F3000\nM400\n")
    x, c, z, b = machine_pos(res, 1e9, M0)
    assert b == pytest.approx(-20.0, abs=0.06)
    assert x == pytest.approx(50.0, abs=0.02)
    assert c == pytest.approx(math.degrees(math.atan2(30, 40)), abs=0.02)
    assert motor_deltas(res)["extruder"] == pytest.approx(2.0, abs=0.002)


def test_mode_switch_round_trip():
    gcode = START + (
        "G1 X0 Y60 F3000\n"        # machine X=60, C=90
        "THETA_MODE MODE=4AXIS\n"
        "G1 C180 F6000\n"          # rotate bed; now at Cartesian (-60, 0)
        "THETA_MODE MODE=POLAR\n"
        "G1 X-60 Y10 F3000\n"
        "THETA_STATUS\nM400\n"
    )
    res = run_ok(gcode)
    x, c, z, b = machine_pos(res, 1e9, M0)
    assert x * math.cos(math.radians(c)) == pytest.approx(-60.0, abs=0.03)
    assert x * math.sin(math.radians(c)) == pytest.approx(10.0, abs=0.03)
    assert "gcode: X=-60.0000 Y=10.0000" in res.log


def test_polar_requires_positive_x():
    res = sim.run(CFG, "THETA_SET_POSITION X=-10 C=0 Z=10 B=0\n"
                  "THETA_MODE MODE=POLAR\nM400\n")
    assert "Polar mode requires X >= 0" in res.log


def test_c_rejected_in_polar_mode():
    res = sim.run(CFG, START + "G1 C10\nM400\n")
    assert "C is not available in polar mode" in res.log


def test_polar_homing_homes_radius_and_angle():
    res = sim.run(CFG, "THETA_MODE MODE=POLAR\nG28 Y\nTHETA_STATUS\nM400\n")
    assert "homed: xc" in res.log
