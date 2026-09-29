# Batch-mode tests of the core_rtheta kinematics (run with pytest).
# See sim.py for the required environment variables.
import os

import pytest
import sim

CFG = os.path.join(sim.REPO, "config", "printer.cfg")
K = 0.222222  # b_mm_per_degree from printer.cfg

# Common preamble: declare the machine position without homing
START = "THETA_SET_POSITION X=50 C=0 Z=10 B=0\n"


def run_ok(gcode):
    res = sim.run(CFG, gcode)
    assert res.returncode == 0, res.log[-3000:]
    return res


def motor_deltas(res):
    return {n: s.final_position() for n, s in res.steppers.items()}


def machine_pos(res, t, start):
    """Rebuild (X, C, Z, B) at time t from the motor step counts."""
    st = res.steppers
    m1 = st["stepper_x"].position_at(t)
    m2 = st["stepper_b"].position_at(t)
    x = start[0] + 0.5 * (m1 + m2)
    b = start[3] + 0.5 * (m2 - m1) / K
    c = start[1] + st["stepper_c"].position_at(t)
    z = start[2] + st["stepper_z"].position_at(t)
    return x, c, z, b


def test_single_axis_moves():
    res = run_ok(START + "G1 X60 F600\nM400\n")
    d = motor_deltas(res)
    assert d["stepper_x"] == pytest.approx(10.0, abs=0.011)
    assert d["stepper_b"] == pytest.approx(10.0, abs=0.011)
    assert d["stepper_c"] == pytest.approx(0.0)
    res = run_ok(START + "G1 B10 F600\nM400\n")
    d = motor_deltas(res)
    assert d["stepper_x"] == pytest.approx(-10 * K, abs=0.011)
    assert d["stepper_b"] == pytest.approx(10 * K, abs=0.011)
    res = run_ok(START + "G1 C90 F6000\nG1 Z20 F600\nM400\n")
    d = motor_deltas(res)
    assert d["stepper_c"] == pytest.approx(90.0, abs=0.012)
    assert d["stepper_z"] == pytest.approx(10.0, abs=0.003)
    assert d["stepper_x"] == pytest.approx(0.0)
    assert d["stepper_b"] == pytest.approx(0.0)


def check_synchronized(res, start, end, t0=0.0, t1=None):
    """All axes must progress in proportion during a single move."""
    tol = {0: 0.011, 1: 0.012, 2: 0.003, 3: 0.011 / K}
    times = [t for t in res.all_step_times() if t >= t0
             and (t1 is None or t <= t1)]
    ref = max(range(4), key=lambda i: abs(end[i] - start[i]) / tol[i])
    worst = 0.0
    for t in times[:: max(1, len(times) // 2000)]:
        p = machine_pos(res, t, start)
        f = (p[ref] - start[ref]) / (end[ref] - start[ref])
        for i in range(4):
            if i == ref:
                continue
            expect = start[i] + f * (end[i] - start[i])
            err = abs(p[i] - expect) - tol[i] - abs(
                (end[i] - start[i]) * tol[ref] / (end[ref] - start[ref]))
            worst = max(worst, err)
    assert worst <= 1e-9, "axes out of sync by %.5f" % worst


def test_combined_move_synchronized():
    start = (50.0, 0.0, 10.0, 0.0)
    end = (80.0, 200.0, 30.0, -40.0)
    res = run_ok(START + "G1 X80 C200 Z30 B-40 F3000\nM400\n")
    p = machine_pos(res, 1e9, start)
    for i in range(4):
        assert p[i] == pytest.approx(end[i], abs=0.05)
    check_synchronized(res, start, end)


def test_rotate_extrude_and_tilt_only_moves():
    gcode = START + (
        "M83\n"
        "G1 C45 E1.0 F3000\n"   # rotate + extrude (kinematic)
        "G1 B-30 F1200\n"       # tilt only (extra axis only)
        "G1 C90 E1.0 F3000\n"
        "G1 B-20 E0.5 F600\n"   # tilt + extrude
        "G1 X40 B0 F1200\n"     # radius + tilt
        "M400\n"
    )
    res = run_ok(gcode)
    p = machine_pos(res, 1e9, (50.0, 0.0, 10.0, 0.0))
    assert p[0] == pytest.approx(40.0, abs=0.02)
    assert p[1] == pytest.approx(90.0, abs=0.02)
    assert p[3] == pytest.approx(0.0, abs=0.06)
    assert motor_deltas(res)["extruder"] == pytest.approx(2.5, abs=0.002)


def test_tilt_requires_homing():
    res = sim.run(CFG, "THETA_SET_POSITION X=50 C=0 Z=10\nG1 B10\nM400\n")
    assert "Must home axis B first" in res.log


def test_y_rejected_in_4axis_mode():
    res = sim.run(CFG, START + "G1 Y10\nM400\n")
    assert "Y is not an axis in 4 axis mode" in res.log


@pytest.mark.parametrize("start_c", [0.0, 1.0e6, 1.0e8])
def test_unlimited_bed_rotation(start_c):
    # C never wraps: whole turns in both directions stay exact even at
    # very large accumulated angles
    res = run_ok("THETA_SET_POSITION X=50 C=%r Z=10 B=0\n"
                 "G1 C%r F20000\nG1 C%r F20000\nM400\n"
                 % (start_c, start_c + 3600.0, start_c - 1800.0))
    st = res.steppers["stepper_c"]
    runs = []
    for a, b in zip(st.steps, st.steps[1:]):
        if runs and (runs[-1] > 0) == (b - a > 0):
            runs[-1] += b - a
        else:
            runs.append(b - a)
    assert [r * st.step_dist for r in runs] == pytest.approx(
        [3600.0, -5400.0], abs=1e-6)
