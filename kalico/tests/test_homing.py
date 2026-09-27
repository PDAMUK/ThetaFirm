# Batch-mode tests of core_rtheta homing (run with pytest).
#
# In batch mode every endstop "triggers" at the end of its homing move,
# so the motor totals below follow from the configured homing geometry:
#   B: forced to 90-1.5*270=-315, homes to 90, retracts 10 (sensorless:
#      single pass)
#   X: forced to 115.5-1.5*153=-114, homes to 115.5, retracts 10
#   Z: hops 10, probe down (B=-90), X to 0, probes twice, lifts to 10
#      and turns the nozzle down (B=0)
import re

import pytest
import sim
from test_kinematics import CFG, K, motor_deltas


def status(res):
    m = re.findall(r"toolhead: X=(\S+) C=(\S+) Z=(\S+) B=(\S+)", res.log)
    h = re.findall(r"homed: (\S*)", res.log)
    return [float(v) for v in m[-1]], h[-1] if h else ""


def run(gcode):
    return sim.run(CFG, gcode + "\nTHETA_STATUS\nM400\n")


def test_home_all():
    res = run("G28")
    assert res.returncode == 0, res.log[-3000:]
    pos, homed = status(res)
    assert pos == pytest.approx([0.0, 0.0, 10.0, 0.0])
    assert homed == "xczb"
    assert "gcode: X=0.0000 C=0.0000 Z=10.0000 E=0.0000 B=0.0000" in res.log
    d = motor_deltas(res)
    b_moves = 405 - 10 - 170 + 90        # tilt: home, retract, probe, nozzle
    x_moves = 229.5 - 10 - 105.5         # radius: home, retract, to X0
    assert d["stepper_x"] == pytest.approx(x_moves - K * b_moves, abs=0.02)
    assert d["stepper_b"] == pytest.approx(x_moves + K * b_moves, abs=0.02)
    assert d["stepper_z"] == pytest.approx(10 - 312.75 + 3 - 6 + 3 + 15.5,
                                           abs=0.01)
    assert d["stepper_c"] == pytest.approx(0.0)


def test_home_b_only():
    res = run("G28 B")
    pos, homed = status(res)
    assert homed == "b"
    assert pos[3] == pytest.approx(80.0)
    d = motor_deltas(res)
    assert d["stepper_x"] == pytest.approx(-K * (405 - 10), abs=0.02)
    assert d["stepper_b"] == pytest.approx(K * (405 - 10), abs=0.02)


def test_home_x_only():
    res = run("G28 X")
    pos, homed = status(res)
    assert homed == "x"
    assert pos[0] == pytest.approx(105.5)
    d = motor_deltas(res)
    assert d["stepper_x"] == pytest.approx(219.5, abs=0.02)
    assert d["stepper_b"] == pytest.approx(219.5, abs=0.02)


def test_home_c_declares_zero():
    res = run("G28 X B\nG28 C\nG1 C400 F20000\nG28 C")
    pos, homed = status(res)
    assert homed == "xcb"
    assert pos[1] == pytest.approx(0.0)
    assert motor_deltas(res)["stepper_c"] == pytest.approx(400.0, abs=0.012)


def test_home_z_requires_x_and_b():
    res = run("G28 Z")
    assert "Axes X and B must be homed before Z" in res.log


def test_moves_require_homing():
    res = run("G1 X10")
    assert "Must home axis X first" in res.log
    res = run("G28 X B\nG1 C10")
    assert "Must home axis C first" in res.log


def test_motor_off_clears_homing():
    res = run("G28\nM84")
    pos, homed = status(res)
    assert homed == ""


def test_home_b_with_endstop_switch(tmp_path):
    # Physical B end stop: two homing passes with a retract in between
    import shutil
    cfg_dir = sim.os.path.dirname(CFG)
    shutil.copy(sim.os.path.join(cfg_dir, "macros.cfg"), tmp_path)
    with open(CFG) as f:
        text = f.read()
    text = text.replace("endstop_pin: tmc2209_stepper_b:virtual_endstop",
                        "endstop_pin: ^PD0")
    cfg = str(tmp_path / "printer.cfg")
    with open(cfg, "w") as f:
        f.write(text)
    res = sim.run(cfg, "G28 B\nTHETA_STATUS\nM400\n")
    assert res.returncode == 0, res.log[-3000:]
    pos, homed = status(res)
    assert homed == "b"
    assert pos[3] == pytest.approx(80.0)
    # home +405, retract -10, second pass +10, final retract -10
    d = motor_deltas(res)
    assert d["stepper_x"] == pytest.approx(-K * 395, abs=0.02)
    assert d["stepper_b"] == pytest.approx(K * 395, abs=0.02)
    tr = res.steppers["stepper_b"]
    runs = []
    for a, b in zip(tr.steps, tr.steps[1:]):
        step = b - a
        if runs and (runs[-1] > 0) == (step > 0):
            runs[-1] += step
        else:
            runs.append(step)
    runs_deg = [r * tr.step_dist / K for r in runs]
    assert runs_deg == pytest.approx([405, -10, 10, -10], abs=0.1)
