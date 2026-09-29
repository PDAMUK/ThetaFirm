# Batch-mode tests of interoperability with other Kalico features and
# with G-code a slicer or front end commonly sends.
import math
import os
import re
import shutil

import pytest
import sim
from test_kinematics import CFG, START, machine_pos, motor_deltas, run_ok

POLAR_START = START + "THETA_MODE MODE=POLAR\n"

REPORT_MACRO = """
[gcode_macro REPORT]
gcode:
  {action_respond_info("target=%.1f pa=%.4f" % (
      printer.extruder.target, printer.extruder.pressure_advance))}
"""


def config_variant(tmp_path, extra="", replace=()):
    cfg_dir = os.path.dirname(CFG)
    shutil.copy(os.path.join(cfg_dir, "macros.cfg"), tmp_path)
    with open(CFG) as f:
        text = f.read()
    for old, new in replace:
        text = text.replace(old, new)
    path = str(tmp_path / "printer.cfg")
    with open(path, "w") as f:
        f.write(text + extra)
    return path


# Kalico's exclude_object observes 5 extrusion moves after an object is
# cancelled before it starts skipping moves, so print some first
WARMUP_4AXIS = "".join("G1 C%d E0.1 F3000\n" % (i,) for i in range(1, 7))
WARMUP_POLAR = "".join("G1 X50 Y%d E0.1 F3000\n" % (i,) for i in range(1, 7))


def test_exclude_object_4axis():
    gcode = START + (
        "EXCLUDE_OBJECT_DEFINE NAME=a\nEXCLUDE_OBJECT_DEFINE NAME=b\n"
        "M83\nEXCLUDE_OBJECT NAME=b\n"
        "EXCLUDE_OBJECT_START NAME=a\n" + WARMUP_4AXIS
        + "EXCLUDE_OBJECT_END NAME=a\n"
        "EXCLUDE_OBJECT_START NAME=b\nG1 X60 C90 B-20 E5 F600\n"
        "EXCLUDE_OBJECT_END NAME=b\n"
        "EXCLUDE_OBJECT_START NAME=a\nG1 X56 C20 E1 F600\n"
        "EXCLUDE_OBJECT_END NAME=a\nM400\n"
    )
    res = run_ok(gcode)
    p = machine_pos(res, 1e9, (50.0, 0.0, 10.0, 0.0))
    assert p[0] == pytest.approx(56.0, abs=0.02)
    assert p[1] == pytest.approx(20.0, abs=0.02)
    # B is modal: object b set B=-20 and object a's move does not set it,
    # so the toolhead follows the G-code state after the excluded object
    assert p[3] == pytest.approx(-20.0, abs=0.06)
    # Object b's extrusion was skipped
    assert motor_deltas(res)["extruder"] == pytest.approx(1.6, abs=0.002)
    # ...and the toolhead never went to object b (X60, C90)
    xs = [machine_pos(res, t, (50.0, 0.0, 10.0, 0.0))[0]
          for t in res.all_step_times()]
    assert max(xs) < 56.1


def test_exclude_object_polar():
    gcode = POLAR_START + (
        "EXCLUDE_OBJECT_DEFINE NAME=a\nEXCLUDE_OBJECT_DEFINE NAME=b\n"
        "M83\nEXCLUDE_OBJECT NAME=b\n"
        "EXCLUDE_OBJECT_START NAME=a\n" + WARMUP_POLAR
        + "EXCLUDE_OBJECT_END NAME=a\n"
        "EXCLUDE_OBJECT_START NAME=b\nG1 X0 Y40 E5 F3000\n"
        "EXCLUDE_OBJECT_END NAME=b\n"
        "EXCLUDE_OBJECT_START NAME=a\nG1 X30 Y30 E1 F3000\n"
        "EXCLUDE_OBJECT_END NAME=a\nM400\n"
    )
    res = run_ok(gcode)
    x, c, z, b = machine_pos(res, 1e9, (50.0, 0.0, 10.0, 0.0))
    assert x * math.cos(math.radians(c)) == pytest.approx(30.0, abs=0.03)
    assert x * math.sin(math.radians(c)) == pytest.approx(30.0, abs=0.03)
    assert motor_deltas(res)["extruder"] == pytest.approx(1.6, abs=0.002)


def test_pause_resume_restores_tilt_and_feed_mode():
    gcode = START + (
        "G1 B-10 F600\n"
        "G93\nG1 X52 F60\n"
        "SAVE_GCODE_STATE NAME=p\n"
        "G94\nG1 X30 C45 B-60 F3000\n"          # e.g. a parking move
        "RESTORE_GCODE_STATE NAME=p MOVE=1 MOVE_SPEED=50\n"
        "THETA_STATUS\nM400\n"
    )
    res = run_ok(gcode)
    p = machine_pos(res, 1e9, (50.0, 0.0, 10.0, 0.0))
    assert p[0] == pytest.approx(52.0, abs=0.02)
    assert p[1] == pytest.approx(0.0, abs=0.02)
    assert p[3] == pytest.approx(-10.0, abs=0.06)
    assert "feed mode: G93 inverse time" in res.log


def test_pause_resume_commands():
    gcode = START + (
        "G1 B-10 F600\n"
        "PAUSE\nG91\nG1 Z5 F600\nG90\nG1 X20 B-45 F3000\n"
        "RESUME\nTHETA_STATUS\nM400\n"
    )
    res = run_ok(gcode)
    p = machine_pos(res, 1e9, (50.0, 0.0, 10.0, 0.0))
    assert p[0] == pytest.approx(50.0, abs=0.02)
    assert p[2] == pytest.approx(10.0, abs=0.01)   # RESUME restores Z too
    assert p[3] == pytest.approx(-10.0, abs=0.06)


def test_arcs_polar_mode():
    # Quarter circle of radius 20 around (40, 0)
    gcode = POLAR_START + "G1 X60 Y0 F3000\nG3 X40 Y20 I-20 J0 F3000\nM400\n"
    res = run_ok(gcode)
    x, c, z, b = machine_pos(res, 1e9, (50.0, 0.0, 10.0, 0.0))
    assert x * math.cos(math.radians(c)) == pytest.approx(40.0, abs=0.03)
    assert x * math.sin(math.radians(c)) == pytest.approx(20.0, abs=0.03)
    # The path stays on the circle (0.1mm arc resolution)
    t_arc = [t for t in res.all_step_times()]
    worst = 0.0
    for t in t_arc:
        xm, cm, _, _ = machine_pos(res, t, (50.0, 0.0, 10.0, 0.0))
        px = xm * math.cos(math.radians(cm))
        py = xm * math.sin(math.radians(cm))
        if py > 0.05:
            worst = max(worst, abs(math.hypot(px - 40.0, py) - 20.0))
    assert worst < 0.05


def test_arcs_rejected_in_4axis_mode():
    res = sim.run(CFG, "THETA_SET_POSITION X=50 C=0 Z=10 B=0\n"
                  "G2 X40 C20 I-5 J0\nM400\n")
    assert "G2/G3 arcs require polar mode" in res.log


def test_rrf_flavour_commands(tmp_path):
    cfg = config_variant(tmp_path, REPORT_MACRO)
    res = sim.run(cfg, "T0\nG10 P0 S205 R150\nM116\nM572 D0 S0.045\n"
                  "REPORT\nG10\nM400\n")
    assert res.returncode == 0, res.log[-3000:]
    assert "Unknown command" not in res.log
    assert "target=205.0 pa=0.0450" in res.log
    assert "G10 retraction is not configured" in res.log


def test_force_move_for_commissioning(tmp_path):
    cfg = config_variant(tmp_path, "\n[force_move]\nenable_force_move: True\n")
    res = sim.run(cfg, "FORCE_MOVE STEPPER=stepper_x DISTANCE=4 VELOCITY=5\n"
                  "FORCE_MOVE STEPPER=stepper_c DISTANCE=10 VELOCITY=10\n"
                  "M400\n")
    assert res.returncode == 0, res.log[-3000:]
    d = motor_deltas(res)
    assert d["stepper_x"] == pytest.approx(4.0, abs=0.011)
    assert d["stepper_b"] == pytest.approx(0.0)
    assert d["stepper_c"] == pytest.approx(10.0, abs=0.012)


def test_probe_commands():
    res = run_ok("G28\nPROBE_DOWN\nPROBE\nQUERY_PROBE\nNOZZLE_DOWN\n"
                 "THETA_STATUS\nM400\n")
    assert re.search(r"probe at [-\d.]+,[-\d.]+ is z=", res.log)
