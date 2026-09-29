# Batch-mode simulation helpers for testing the core_rtheta plugin.
#
# Runs Kalico's klippy in file-output mode (no printer needed), decodes
# the generated MCU command stream and rebuilds each stepper's position
# as a function of time.
#
# Environment:
#   KALICO_DIR   path to a Kalico checkout (default ~/kalico)
#   KALICO_DICT  path to an stm32f407 klipper.dict built from that checkout
#   KALICO_PY    python interpreter with Kalico's dependencies
import bisect
import configparser
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PLUGIN_SRC = os.path.join(REPO, "plugins", "core_rtheta.py")


def kalico_dir():
    return os.path.abspath(
        os.environ.get("KALICO_DIR", os.path.expanduser("~/kalico"))
    )


def install_plugin():
    dest = os.path.join(kalico_dir(), "klippy", "plugins", "core_rtheta.py")
    if os.path.islink(dest) or os.path.exists(dest):
        if os.path.realpath(dest) == os.path.realpath(PLUGIN_SRC):
            return
        os.unlink(dest)
    os.symlink(PLUGIN_SRC, dest)


class StepperTrace:
    def __init__(self, name, oid, invert_dir, step_dist):
        self.name = name
        self.oid = oid
        self.invert_dir = invert_dir
        self.step_dist = step_dist
        self.times = [0.0]
        self.steps = [0]
        self.clock = 0
        self.dir = 1
        self.pos = 0

    def queue_step(self, interval, count, add, freq):
        clock = self.clock
        d = self.dir
        for i in range(count):
            clock += interval
            interval += add
            self.pos += d
            self.times.append(clock / freq)
            self.steps.append(self.pos)
        self.clock = clock

    def step_count_at(self, t):
        i = bisect.bisect_right(self.times, t) - 1
        return self.steps[max(i, 0)]

    def position_at(self, t):
        # Position in stepper units (relative to power on)
        return self.step_count_at(t) * self.step_dist

    def final_position(self):
        return self.steps[-1] * self.step_dist


class SimResult:
    def __init__(self, returncode, log, steppers, freq, output_lines):
        self.returncode = returncode
        self.log = log
        self.steppers = steppers
        self.freq = freq
        self.output_lines = output_lines

    def responses(self):
        # Lines klippy wrote in response to commands (from the log)
        return [l for l in self.log.splitlines()]

    def all_step_times(self):
        times = set()
        for s in self.steppers.values():
            times.update(s.times)
        return sorted(times)


def parse_steppers_from_config(cfg_path):
    """Return {step_pin: (section, dir_inverted, step_dist)}"""
    cp = configparser.RawConfigParser(inline_comment_prefixes=("#", ";"),
                                      strict=False)
    cp.read(cfg_path)
    res = {}
    for sect in cp.sections():
        if not cp.has_option(sect, "step_pin"):
            continue
        step_pin = cp.get(sect, "step_pin").strip()
        dir_pin = cp.get(sect, "dir_pin").strip()
        inv = dir_pin.startswith("!")
        rd = float(cp.get(sect, "rotation_distance"))
        micro = int(cp.get(sect, "microsteps"))
        fsteps = 200
        if cp.has_option(sect, "full_steps_per_rotation"):
            fsteps = int(cp.get(sect, "full_steps_per_rotation"))
        res[step_pin.lstrip("!^~")] = (sect, inv, rd / (fsteps * micro))
    return res


def decode_output(dict_path, output_path, pin_map):
    kd = kalico_dir()
    if kd not in sys.path:
        sys.path.insert(0, kd)
    from klippy import msgproto

    with open(dict_path, "rb") as f:
        dictionary = f.read()
    mp = msgproto.MessageParser()
    mp.process_identify(dictionary, decompress=False)
    freq = float(mp.get_constant("CLOCK_FREQ"))
    with open(output_path, "rb") as f:
        data = bytearray(f.read())
    lines = []
    while data:
        l = mp.check_packet(data)
        if l <= 0:
            break
        lines.extend(mp.dump(data[:l])[1:])
        data = data[l:]
    steppers = {}
    by_oid = {}
    re_kv = re.compile(r"(\w+)=(\S+)")
    for line in lines:
        name = line.split(" ", 1)[0]
        if name not in ("config_stepper", "queue_step", "set_next_step_dir",
                        "reset_step_clock"):
            continue
        kv = dict(re_kv.findall(line))
        oid = int(kv["oid"])
        if name == "config_stepper":
            pin = kv["step_pin"]
            sect, inv, step_dist = pin_map[pin]
            st = StepperTrace(sect, oid, inv, step_dist)
            by_oid[oid] = st
            steppers[sect] = st
        elif oid not in by_oid:
            continue
        elif name == "queue_step":
            by_oid[oid].queue_step(int(kv["interval"]), int(kv["count"]),
                                   int(kv["add"]), freq)
        elif name == "set_next_step_dir":
            st = by_oid[oid]
            d = int(kv["dir"])
            # The host inverts the dir bit for inverted dir pins
            if st.invert_dir:
                d = 1 - d
            st.dir = 1 if d else -1
        elif name == "reset_step_clock":
            by_oid[oid].clock = int(kv["clock"])
    return steppers, freq, lines


def run(cfg_path, gcode, dict_path=None, python=None, extra_files=None,
        timeout=600):
    """Run klippy in batch mode on cfg_path with the given gcode text."""
    install_plugin()
    dict_path = dict_path or os.environ.get("KALICO_DICT")
    if not dict_path:
        raise RuntimeError("KALICO_DICT not set")
    python = python or os.environ.get("KALICO_PY", sys.executable)
    tmpdir = tempfile.mkdtemp(prefix="rtheta_sim_")
    cfg_dir = os.path.dirname(os.path.abspath(cfg_path))
    # Kalico's file output mode stops its serial writer thread at exit
    # without draining it, so the last commands can be lost.  Generate all
    # remaining steps (dwell) and then wait in real time before the input
    # ends so everything reaches the output file.
    run_cfg = os.path.join(tmpdir, "sim_printer.cfg")
    with open(run_cfg, "w") as f:
        f.write("[include %s]\n\n" % (os.path.abspath(cfg_path),)
                + "[gcode_shell_command rtheta_sim_drain]\n"
                "command: sleep 0.3\ntimeout: 10\nverbose: False\n")
    gcode_path = os.path.join(tmpdir, "test.gcode")
    with open(gcode_path, "w") as f:
        f.write(gcode + "\nM400\nG4 P500\n"
                "RUN_SHELL_COMMAND CMD=rtheta_sim_drain\n")
    out_path = os.path.join(tmpdir, "out.serial")
    log_path = os.path.join(tmpdir, "klippy.log")
    env = dict(os.environ)
    env["PYTHONPATH"] = kalico_dir()
    args = [python, "-m", "klippy", run_cfg, "-i",
            gcode_path, "-o", out_path, "-v", "-d", dict_path, "-l",
            log_path]
    proc = subprocess.run(args, cwd=cfg_dir, env=env, timeout=timeout,
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log = ""
    if os.path.exists(log_path):
        with open(log_path) as f:
            log = f.read()
    log += proc.stdout.decode(errors="replace")
    steppers, freq, lines = {}, 1.0, []
    if os.path.exists(out_path):
        pin_map = parse_steppers_from_config(cfg_path)
        steppers, freq, lines = decode_output(dict_path, out_path, pin_map)
    return SimResult(proc.returncode, log, steppers, freq, lines)
