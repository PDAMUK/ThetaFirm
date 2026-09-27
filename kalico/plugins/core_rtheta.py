# Kalico plugin: kinematics and G-code support for the ThetaFirm
# "Core R-Theta" 4 axis polar printer
#
# Copyright (C) 2026  ThetaFirm contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Machine summary
# ---------------
# * C (driver "stepper_c") rotates the bed.  Units: degrees, unlimited.
# * X is the radial position of the toolhead carriage.  Units: mm.
# * B tilts the nozzle.  Units: degrees.  B=0 is nozzle pointing down.
# * Z moves the gantry vertically.  Units: mm.
#
# X and B are driven by a pair of motors in a "core" arrangement:
#
#     stepper_x = X - k*B        stepper_b = X + k*B
#
# where k (b_mm_per_degree) is the length of belt moved per degree of
# tilt.  This is the same coupling as the RepRapFirmware configuration
# (M669 K0 ... X-1:0:0:1:0 B0.2222:0:0:0.2222:0) with driver 1 using its
# "polar mode" direction.
#
# Implementation summary
# ----------------------
# Kalico's toolhead plans moves for three kinematic coordinates plus any
# number of "extra axes".  In 4 axis mode the toolhead coordinates are
# (X, C, Z) and B is registered as an extra axis, so bed rotation is a
# first class kinematic axis (junction speeds, pressure advance and
# per-axis limits all apply to rotate+extrude moves).  Because the core
# motors need both X and B, they are fed from a private trapq holding
# (X, k*B) that is populated from the toolhead's moves.  The motors then
# use the standard corexy step solvers (x-y and x+y).
import logging
import math
import sys

from klippy import chelper, stepper
from klippy.extras import force_move
from klippy.extras import homing as homing_mod

KINEMATICS_NAME = "core_rtheta"

# Allow "kinematics: core_rtheta" in the [printer] section.  Kalico only
# looks for kinematics in klippy/kinematics/, so make this plugin module
# importable under that name.  The [core_rtheta] config section must be
# present so this module is imported before the toolhead is created.
sys.modules.setdefault(
    "klippy.kinematics." + KINEMATICS_NAME, sys.modules[__name__]
)

MODE_4AXIS = "4axis"
MODE_POLAR = "polar"

NEVER = 9999999999999999.0


######################################################################
# Private motion queue for the two core motors
######################################################################


class CoreMotionQueue:
    """Trapq holding (X, k*B) for the X/B core motors.

    Kinematic toolhead moves are captured when the toolhead appends them
    to its own trapq (X comes from there) and tilt moves arrive through
    the tilt axis' process_move() callback.  Both happen, in order, from
    the same loop in ToolHead._process_lookahead(), so a kinematic move is
    held as "pending" until it is known whether B moves with it."""

    def __init__(self, b_scale):
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self._trapq_append = ffi_lib.trapq_append
        self._trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self._trapq_set_position = ffi_lib.trapq_set_position
        self.b_scale = b_scale
        self.enabled = True
        self.pending = None
        # Tilt position (degrees) at the end of the queued motion
        self.last_b = 0.0

    def append(self, print_time, accel_t, cruise_t, decel_t, start_x,
               start_y, axis_r_x, axis_r_y, start_v, cruise_v, accel):
        self._trapq_append(
            self.trapq, print_time, accel_t, cruise_t, decel_t,
            start_x, start_y, 0.0, axis_r_x, axis_r_y, 0.0,
            start_v, cruise_v, accel,
        )

    def note_kinematic_move(self, print_time, accel_t, cruise_t, decel_t,
                            start_x, axis_r_x, start_v, cruise_v, accel):
        if not self.enabled:
            return
        self.commit()
        self.pending = (print_time, accel_t, cruise_t, decel_t, start_x,
                        axis_r_x, start_v, cruise_v, accel)

    def note_tilt_move(self, print_time, move, ea_index):
        start_b = move.start_pos[ea_index]
        axis_r_b = move.axes_r[ea_index]
        k = self.b_scale
        p = self.pending
        if p is not None and p[0] == print_time:
            # Tilt moves together with a kinematic move - merge them
            self.pending = None
            self.append(p[0], p[1], p[2], p[3], p[4], k * start_b,
                        p[5], k * axis_r_b, p[6], p[7], p[8])
        else:
            # Tilt only move (not queued on the toolhead trapq)
            self.commit()
            self.append(print_time, move.accel_t, move.cruise_t,
                        move.decel_t, move.start_pos[0], k * start_b,
                        0.0, k * axis_r_b, move.start_v, move.cruise_v,
                        move.accel)
        self.last_b = move.end_pos[ea_index]

    def commit(self, flush_time=None):
        # Queue a pending kinematic move (with a stationary tilt)
        p = self.pending
        if p is None:
            return
        self.pending = None
        self.append(p[0], p[1], p[2], p[3], p[4], self.b_scale * self.last_b,
                    p[5], 0.0, p[6], p[7], p[8])

    def discard_all(self):
        self.pending = None
        self._trapq_finalize_moves(self.trapq, NEVER, 0.0)

    def set_position(self, print_time, x, b):
        self.pending = None
        self.last_b = b
        self._trapq_set_position(self.trapq, print_time, x,
                                 self.b_scale * b, 0.0)


######################################################################
# Tilt (B) axis - registered with the toolhead as an extra axis
######################################################################


class TiltAxis:
    def __init__(self, kin, rcfg, max_velocity, max_accel):
        self.kin = kin
        self.printer = kin.printer
        self.max_velocity = rcfg.getfloat(
            "max_b_velocity", max_velocity, above=0.0
        )
        self.max_accel = rcfg.getfloat("max_b_accel", max_accel, above=0.0)
        self.instant_corner_v = rcfg.getfloat(
            "b_instantaneous_corner_velocity", 1.0, minval=0.0
        )
        self.limits = (1.0, -1.0)

    def is_homed(self):
        return self.limits[0] <= self.limits[1]

    # Extra axis interface
    def get_axis_gcode_id(self):
        return "B"

    def get_name(self):
        return "tilt"

    def get_trapq(self):
        return self.kin.core_queue.trapq

    def check_move(self, move, ea_index):
        pos = move.end_pos[ea_index]
        if pos < self.limits[0] or pos > self.limits[1]:
            if not self.is_homed():
                raise self.printer.command_error("Must home axis B first")
            raise self.printer.command_error(
                "Move out of range: B=%.3f (limits %.3f to %.3f)"
                % (pos, self.limits[0], self.limits[1])
            )
        ratio = move.move_d / abs(move.axes_d[ea_index])
        move.limit_speed(self.max_velocity * ratio, self.max_accel * ratio)

    def calc_junction(self, prev_move, move, ea_index):
        diff_r = move.axes_r[ea_index] - prev_move.axes_r[ea_index]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r)) ** 2
        return move.max_cruise_v2

    def process_move(self, print_time, move, ea_index):
        self.kin.core_queue.note_tilt_move(print_time, move, ea_index)


######################################################################
# Toolhead-like helper used to home the tilt axis
######################################################################


class TiltHomingHelper:
    """Implements the subset of the toolhead interface used by
    homing.HomingMove.  Positions are in core queue coordinates
    [X, k*B, 0] so that step rate estimates are correct."""

    def __init__(self, kin, accel):
        self.kin = kin
        self.toolhead = kin.toolhead
        self.accel = accel

    def get_position(self):
        x, y = self.kin.get_core_position()
        return [x, y, 0.0, 0.0]

    def set_position(self, newpos, homing_axes=""):
        self.kin.set_core_position(newpos[0], newpos[1])

    def flush_step_generation(self):
        self.toolhead.flush_step_generation()

    def get_last_move_time(self):
        return self.toolhead.get_last_move_time()

    def dwell(self, delay):
        self.toolhead.dwell(delay)

    def drip_move(self, newpos, speed, drip_completion):
        toolhead = self.toolhead
        toolhead.dwell(toolhead.kin_flush_delay)
        print_time = toolhead.get_last_move_time()
        cq = self.kin.core_queue
        cq.commit()
        start_x, start_y = self.kin.get_core_position()
        axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(
            newpos[1] - start_y, speed, self.accel
        )
        cq.append(print_time, accel_t, cruise_t, accel_t, start_x, start_y,
                  0.0, axis_r, 0.0, cruise_v, self.accel)
        end_time = print_time + accel_t + cruise_t + accel_t
        toolhead.drip_update_time(end_time, drip_completion)
        cq.discard_all()

    def get_kinematics(self):
        return self

    def get_steppers(self):
        return self.kin.get_core_steppers()

    def calc_position(self, stepper_positions):
        m1 = stepper_positions[self.kin.rail_x.get_name()]
        m2 = stepper_positions[self.kin.rail_b.get_name()]
        return [0.5 * (m1 + m2), 0.5 * (m2 - m1), 0.0]


######################################################################
# Kinematics
######################################################################


class CoreRThetaKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead = toolhead
        self.reactor = self.printer.get_reactor()
        rcfg = config.getsection(KINEMATICS_NAME)
        self.b_scale = rcfg.getfloat("b_mm_per_degree")
        if not self.b_scale:
            raise config.error("b_mm_per_degree must be non-zero")
        self.mode = MODE_4AXIS
        # Steppers
        self.rail_x = stepper.LookupMultiRail(config.getsection("stepper_x"))
        self.rail_b = stepper.LookupMultiRail(config.getsection("stepper_b"))
        self.rail_z = stepper.LookupMultiRail(config.getsection("stepper_z"))
        self.stepper_c = stepper.PrinterStepper(config.getsection("stepper_c"))
        # Both core motors move for any X or B motion, so both must be
        # halted by either endstop
        for s in self.rail_b.get_steppers():
            self.rail_x.get_endstops()[0][0].add_stepper(s)
        for s in self.rail_x.get_steppers():
            self.rail_b.get_endstops()[0][0].add_stepper(s)
        self.rails = [self.rail_x, self.rail_b, self.rail_z]
        # Motion queue for the core motors
        self.core_queue = CoreMotionQueue(self.b_scale)
        self.main_trapq = toolhead.get_trapq()
        self.rail_x.setup_itersolve("corexy_stepper_alloc", b"-")
        self.rail_b.setup_itersolve("corexy_stepper_alloc", b"+")
        self.rail_x.set_trapq(self.core_queue.trapq)
        self.rail_b.set_trapq(self.core_queue.trapq)
        self.stepper_c.setup_itersolve("cartesian_stepper_alloc", b"y")
        self.stepper_c.set_trapq(self.main_trapq)
        self.rail_z.setup_itersolve("cartesian_stepper_alloc", b"z")
        self.rail_z.set_trapq(self.main_trapq)
        # The pending core move must be committed before the core motors
        # generate steps, so this must be the first step generator.
        toolhead.register_step_generator(self.core_queue.commit)
        for s in self.get_steppers():
            toolhead.register_step_generator(s.generate_steps)
        # Capture the toolhead's kinematic moves
        self._orig_trapq_append = toolhead.trapq_append
        self._orig_trapq_finalize_moves = toolhead.trapq_finalize_moves
        toolhead.trapq_append = self._trapq_append
        toolhead.trapq_finalize_moves = self._trapq_finalize_moves
        # Velocity and acceleration limits (per axis)
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_x_velocity = rcfg.getfloat(
            "max_x_velocity", max_velocity, above=0.0
        )
        self.max_x_accel = rcfg.getfloat("max_x_accel", max_accel, above=0.0)
        # C (bed rotation) limits.  Named "y" so SET_VELOCITY_LIMIT
        # Y_VELOCITY/Y_ACCEL adjust them.
        self.max_y_velocity = rcfg.getfloat(
            "max_c_velocity", max_velocity, above=0.0
        )
        self.max_y_accel = rcfg.getfloat("max_c_accel", max_accel, above=0.0)
        self.max_z_velocity = rcfg.getfloat(
            "max_z_velocity", max_velocity, above=0.0
        )
        self.max_z_accel = rcfg.getfloat("max_z_accel", max_accel, above=0.0)
        self.tilt = TiltAxis(self, rcfg, max_velocity, max_accel)
        # Position limits
        self.c_range = (
            rcfg.getfloat("c_position_min", -1.0e9),
            rcfg.getfloat("c_position_max", 1.0e9),
        )
        self.limits = [(1.0, -1.0)] * 3
        x_range, z_range = self.rail_x.get_range(), self.rail_z.get_range()
        self.axes_min = toolhead.Coord(
            x_range[0], self.c_range[0], z_range[0], e=0.0
        )
        self.axes_max = toolhead.Coord(
            x_range[1], self.c_range[1], z_range[1], e=0.0
        )
        self.supports_dual_carriage = False
        # Homing
        self.probe_b = rcfg.getfloat("probe_b_angle", -90.0)
        self.nozzle_b = rcfg.getfloat("nozzle_b_angle", 0.0)
        self.z_home_x = rcfg.getfloat("z_home_x", 0.0)
        self.z_hop = rcfg.getfloat("z_hop", 10.0, minval=0.0)
        self.z_hop_speed = rcfg.getfloat("z_hop_speed", 15.0, above=0.0)
        self.z_home_travel_speed = rcfg.getfloat(
            "z_home_travel_speed", 50.0, above=0.0
        )
        self.b_travel_speed = rcfg.getfloat(
            "b_travel_speed", self.tilt.max_velocity, above=0.0
        )
        self.restore_nozzle_after_z_home = rcfg.getboolean(
            "restore_nozzle_after_z_home", True
        )
        self.home_all_includes_b = rcfg.getboolean(
            "home_all_includes_b", True
        )
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )

    def _handle_connect(self):
        # Register the tilt axis ("B") with the toolhead
        self.toolhead.add_extra_axis(self.tilt, 0.0)

    # Hooks on the toolhead trapq
    def _trapq_append(self, trapq, print_time, accel_t, cruise_t, decel_t,
                      start_pos_x, start_pos_y, start_pos_z, axes_r_x,
                      axes_r_y, axes_r_z, start_v, cruise_v, accel):
        self._orig_trapq_append(
            trapq, print_time, accel_t, cruise_t, decel_t, start_pos_x,
            start_pos_y, start_pos_z, axes_r_x, axes_r_y, axes_r_z,
            start_v, cruise_v, accel,
        )
        if trapq == self.main_trapq:
            self.core_queue.note_kinematic_move(
                print_time, accel_t, cruise_t, decel_t, start_pos_x,
                axes_r_x, start_v, cruise_v, accel,
            )

    def _trapq_finalize_moves(self, trapq, print_time, clear_history_time):
        self._orig_trapq_finalize_moves(trapq, print_time, clear_history_time)
        if trapq == self.main_trapq and print_time >= self.reactor.NEVER:
            # End of a homing/probing "drip" move - drop the remainder
            self.core_queue.discard_all()

    # Tilt axis helpers
    def get_tilt_index(self):
        extra_axes = self.toolhead.get_extra_axes()
        if self.tilt not in extra_axes:
            raise self.printer.command_error("Tilt axis not registered")
        return extra_axes.index(self.tilt)

    def get_tilt_position(self):
        return self.toolhead.get_position()[self.get_tilt_index()]

    def get_core_steppers(self):
        return self.rail_x.get_steppers() + self.rail_b.get_steppers()

    def get_core_position(self):
        # Current core queue coordinates (X, k*B)
        pos = self.toolhead.get_position()
        return pos[0], self.b_scale * pos[self.get_tilt_index()]

    def set_core_position(self, x, y):
        # Set the X/B position from core queue coordinates
        b = y / self.b_scale
        toolhead = self.toolhead
        toolhead.flush_step_generation()
        toolhead.commanded_pos[self.get_tilt_index()] = b
        newpos = toolhead.get_position()
        newpos[0] = x
        toolhead.set_position(newpos)

    def set_tilt_position(self, b, homed=None):
        toolhead = self.toolhead
        toolhead.flush_step_generation()
        toolhead.commanded_pos[self.get_tilt_index()] = b
        if homed is True:
            self.tilt.limits = self.rail_b.get_range()
        elif homed is False:
            self.tilt.limits = (1.0, -1.0)
        toolhead.set_position(toolhead.get_position())

    # Kinematics interface
    def get_steppers(self):
        return self.get_core_steppers() + [self.stepper_c] + [
            s for s in self.rail_z.get_steppers()
        ]

    def calc_position(self, stepper_positions):
        m1 = stepper_positions[self.rail_x.get_name()]
        m2 = stepper_positions[self.rail_b.get_name()]
        c = stepper_positions[self.stepper_c.get_name()]
        z = stepper_positions[self.rail_z.get_name()]
        return [0.5 * (m1 + m2), c, z]

    def set_position(self, newpos, homing_axes):
        try:
            b = self.get_tilt_position()
        except self.printer.command_error:
            b = 0.0
        print_time = self.toolhead.print_time
        self.core_queue.set_position(print_time, newpos[0], b)
        core_pos = [newpos[0], self.b_scale * b, 0.0]
        self.rail_x.set_position(core_pos)
        self.rail_b.set_position(core_pos)
        self.stepper_c.set_position(newpos)
        self.rail_z.set_position(newpos)
        if "x" in homing_axes:
            self.limits[0] = self.rail_x.get_range()
        if "y" in homing_axes:
            self.limits[1] = self.c_range
        if "z" in homing_axes:
            self.limits[2] = self.rail_z.get_range()

    def note_z_not_homed(self):
        self.clear_homing_state("z")

    def clear_homing_state(self, clear_axes):
        for axis, axis_name in enumerate("xyz"):
            if axis_name in clear_axes:
                self.limits[axis] = (1.0, -1.0)
        if "x" in clear_axes:
            # The tilt axis shares its motors with X
            self.tilt.limits = (1.0, -1.0)

    def _check_limits(self, move):
        end_pos = move.end_pos
        for i, axis_name in enumerate("XCZ"):
            if move.axes_d[i] and (
                end_pos[i] < self.limits[i][0]
                or end_pos[i] > self.limits[i][1]
            ):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error(
                        "Must home axis %s first" % (axis_name,)
                    )
                raise move.move_error()

    def check_move(self, move):
        self._check_limits(move)
        move_d = move.move_d
        axes_d = move.axes_d
        for axis_d, max_v, max_a in (
            (axes_d[0], self.max_x_velocity, self.max_x_accel),
            (axes_d[1], self.max_y_velocity, self.max_y_accel),
            (axes_d[2], self.max_z_velocity, self.max_z_accel),
        ):
            if axis_d:
                ratio = move_d / abs(axis_d)
                move.limit_speed(max_v * ratio, max_a * ratio)

    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        axes_min, axes_max = self.axes_min, self.axes_max
        if self.mode == MODE_POLAR:
            # Cartesian X/Y need both the radius and the bed angle
            if "x" not in axes or "y" not in axes:
                axes = [a for a in axes if a == "z"]
            r = self.rail_x.get_range()[1]
            axes_min = axes_min._replace(x=-r, y=-r)
            axes_max = axes_max._replace(x=r, y=r)
        return {
            "homed_axes": "".join(axes),
            "axis_minimum": axes_min,
            "axis_maximum": axes_max,
            "theta_mode": self.mode,
            "b_homed": self.tilt.is_homed(),
        }

    # Homing
    def home(self, homing_state):
        axes = homing_state.get_axes()
        if self.home_all_includes_b and sorted(axes) == [0, 1, 2]:
            self.home_tilt()
        if 0 in axes:
            self._home_x(homing_state)
        if 1 in axes:
            self._home_c(homing_state)
        if 2 in axes:
            self._home_z(homing_state)

    def _home_rail(self, homing_state, axis, rail):
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        homing_state.home_rails([rail], forcepos, homepos)

    def _home_x(self, homing_state):
        self._home_rail(homing_state, 0, self.rail_x)

    def _home_c(self, homing_state):
        # The bed has no endstop - declare the current angle as C=0
        toolhead = self.toolhead
        pos = toolhead.get_position()
        pos[1] = 0.0
        toolhead.set_position(pos, homing_axes="y")
        gcode_move = self.printer.lookup_object("gcode_move")
        gcode_move.base_position[1] = gcode_move.homing_position[1]
        gcode_move.reset_last_position()

    def _home_z(self, homing_state):
        toolhead = self.toolhead
        if self.limits[0][0] > self.limits[0][1] or not self.tilt.is_homed():
            raise self.printer.command_error(
                "Axes X and B must be homed before Z"
            )
        # Lift the nozzle before rotating the toolhead
        pos = toolhead.get_position()
        if self.z_hop:
            if self.limits[2][0] > self.limits[2][1]:
                # Z not homed - assume the gantry is at Z=0
                pos[2] = 0.0
                toolhead.set_position(pos, homing_axes="z")
                pos[2] = self.z_hop
                toolhead.manual_move(pos, self.z_hop_speed)
                toolhead.wait_moves()
                self.clear_homing_state("z")
            elif pos[2] < self.z_hop:
                pos[2] = self.z_hop
                toolhead.manual_move(pos, self.z_hop_speed)
        # Move the probe over the homing point, facing down
        tilt_index = self.get_tilt_index()
        pos = toolhead.get_position()
        pos[tilt_index] = self.probe_b
        toolhead.manual_move(pos, self.b_travel_speed)
        pos[0] = self.z_home_x
        toolhead.manual_move(pos, self.z_home_travel_speed)
        # Home Z with the probe
        self._home_rail(homing_state, 2, self.rail_z)
        # Lift and point the nozzle down again
        pos = toolhead.get_position()
        if self.z_hop:
            pos[2] = max(pos[2], self.z_hop)
            toolhead.manual_move(pos, self.z_hop_speed)
        if self.restore_nozzle_after_z_home:
            pos[tilt_index] = self.nozzle_b
            toolhead.manual_move(pos, self.b_travel_speed)

    def _set_homing_current(self, rails, pre_homing):
        toolhead = self.toolhead
        print_time = toolhead.get_last_move_time()
        dwell_time = 0.0
        for rail in rails:
            for ch in rail.get_tmc_current_helpers():
                if ch is not None:
                    dwell_time = max(
                        dwell_time, ch.set_current_for_homing(
                            print_time, pre_homing)
                    )
        if dwell_time:
            toolhead.dwell(dwell_time)

    def home_tilt(self):
        """Home the tilt axis with the stepper_b endstop.  Only B moves;
        the X position does not need to be known."""
        toolhead = self.toolhead
        rail = self.rail_b
        hi = rail.get_homing_info()
        position_min, position_max = rail.get_range()
        if hi.positive_dir:
            start_b = hi.position_endstop - 1.5 * (
                hi.position_endstop - position_min
            )
            retract_b = hi.position_endstop - hi.retract_dist
        else:
            start_b = hi.position_endstop + 1.5 * (
                position_max - hi.position_endstop
            )
            retract_b = hi.position_endstop + hi.retract_dist
        k = self.b_scale
        accel = hi.accel if hi.accel is not None else self.tilt.max_accel
        helper = TiltHomingHelper(self, k * accel)
        endstops = rail.get_endstops()
        x = toolhead.get_position()[0]
        homepos = [x, k * hi.position_endstop, 0.0, 0.0]
        core_rails = [self.rail_x, self.rail_b]
        self.set_tilt_position(start_b, homed=False)
        speeds = [hi.speed]
        if hi.retract_dist and not hi.use_sensorless_homing:
            speeds.append(hi.second_homing_speed)
        for i, speed in enumerate(speeds):
            if i:
                # Retract before homing again
                self._tilt_raw_move(retract_b, hi.retract_speed, accel)
                self.set_tilt_position(retract_b)
            self._set_homing_current(core_rails, True)
            print_time = toolhead.get_last_move_time()
            for mcu_endstop, name in endstops:
                mcu_endstop.query_endstop(print_time)
            try:
                hmove = homing_mod.HomingMove(self.printer, endstops, helper)
                hmove.homing_move(homepos, k * speed)
                if i and hmove.check_no_movement() is not None:
                    raise self.printer.command_error(
                        "Endstop %s still triggered after retract"
                        % (hmove.check_no_movement(),)
                    )
            finally:
                self._set_homing_current(core_rails, False)
        self.set_tilt_position(hi.position_endstop, homed=True)
        if hi.retract_dist:
            pos = toolhead.get_position()
            pos[self.get_tilt_index()] = retract_b
            toolhead.manual_move(pos, hi.retract_speed)
        toolhead.flush_step_generation()

    def _tilt_raw_move(self, b, speed, accel):
        # Move only the tilt axis, bypassing homing checks
        toolhead = self.toolhead
        toolhead.flush_step_generation()
        cq = self.core_queue
        start_x, start_y = self.get_core_position()
        k = self.b_scale
        axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(
            k * b - start_y, k * speed, k * accel
        )
        print_time = toolhead.get_last_move_time()
        cq.append(print_time, accel_t, cruise_t, accel_t, start_x, start_y,
                  0.0, axis_r, 0.0, cruise_v, k * accel)
        toolhead.note_mcu_movequeue_activity(
            print_time + 2.0 * accel_t + cruise_t, set_step_gen_time=True
        )
        toolhead.dwell(2.0 * accel_t + cruise_t)
        toolhead.flush_step_generation()
        cq.discard_all()


def load_kinematics(toolhead, config):
    return CoreRThetaKinematics(toolhead, config)


######################################################################
# Polar (Cartesian G-code) mode
######################################################################


class ThetaMoveTransform:
    """G-code move transform.  In 4 axis mode G-code coordinates are
    machine coordinates and moves pass straight through.  In polar mode
    G-code X/Y are Cartesian bed coordinates: each move is split into
    short segments that are linear in machine (X=radius, C=angle) space,
    within polar_max_deviation of the straight Cartesian path.  The
    toolhead then applies all machine axis limits, which automatically
    slows moves that pass close to the bed centre."""

    def __init__(self, rtheta, config):
        self.rtheta = rtheta
        self.printer = rtheta.printer
        self.next_transform = None
        self.max_deviation = config.getfloat(
            "polar_max_deviation", 0.01, above=0.0
        )
        self.max_segment_angle = config.getfloat(
            "polar_max_segment_angle", 5.0, above=0.0
        )
        self.min_segment_length = config.getfloat(
            "polar_min_segment_length", 0.005, above=0.0
        )
        self.center_tolerance = config.getfloat(
            "polar_center_tolerance", 0.05, minval=0.0
        )

    def is_polar(self):
        return self.rtheta.kin.mode == MODE_POLAR

    @staticmethod
    def machine_to_cartesian(pos):
        pos = list(pos)
        radius, angle = pos[0], math.radians(pos[1])
        pos[0] = radius * math.cos(angle)
        pos[1] = radius * math.sin(angle)
        return pos

    def get_position(self):
        pos = self.next_transform.get_position()
        if self.is_polar():
            return self.machine_to_cartesian(pos)
        return pos

    def move(self, newpos, speed):
        if not self.is_polar():
            self.next_transform.move(newpos, speed)
            return
        self._polar_move(newpos, speed)

    # Polar mode move splitting
    def _machine_point(self, cart, near_angle):
        # Machine coordinates (radius >= 0) for a Cartesian point, with
        # the bed angle unwrapped to be closest to near_angle
        mpos = list(cart)
        x, y = cart[0], cart[1]
        radius = math.sqrt(x * x + y * y)
        if radius < 1e-9:
            angle = near_angle
        else:
            angle = math.degrees(math.atan2(y, x))
            angle += 360.0 * round((near_angle - angle) / 360.0)
        mpos[0] = radius
        mpos[1] = angle
        return mpos

    def _subdivide(self, pa, ma, pb, mb, depth, out):
        dx, dy = pb[0] - pa[0], pb[1] - pa[1]
        seg_len = math.sqrt(dx * dx + dy * dy)
        if depth < 24 and seg_len > self.min_segment_length:
            pm = [0.5 * (a + b) for a, b in zip(pa, pb)]
            mm_lin = [0.5 * (a + b) for a, b in zip(ma, mb)]
            lin_xy = self.machine_to_cartesian(mm_lin)
            dev = math.hypot(lin_xy[0] - pm[0], lin_xy[1] - pm[1])
            if (dev > self.max_deviation
                    or abs(mb[1] - ma[1]) > self.max_segment_angle):
                mm = self._machine_point(pm, ma[1])
                self._subdivide(pa, ma, pm, mm, depth + 1, out)
                self._subdivide(pm, mm, pb, mb, depth + 1, out)
                return
        out.append((pa, ma, pb, mb))

    @staticmethod
    def _toolhead_dist(start, end):
        axes_d = [e - s for s, e in zip(start, end)]
        move_d = math.sqrt(sum([d * d for d in axes_d[:3]]))
        if move_d < 0.000000001:
            move_d = max([abs(d) for d in axes_d[3:]] + [0.0])
        return move_d

    def _polar_move(self, newpos, speed):
        nt = self.next_transform
        mstart = nt.get_position()
        if mstart[0] < -self.center_tolerance:
            raise self.printer.command_error(
                "Polar mode requires X >= 0 (X=%.3f); move the carriage "
                "to the positive side of the bed centre first"
                % (mstart[0],)
            )
        cstart = self.machine_to_cartesian(mstart)
        if abs(mstart[0]) <= self.center_tolerance:
            # At the centre the bed angle is arbitrary - keep it
            cstart[0] = cstart[1] = 0.0
        segments = []
        if (abs(newpos[0] - cstart[0]) < 1e-9
                and abs(newpos[1] - cstart[1]) < 1e-9):
            # No X/Y motion - keep the exact machine X and C
            mend = list(newpos)
            mend[0], mend[1] = mstart[0], mstart[1]
            segments.append((cstart, mstart, list(newpos), mend))
        else:
            mend = self._machine_point(newpos, mstart[1])
            self._subdivide(cstart, mstart, list(newpos), mend, 0, segments)
        toolhead = self.printer.lookup_object("toolhead")
        limits = (toolhead.max_velocity, toolhead.max_accel,
                  toolhead.square_corner_velocity)
        try:
            for pa, ma, pb, mb in segments:
                cart_d = self._toolhead_dist(pa, pb)
                mach_d = self._toolhead_dist(ma, mb)
                if not mach_d:
                    continue
                # Scale velocity/accel limits so they apply to the
                # Cartesian path rather than to machine units
                q = mach_d / cart_d if cart_d else 1.0
                toolhead.max_velocity = limits[0] * q
                toolhead.max_accel = limits[1] * q
                toolhead.square_corner_velocity = limits[2] * q
                toolhead._calc_junction_deviation()
                nt.move(mb, speed * q)
        finally:
            (toolhead.max_velocity, toolhead.max_accel,
             toolhead.square_corner_velocity) = limits
            toolhead._calc_junction_deviation()


######################################################################
# G-code support ([core_rtheta] config section)
######################################################################


class CoreRTheta:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode_move = None
        self.kin = None
        self.inverse_time = False
        self.inverse_feed = None
        self.prev_cmds = {}
        self.transform = ThetaMoveTransform(self, config)
        self.initial_mode = config.getchoice(
            "mode", {"4axis": MODE_4AXIS, "polar": MODE_POLAR}, "4axis"
        )
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )

    def _handle_connect(self):
        toolhead = self.printer.lookup_object("toolhead")
        kin = toolhead.get_kinematics()
        if not isinstance(kin, CoreRThetaKinematics):
            raise self.printer.config_error(
                "[%s] requires 'kinematics: %s' in [printer]"
                % (KINEMATICS_NAME, KINEMATICS_NAME)
            )
        self.kin = kin
        kin.mode = self.initial_mode
        self.gcode_move = gm = self.printer.lookup_object("gcode_move")
        if gm.move_transform is not None:
            logging.warning(
                "core_rtheta: G-code move transform '%s' will receive "
                "machine coordinates", type(gm.move_transform).__name__
            )
        self.transform.next_transform = gm.set_move_transform(
            self.transform, force=True
        )
        gcode = self.gcode
        for cmd in ["G0", "G1", "G92", "M114", "G28"]:
            self.prev_cmds[cmd] = gcode.register_command(cmd, None)
        gcode.register_command("G0", self.cmd_G1)
        gcode.register_command("G1", self.cmd_G1)
        # Zero padded forms (accepted by RRF, used by the radial slicer)
        for cmd in ["G00", "G01"]:
            if gcode.register_command(cmd, None) is None:
                gcode.register_command(cmd, self.cmd_G1)
        gcode.register_command("G92", self.cmd_G92)
        gcode.register_command("M114", self.cmd_M114, True)
        gcode.register_command("G28", self.cmd_G28)
        gcode.register_command("G93", self.cmd_G93)
        gcode.register_command("G94", self.cmd_G94)
        gcode.register_command(
            "THETA_STATUS", self.cmd_THETA_STATUS,
            desc=self.cmd_THETA_STATUS_help,
        )
        gcode.register_command(
            "THETA_SET_POSITION", self.cmd_THETA_SET_POSITION,
            desc=self.cmd_THETA_SET_POSITION_help,
        )
        gcode.register_command(
            "THETA_MODE", self.cmd_THETA_MODE, desc=self.cmd_THETA_MODE_help
        )

    def get_axis_map(self):
        # G-code letter to G-code position index
        axis_map = dict(self.gcode_move.axis_map)
        if self.kin.mode == MODE_4AXIS:
            axis_map.pop("Y", None)
            axis_map["C"] = 1
        return axis_map

    def check_letters(self, gcmd):
        params = gcmd.get_command_parameters()
        if self.kin.mode == MODE_4AXIS and "Y" in params:
            raise gcmd.error(
                "Y is not an axis in 4 axis mode (use C for bed rotation)"
            )
        if self.kin.mode == MODE_POLAR and "C" in params:
            raise gcmd.error(
                "C is not available in polar mode (use THETA_MODE MODE=4AXIS)"
            )

    def set_mode(self, mode):
        kin = self.kin
        if mode == kin.mode:
            return
        toolhead = self.printer.lookup_object("toolhead")
        if (mode == MODE_POLAR
                and toolhead.get_position()[0]
                < -self.transform.center_tolerance):
            raise self.printer.command_error(
                "Polar mode requires X >= 0; move the carriage to the "
                "positive side of the bed centre first"
            )
        kin.mode = mode
        # X/Y(C) now have a different meaning - drop their G-code offsets
        gm = self.gcode_move
        for i in (0, 1):
            gm.base_position[i] = gm.homing_position[i]
        gm.reset_last_position()

    cmd_THETA_MODE_help = (
        "Select 4AXIS (machine X/C/Z/B coordinates) or POLAR (Cartesian "
        "X/Y/Z) G-code mode"
    )

    def cmd_THETA_MODE(self, gcmd):
        mode = gcmd.get("MODE", None)
        if mode is not None:
            modes = {"4AXIS": MODE_4AXIS, "4-AXIS": MODE_4AXIS,
                     "FOURAXIS": MODE_4AXIS, "POLAR": MODE_POLAR}
            if mode.upper() not in modes:
                raise gcmd.error("Unknown mode '%s'" % (mode,))
            self.set_mode(modes[mode.upper()])
        gcmd.respond_info("Core R-Theta mode: %s" % (self.kin.mode,))

    def calc_move_speed(self, startpos, endpos, speed_factor):
        axes_d = [ep - sp for sp, ep in zip(startpos, endpos)]
        th_move_d = math.sqrt(sum([d * d for d in axes_d[:3]]))
        if th_move_d < 0.000000001:
            th_move_d = max([abs(d) for d in axes_d[3:]] + [0.0])
        if not th_move_d:
            return None
        if self.inverse_time:
            # Move must complete in 1/F minutes
            return th_move_d * self.inverse_feed * speed_factor
        gcode_speed = self.gcode_move.speed
        if self.kin.mode != MODE_4AXIS:
            return gcode_speed
        # Feed rate applies to the length of the move in (X, C, Z, B),
        # all treated as linear axes (like RepRapFirmware "M584 S0")
        tilt_index = self.kin.get_tilt_index()
        d4 = math.sqrt(
            sum([d * d for d in axes_d[:3]]) + axes_d[tilt_index] ** 2
        )
        if d4 < 0.000000001:
            return gcode_speed
        return th_move_d * gcode_speed / d4

    def cmd_G1(self, gcmd):
        self.check_letters(gcmd)
        gm = self.gcode_move
        params = gcmd.get_command_parameters()
        startpos = list(gm.last_position)
        try:
            for axis, pos in self.get_axis_map().items():
                if axis in params:
                    v = float(params[axis])
                    absolute_coord = gm.absolute_coord
                    if axis == "E":
                        v *= gm.extrude_factor
                        if not gm.absolute_extrude:
                            absolute_coord = False
                    if not absolute_coord:
                        gm.last_position[pos] += v
                    else:
                        gm.last_position[pos] = v + gm.base_position[pos]
            if "F" in params:
                feed = float(params["F"])
                if feed <= 0.0:
                    raise gcmd.error(
                        "Invalid speed in '%s'" % (gcmd.get_commandline(),)
                    )
                if self.inverse_time:
                    self.inverse_feed = feed
                else:
                    gm.speed = feed * gm.speed_factor
        except ValueError:
            raise gcmd.error(
                "Unable to parse move '%s'" % (gcmd.get_commandline(),)
            )
        if self.inverse_time and self.inverse_feed is None:
            raise gcmd.error("G93 (inverse time) moves require an F value")
        speed = self.calc_move_speed(
            startpos, gm.last_position, gm.speed_factor
        )
        if speed is None:
            speed = gm.speed
        gm.move_with_transform(gm.last_position, speed)

    def cmd_G92(self, gcmd):
        self.check_letters(gcmd)
        gm = self.gcode_move
        seen = False
        for axis, pos in self.get_axis_map().items():
            offset = gcmd.get_float(axis, None)
            if offset is None:
                continue
            if axis == "E":
                offset *= gm.extrude_factor
            gm.base_position[pos] = gm.last_position[pos] - offset
            seen = True
        if not seen:
            gm.base_position[:] = gm.last_position[:]

    def get_gcode_position(self):
        # [(letter, position)] in G-code coordinates (after G92 offsets)
        gm = self.gcode_move
        p = [lp - bp for lp, bp in zip(gm.last_position, gm.base_position)]
        p[3] /= gm.extrude_factor
        axes = sorted(self.get_axis_map().items(), key=lambda item: item[1])
        return [(a, p[i]) for a, i in axes]

    def cmd_M114(self, gcmd):
        parts = ["%s:%.3f" % lp for lp in self.get_gcode_position()]
        gcmd.respond_raw(" ".join(parts))

    def cmd_G28(self, gcmd):
        params = gcmd.get_command_parameters()
        home_b = "B" in params
        axes = [a for a in "XYZ" if a in params]
        if self.kin.mode == MODE_4AXIS and "C" in params:
            axes.append("Y")
        if self.kin.mode == MODE_POLAR and ("X" in axes or "Y" in axes):
            # Cartesian X and Y both need the radius and bed angle
            axes.extend(["X", "Y"])
        if home_b:
            try:
                self.kin.home_tilt()
            except self.printer.command_error:
                if self.printer.is_shutdown():
                    raise self.printer.command_error(
                        "Homing failed due to printer shutdown"
                    )
                self.printer.lookup_object("stepper_enable").motor_off()
                raise
            if not axes:
                return
        if not home_b and not axes and ("C" in params or "Y" in params):
            return
        cmdline = "G28 " + " ".join(["%s0" % a for a in sorted(set(axes))])
        new_params = dict((a, "0") for a in axes)
        newcmd = self.gcode.create_gcode_command("G28", cmdline, new_params)
        self.prev_cmds["G28"](newcmd)

    def cmd_G93(self, gcmd):
        # Inverse time feed rate mode
        self.inverse_time = True
        self.inverse_feed = None

    def cmd_G94(self, gcmd):
        # Units per minute feed rate mode
        self.inverse_time = False

    cmd_THETA_STATUS_help = "Report Core R-Theta kinematic state"

    def cmd_THETA_STATUS(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        pos = toolhead.get_position()
        kin = self.kin
        tilt_index = kin.get_tilt_index()
        homed = kin.get_status(None)["homed_axes"]
        gcode_pos = " ".join(
            ["%s=%.4f" % lp for lp in self.get_gcode_position()]
        )
        gcmd.respond_info(
            "mode: %s\n"
            "toolhead: X=%.4f C=%.4f Z=%.4f B=%.4f E=%.4f\n"
            "gcode: %s\n"
            "homed: %s%s\n"
            "feed mode: %s"
            % (
                kin.mode,
                pos[0], pos[1], pos[2], pos[tilt_index], pos[3],
                gcode_pos,
                homed.replace("y", "c"), "b" if kin.tilt.is_homed() else "",
                "G93 inverse time" if self.inverse_time else "G94 units/min",
            )
        )

    cmd_THETA_SET_POSITION_help = (
        "Force the machine position of X, C, Z and/or B and mark those "
        "axes homed (use with care)"
    )

    def cmd_THETA_SET_POSITION(self, gcmd):
        # Always uses machine coordinates, in either G-code mode
        toolhead = self.printer.lookup_object("toolhead")
        kin = self.kin
        toolhead.flush_step_generation()
        pos = toolhead.get_position()
        homing_axes = ""
        letters = [("X", 0, "x"), ("C", 1, "y"), ("Z", 2, "z")]
        for letter, index, axis_name in letters:
            v = gcmd.get_float(letter, None)
            if v is not None:
                pos[index] = v
                homing_axes += axis_name
        b = gcmd.get_float("B", None)
        if b is not None:
            pos[kin.get_tilt_index()] = b
            toolhead.commanded_pos[kin.get_tilt_index()] = b
            kin.tilt.limits = kin.rail_b.get_range()
        toolhead.set_position(pos, homing_axes=homing_axes)

    def get_status(self, eventtime):
        return {
            "inverse_time": self.inverse_time,
            "mode": self.kin.mode if self.kin is not None else MODE_4AXIS,
        }


def load_config(config):
    return CoreRTheta(config)
