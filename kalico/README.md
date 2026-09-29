# ThetaFirm on Kalico

This directory lets the Core R-Theta printer run on
[Kalico](https://github.com/KalicoCrew/kalico) (a Klipper fork) instead of
RepRapFirmware.  It contains:

| Path | What it is |
| --- | --- |
| `plugins/core_rtheta.py` | Kalico plugin: `core_rtheta` kinematics and G-code support |
| `config/printer.cfg` | Printer configuration for the Mellow Fly-E3-Pro-v3, translated from `reprap firmware config/` |
| `config/macros.cfg` | Mode switching, tilt poses, `PRINT_START` / `PRINT_END` |
| `install.sh` | Links the plugin into a Kalico installation |
| `tests/` | Batch-mode simulation tests that run the real Kalico code |

**Status:** kinematics, G-code handling, homing and polar mode are verified in
Kalico's batch simulation mode (see [Tests](#tests)).  The configuration has
not been run on the physical printer yet, so work through the
[commissioning checklist](#commissioning-checklist) before the first print.

## How it works

| Axis | Motor(s) | Units | Notes |
| --- | --- | --- | --- |
| X | `stepper_x` + `stepper_b` | mm | Radial carriage position.  Signed: negative X is past the bed centre (RRF limits -37.5 to 115.5). |
| C | `stepper_c` | degrees | Bed rotation.  Unlimited, never wraps. |
| Z | `stepper_z` | mm | Gantry height. |
| B | `stepper_x` + `stepper_b` | degrees | Nozzle tilt.  0 = nozzle down, -90 = Z probe down, +90 = end stop. |

X and B share two motors in a "core" arrangement:

    stepper_x = X - k*B        stepper_b = X + k*B

where `k = b_mm_per_degree` (0.2222 mm of belt per degree: 40T tilt pulley,
2 mm pitch).  This is the same coupling as the RRF `M669` matrix in
`to4axis.g`.

Kalico's toolhead plans three kinematic coordinates plus "extra axes".  The
plugin uses (X, C, Z) as the kinematic coordinates, so bed rotation gets full
look-ahead, junction speed control, pressure advance and per-axis limits.
Most moves from a radial slicer are rotate-and-extrude moves.  B is
registered as an extra axis, so the `B` letter works in G-code.  The two core
motors read a private motion queue holding (X, k·B), which the plugin fills
from the toolhead's moves, and use Kalico's stock corexy step solvers.  No C
code or changes to Kalico are needed.

## Installation

1. Install Kalico, for example with KIAUH.  Mainline Klipper does not support
   plugins, so it won't work.
2. Clone this repository on the printer host and run the installer:

   ```sh
   git clone https://github.com/pdamuk/ThetaFirm.git ~/ThetaFirm
   ~/ThetaFirm/kalico/install.sh          # or: install.sh /path/to/kalico
   ```

   The installer symlinks the plugin into `klippy/plugins/`.  Kalico ignores
   that directory in git, so Kalico updates keep working.  The script also
   prints a Moonraker `update_manager` entry.
3. Copy `config/printer.cfg` and `config/macros.cfg` to your configuration
   directory (usually `~/printer_data/config/`), then set the `[mcu] serial`.
   Build Kalico's firmware for the STM32F407 with a 32KiB bootloader and USB.
4. Restart Kalico, then follow the commissioning checklist.

The `[core_rtheta]` section must be present.  Loading it is what makes
`kinematics: core_rtheta` available.

## Configuration reference

```ini
[printer]
kinematics: core_rtheta
max_velocity: 500          # applies to the (X, C, Z) move length
max_accel: 5000

[core_rtheta]
b_mm_per_degree: 0.222222  # required: belt mm per degree of tilt (may be negative)
#mode: 4axis               # G-code mode at startup: 4axis or polar
#max_x_velocity / max_x_accel    (mm/s, mm/s^2; default: printer limits)
#max_c_velocity / max_c_accel    (deg/s, deg/s^2)
#max_z_velocity / max_z_accel
#max_b_velocity / max_b_accel
#b_instantaneous_corner_velocity: 1.0   # deg/s, like the extruder setting
#c_position_min: -1e9
#c_position_max: 1e9
#nozzle_b_angle: 0         # B with the nozzle pointing down
#probe_b_angle: -90        # B with the Z probe pointing down
#z_home_x: 0               # radius at which Z is probed
#z_hop: 10                 # lift before rotating the toolhead to probe
#z_hop_speed: 15
#z_home_travel_speed: 50
#b_travel_speed: max_b_velocity
#restore_nozzle_after_z_home: True
#home_all_includes_b: True # plain G28 also homes B
#polar_max_deviation: 0.01      # mm, polar mode path tolerance
#polar_max_segment_angle: 5     # deg, polar mode segment limit
#polar_min_segment_length: 0.005
#polar_center_tolerance: 0.05
```

Motor sections:

* `[stepper_x]` is core motor A (RRF driver 1).  Its `position_min`,
  `position_max`, `position_endstop` and homing options describe the X axis.
  `position_endstop` replaces RRF's `X_LENGTH`.
* `[stepper_b]` is core motor B (RRF driver 2).  Its position and homing
  options describe the B axis, in degrees.  `position_endstop` replaces RRF's
  `90 - B_OFFSET`.
* `[stepper_c]` is the bed (RRF driver 0).  Use `rotation_distance` in
  degrees per motor revolution.  It has no endstop.
* `[stepper_z]` uses the probe as its endstop (`probe:z_virtual_endstop`).

Both core motors use `rotation_distance` in mm of belt (32 for a 16T GT2
pulley).  `SET_VELOCITY_LIMIT` accepts `X_VELOCITY`/`X_ACCEL`, `Y_VELOCITY`/
`Y_ACCEL` (these set the **C** limits) and `Z_VELOCITY`/`Z_ACCEL`.

## G-code reference (for slicers)

This section describes the G-code dialect for the companion OrcaSlicer fork
and other slicers.

### Modes

`THETA_MODE MODE=4AXIS|POLAR` switches the coordinate system at any time.
Queued moves are unaffected.  The default is `mode:` in `[core_rtheta]`,
which is 4axis unless changed.  The current mode is reported by
`THETA_STATUS` and `printer.core_rtheta.mode`.

### 4 axis mode (machine coordinates)

| Word | Meaning |
| --- | --- |
| `X` | Carriage radius, mm, signed |
| `C` | Bed angle, degrees, unlimited.  Accumulate the angle; do not wrap to ±180. |
| `Z` | Height, mm |
| `B` | Tilt, degrees (0 = nozzle down) |
| `E` | Extruder (`M82`/`M83` as usual) |
| `Y` | Rejected with an error |

* **Bed angle convention:** a bed point at polar angle θ (in the bed's own
  frame, `θ = atan2(y, x)`) is under the nozzle when `C = θ`.  This matches
  jyjblrd's radial slicer and Kalico's polar kinematics.  If prints come out
  mirrored, invert `stepper_c`'s `dir_pin`.
* **Tool centre point:** the firmware does not compensate for the nozzle tip
  being about 43 mm from the tilt axis.  As with RRF, the slicer must output
  joint positions.  For a tilt of `t = -B` (radians) the radial slicer uses
  `X = r + sin(t)·L` and `Z = z + (cos(t) − 1)·L`, with `L = 43` mm.
  Measure `L` on your printer.
* **`G94` (default), units per minute:** `F` is the feed along the Euclidean
  length of (X, C, Z, B), with degrees treated as mm.  This matches RRF's
  `M584 ... S0`.
* **`G93`, inverse time:** each move takes `1/F` minutes.  After `G93`, `F`
  must be given before the first move.  It then persists.  This is the
  recommended mode for 4 axis prints because it gives exact nozzle speeds
  regardless of the unit mix.  `M220` scales both modes.
* `G0` behaves like `G1`.  `G00`/`G01` are accepted, as RRF does.
* `G92` works on every axis, including `B` and `C`.
* Moves are limited by the per-axis velocity/accel limits and by
  `max_velocity`/`max_accel` along the (X, C, Z) length.

Example (radial slicer style):

```gcode
PRINT_START MODE=4AXIS HOTEND=215
G93
G1 C46.310 X11.534 Z-1.256 B-15.089 E0.0198 F1642.71
G1 C0.646 X11.551 Z-1.261 B-15.093 E0.0211 F1544.88
G94
G0 C88.576 X11.560 Z2 B-15.094 F50000
PRINT_END
```

### Polar mode (Cartesian coordinates)

`X`, `Y` and `Z` are Cartesian mm in the bed's frame, with the origin at the
bed centre.  `F` is mm/min along the XYZ path (or use `G93`).  `B` still
tilts the nozzle and can move together with X/Y.  `C` is rejected.

Each move is split into segments that are straight in (radius, angle)
space and stay within `polar_max_deviation` of the ideal line.
Accelerations and speeds apply to the Cartesian path, while the bed
rotation limits (`max_c_velocity`/`max_c_accel`) still apply.  Moves
therefore slow down near the bed centre.  A line through the exact centre
makes the bed turn 180° while the nozzle is at the centre.  Polar mode
keeps X ≥ 0; switching to it with the carriage past the centre is refused.

### Commands

| Command | Description |
| --- | --- |
| `G28` | Home B, X, C, Z.  `G28 B`, `G28 X`, `G28 C` (or `Y`) and `G28 Z` home single axes.  In polar mode `G28 X`/`G28 Y` home both X and C. |
| `THETA_MODE MODE=4AXIS\|POLAR` | Select the G-code coordinate system. |
| `THETA_STATUS` | Report mode, machine position, G-code position, homed axes and feed mode. |
| `THETA_SET_POSITION [X=] [C=] [Z=] [B=]` | Force machine positions and mark those axes homed.  Always uses machine coordinates.  For commissioning and recovery only. |
| `G93` / `G94` | Inverse time / units per minute feed rate. |
| `M114` | Position in the current mode's letters (for example `X: C: Z: E: B:`). |
| `TO_4AXIS`, `TO_POLAR` | Macros matching RRF `to4axis.g` / `topolar.g`. |
| `NOZZLE_DOWN`, `PROBE_DOWN` | Rotate B to the nozzle or probe pose. |
| `PRINT_START [MODE=] [HOTEND=]`, `PRINT_END` | Start and end macros. |
| `SAVE_GCODE_STATE` / `RESTORE_GCODE_STATE` | Also save and restore B and the G93/G94 mode, so `PAUSE`/`RESUME` return the nozzle tilt too. |
| `G2` / `G3` | Arcs, polar mode only (`[gcode_arcs]` is enabled; rejected in 4 axis mode). |
| `T0`, `G10 P0 S<t>`, `M116`, `M572 D0 S<pa>` | Macros so RepRapFirmware-flavour slicer output works: tool select (no-op), tool temperature, wait for temperature, pressure advance. |

Status variables for macros: `printer.toolhead.b_homed`,
`printer.toolhead.theta_mode`, `printer.core_rtheta.mode`,
`printer.core_rtheta.inverse_time`.  `printer.toolhead.homed_axes` uses `y`
for C.

#### Homing sequence

`G28` runs the same order as RRF `homeall.g` in 4 axis mode:

1. **B:** sensorless on core motor B (`stepper_b` endstop), towards +90.  Only
   the tilt moves.
2. **X:** sensorless on core motor A, towards `position_endstop`.  Both core
   motors stop on either endstop.
3. **C:** the current bed angle becomes C=0.  There is no C endstop.
4. **Z:**
   1. Lift by `z_hop`.
   2. Tilt to `probe_b_angle` and move to `z_home_x`.
   3. Probe twice.
   4. Lift again and tilt back to `nozzle_b_angle`.

Homing Z needs X and B homed.  Don't combine this with `[safe_z_home]`,
because it would move the toolhead in machine coordinates.

## Commissioning checklist

1. **Directions.**  Enable `[force_move] enable_force_move: True`, then move
   each motor alone and compare with the table.  Fix the `dir_pin`, then
   disable force_move again.

   | Command | Expected motion |
   | --- | --- |
   | `FORCE_MOVE STEPPER=stepper_c DISTANCE=10 VELOCITY=10` | Bed turns 10° clockwise seen from above (see the bed angle convention) |
   | `FORCE_MOVE STEPPER=stepper_x DISTANCE=4 VELOCITY=5` | Carriage moves 2 mm outwards and the nozzle tilts 9° towards the probe side (B −9) |
   | `FORCE_MOVE STEPPER=stepper_b DISTANCE=4 VELOCITY=5` | Carriage moves 2 mm outwards and the nozzle tilts 9° towards the B end stop (B +9) |
   | `FORCE_MOVE STEPPER=stepper_z DISTANCE=5 VELOCITY=5` | Gantry moves up 5 mm |
   | `FORCE_MOVE STEPPER=extruder DISTANCE=5 VELOCITY=2` | Filament goes in |

2. **Sensorless homing.**  Tune `driver_SGTHRS`, `home_current` and
   `homing_speed` for `stepper_b`, then `stepper_x`, using Kalico's sensorless
   homing guide.  Test with `G28 B` first, then `G28 X`, keeping a hand on the
   power switch.
3. **Axis offsets.**  Adjust `[stepper_b] position_endstop` until
   `NOZZLE_DOWN` points the nozzle straight down.  This replaces RRF's
   `B_OFFSET`.  Adjust `[stepper_x] position_endstop` until X=0 is the bed
   centre (RRF's `X_LENGTH`).
4. **Probe.**  Run `G28`, then check that `G1 Z0` with the nozzle down just
   touches the bed.  Correct `[probe] z_offset` by the error.  RRF used
   `G31 Z-8.5`.
5. **Heater.**  Run `PID_CALIBRATE HEATER=extruder TARGET=215`, then
   `SAVE_CONFIG`.
6. **First motion.**  Home, then try slow moves in both modes, for example
   `G1 C360 F3000`, `G1 X50 F1200`, `G1 B-45 F600`, then
   `TO_POLAR` and `G1 X20 Y20 F1200`.

## Differences from the RRF configuration

| RRF | Kalico |
| --- | --- |
| `to4axis.g` / `topolar.g` (reconfigure drives) | `THETA_MODE` (runtime switch, no reconfiguration) |
| `M669 K0` matrix | `kinematics: core_rtheta`, `b_mm_per_degree` |
| `M669 K7` polar with segmentation | Polar mode via segmentation in machine space |
| `M569` directions | `dir_pin` inversion (verify) |
| `M906` peak currents | `run_current` (RMS = peak × 0.707) |
| `M913` homing currents | `home_current` |
| `M915` stall thresholds | `driver_SGTHRS` (retune) |
| `homeb.g`, `homex.g`, `homez.g` | Built into `G28` |
| Polar mode B homing (drive V against the hard stop) | Sensorless B homing in both modes |
| `global B_OFFSET`, `X_LENGTH` | `[stepper_b]` / `[stepper_x]` `position_endstop` |
| `M84 S30` idle timeout | `[idle_timeout] timeout: 600` (motors off loses homing) |
| `M307` heater model | PID; recalibrate (MPC is also available in Kalico) |

## Limitations and future work

* Not yet validated on the real printer (sensorless thresholds and
  directions need tuning).
* No tool centre point compensation: the slicer outputs joint positions,
  the same as with RRF.  Firmware compensation could be added as another
  mode later.
* Cartesian calibration tools (`BED_MESH_CALIBRATE`, `SCREWS_TILT_CALCULATE`
  and similar) move the toolhead in machine coordinates, so they are not
  supported.  `[input_shaper]` has not been validated with this kinematics.
* Kalico only reports unknown G-code commands; it does not stop.  For the
  OrcaSlicer fork, the Klipper G-code flavour is recommended.  The RRF
  compatibility macros cover the common RRF commands.
* Object cancelling (`EXCLUDE_OBJECT`, used by Mainsail and Fluidd) works in
  both modes.  Kalico's `exclude_object` assumes exactly four axes and would
  crash with the B axis, so the plugin patches it at start-up.

## Tests

The tests run Kalico's real `klippy` in batch mode.  They decode the MCU
command stream and rebuild every stepper's position over time, then check:

* single-axis and combined moves against the coupling equations, including a
  check that X, B, C and Z stay synchronised along a combined move
* G93 durations, RRF-style G94 feed rates, `G92`/`G91`/`M114` with B and C,
  and G-code in the radial slicer's style
* every homing path, and the homing and motor-off state handling
* polar mode: the reconstructed Cartesian path stays within tolerance of the
  ideal line, including lines through and next to the bed centre; Cartesian
  feed rates; mode round trips
* compatibility: object cancelling, `PAUSE`/`RESUME` with tilt and feed
  mode, arcs in polar mode, RRF-flavour commands, `FORCE_MOVE`, probe
  commands, and unlimited bed rotation

```sh
kalico/tests/setup_env.sh          # clones Kalico, builds the MCU dictionary, makes a venv
# run the printed export lines, then:
cd kalico/tests && pytest -q
```

`setup_env.sh` needs `arm-none-eabi-gcc` (`apt install gcc-arm-none-eabi
libnewlib-arm-none-eabi`).  GitHub Actions runs the suite for every change
under `kalico/`.  It tests against the pinned Kalico commit and, as an early
warning, against Kalico `main`.

In addition, the first 30,000 lines of the radial slicer's propeller G-code
were run in simulation.  Motor positions matched the final G-code position
after 49,000° of accumulated bed rotation, and every G93 move was planned
with exactly its requested duration.
