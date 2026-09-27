# Generate G-code in the style of jyjblrd/Radial_Non_Planar_Slicer
# (4 axis moves "G1 C.. X.. Z.. B.. E.. F<inverse time>").
import math


def generate(rings=6, moves_per_ring=100, speed=30.0, start_r=12.0,
             ring_step=0.45, nozzle_offset=43.0):
    """Returns (gcode_text, final_position, total_g93_time).

    final_position is (X, C, Z, B) of the last move and total_g93_time
    the sum of the requested inverse time durations (seconds)."""
    out = [
        "G94 ; mm/min feed",
        "M83 ; relative extrusion",
        "G90 ; absolute positioning",
        "G0 C0 X0 Z20 B-15.0 F6000",
        "G93 ; inverse time feed",
    ]
    total_t = 0.0
    theta = 0.0
    prev_a = 0.0
    pos = None
    prev_xy = None
    for ring in range(rings):
        r0 = start_r + ring * ring_step
        tilt = 15.0 + 0.4 * ring
        z0 = -1.2 - 0.05 * ring
        for i in range(moves_per_ring + 1):
            # Cartesian point on a slightly wobbly ring
            a = -2.0 * math.pi * i / moves_per_ring
            rr = r0 + 0.02 * math.sin(5 * a)
            x, y = rr * math.cos(a), rr * math.sin(a)
            rot = math.radians(tilt)
            r = rr + math.sin(rot) * nozzle_offset
            z = z0 + (math.cos(rot) - 1) * nozzle_offset + 0.01 * math.cos(a)
            # continuous (unwrapped) bed angle, like the radial slicer
            dtheta = (math.degrees(a - prev_a) + 180.0) % 360.0 - 180.0
            theta += dtheta
            prev_a = a
            b = -tilt
            if i == 0:
                # travel to the start of the ring
                out.append("G94")
                out.append("G0 C%.5f X%.5f Z%.5f B%.5f F50000"
                           % (theta, r, z + 2.0, b))
                out.append("G1 C%.5f X%.5f Z%.5f B%.5f F3000"
                           % (theta, r, z, b))
                out.append("G93")
                prev_xy = (x, y)
                pos = (r, theta, z, b)
                continue
            seg = math.hypot(x - prev_xy[0], y - prev_xy[1])
            t_move = seg / speed
            total_t += t_move
            e = seg * 0.4 * 0.2 / (math.pi * 0.875 ** 2)
            out.append("G1 C%.5f X%.5f Z%.5f B%.5f E%.4f F%.4f"
                       % (theta, r, z, b, e, 60.0 / t_move))
            prev_xy = (x, y)
            pos = (r, theta, z, b)
    out.append("G94")
    return "\n".join(out) + "\n", pos, total_t
