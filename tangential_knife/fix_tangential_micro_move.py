#!/usr/bin/env python3
"""
Tangential knife micro-move pre-adjustment for UCCNC.

UCCNC in tangential knife mode automatically calculates the A-axis position
based on the XY movement direction, but it does NOT re-orient before Z-only
moves.  Direct A-axis commands (G0/G1 A…) are not allowed in tangential mode.

Workaround: before every G0 Z-only move, insert a tiny (0.001 mm) XY step
in the direction of the NEXT real movement.  This causes UCCNC to rotate the
knife to the correct angle before the plunge/retract.

Supported move types after Z:
  - G0  (rapid)      → micro-step in XY direction
  - G1  (linear cut) → micro-step in XY direction
  - G2  (CW arc)     → micro-step along tangent at arc start
  - G3  (CCW arc)    → micro-step along tangent at arc start

Modal G-codes are tracked so bare "X… Y…" lines inherit the last G0-G3.

Usage:
    python fix_tangential_micro_move.py                   # process all .gc in input/
    python fix_tangential_micro_move.py somefile.gc       # process a specific file
"""

import re
import math
import os
import sys
import glob

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INPUT_DIR = os.path.join(SCRIPT_DIR, "input")
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")

# Size of the micro-step in mm
MICRO_STEP = 0.001

# Motion G-codes whose mode is "sticky" (modal group 1)
MODAL_MOTION_CODES = {0, 1, 2, 3}


def parse_gcode_params(line):
    """
    Parse G-code letter/value pairs from a line (ignoring comments).
    Returns a dict like {'G': 0.0, 'X': 50.0, 'Z': -5.5}.
    Multiple G words on the same line are collected into a list under 'G_ALL'.
    The last motion G-code (0/1/2/3) found is stored under 'G'.
    """
    code_part = line.split(';')[0].strip()
    params = {}
    g_codes = []
    for m in re.finditer(r'([A-Za-z])\s*(-?\d*\.?\d+)', code_part):
        letter = m.group(1).upper()
        value = float(m.group(2))
        if letter == 'G':
            g_codes.append(value)
            if value in MODAL_MOTION_CODES:
                params['G'] = value
        else:
            params[letter] = value
    params['G_ALL'] = g_codes
    return params


def is_g0_z_only(line):
    """
    Return True if the line is a G0 move with a negative Z and NO X/Y.
    Only plunge moves (Z < 0) need knife orientation; retracts do not.
    """
    params = parse_gcode_params(line)
    if params.get('G') != 0:
        return False
    if 'Z' not in params or params['Z'] >= 0:
        return False
    if 'X' in params or 'Y' in params:
        return False
    return True


def _angle_deg(dx, dy):
    """Return angle in [0, 360) degrees from dx, dy."""
    a = math.degrees(math.atan2(dy, dx))
    return a % 360


def _arc_start_tangent(g_code, i_val, j_val):
    """
    Compute the tangent direction at the START of an arc.

    I, J are the offsets from the arc start point to the arc center,
    so the radius vector from center → start = (-I, -J).

    G2 (CW):  tangent = radius rotated −90°  → (-J,  I)
    G3 (CCW): tangent = radius rotated +90°  → ( J, -I)
    """
    if g_code == 2:
        return (-j_val, i_val)
    else:  # g_code == 3
        return (j_val, -i_val)


def _arc_start_tangent_from_r(g_code, dx, dy, r_val):
    """
    Compute tangent direction at arc start when only R (radius) is given.

    dx, dy: relative endpoint offsets.
    r_val:  signed radius (positive = minor arc, negative = major arc).

    We reconstruct I, J from R and the chord, then delegate to
    _arc_start_tangent().
    """
    d = math.hypot(dx, dy)
    if d == 0 or abs(r_val) < d / 2:
        # Degenerate — fall back to chord direction
        return (dx, dy)

    # Half-chord to perpendicular distance
    h = math.sqrt(r_val * r_val - (d / 2) ** 2)

    # Unit vectors: along chord and perpendicular
    ux, uy = dx / d, dy / d
    # Choose perpendicular side based on sign(R) and CW/CCW
    if (r_val > 0) == (g_code == 2):
        px, py = uy, -ux   # right-hand perpendicular
    else:
        px, py = -uy, ux   # left-hand perpendicular

    # Center offsets (I, J) from start
    i_val = dx / 2 + h * px
    j_val = dy / 2 + h * py

    return _arc_start_tangent(g_code, i_val, j_val)


def find_next_move_direction(lines, start_idx):
    """
    Scan forward from start_idx to find the first line with an XY (or arc)
    movement.  Returns (dx, dy) unit direction vector, or None.

    Tracks modal G-code state so bare coordinate lines are handled.
    """
    modal_g = None  # last active motion mode

    for i in range(start_idx, len(lines)):
        params = parse_gcode_params(lines[i])

        # Update modal motion code if a new one appears
        if 'G' in params:
            modal_g = int(params['G'])

        has_xy = ('X' in params and params['X'] != 0) or \
                 ('Y' in params and params['Y'] != 0)
        has_ij = 'I' in params or 'J' in params
        has_r = 'R' in params

        if not has_xy and not has_ij:
            continue  # no movement on this line

        dx = params.get('X', 0)
        dy = params.get('Y', 0)

        # G0 / G1 — linear moves
        if modal_g in (0, 1, None):
            if dx != 0 or dy != 0:
                mag = math.hypot(dx, dy)
                return (dx / mag, dy / mag)

        # G2 / G3 — arcs
        elif modal_g in (2, 3):
            if has_ij:
                i_val = params.get('I', 0)
                j_val = params.get('J', 0)
                tx, ty = _arc_start_tangent(modal_g, i_val, j_val)
            elif has_r:
                r_val = params['R']
                tx, ty = _arc_start_tangent_from_r(modal_g, dx, dy, r_val)
            else:
                # Arc with only XY and no center info — use chord as fallback
                tx, ty = dx, dy

            if tx != 0 or ty != 0:
                mag = math.hypot(tx, ty)
                return (tx / mag, ty / mag)

    return None


def _format_coord(value):
    """Format a coordinate value, stripping unnecessary trailing zeros."""
    s = f"{value:.4f}"
    # Strip trailing zeros but keep at least one decimal place
    s = s.rstrip('0').rstrip('.')
    return s


def process_gcode(input_path, output_path):
    """Read a G-code file, insert micro-move adjustments, write result."""
    with open(input_path, 'r') as f:
        lines = f.readlines()

    output_lines = []
    adjustments = 0

    for i, line in enumerate(lines):
        if is_g0_z_only(line.strip()):
            direction = find_next_move_direction(lines, i + 1)
            if direction is not None:
                ux, uy = direction

                # Extract the Z value from the original line
                z_params = parse_gcode_params(line.strip())
                z_val = z_params['Z']

                # Build combined line: micro-move XY + Z on same G0 line
                # Use a fixed 0.001 on each axis that participates in the move
                parts = ["G0"]
                if abs(ux) > 1e-9:
                    parts.append(f"X{_format_coord(math.copysign(MICRO_STEP, ux))}")
                if abs(uy) > 1e-9:
                    parts.append(f"Y{_format_coord(math.copysign(MICRO_STEP, uy))}")
                parts.append(f"Z{_format_coord(z_val)}")

                comment = " (tangential micro-move to orient knife)"
                combined_line = " ".join(parts) + comment + "\n"
                output_lines.append(combined_line)
                adjustments += 1
            else:
                output_lines.append(line)
        else:
            output_lines.append(line)

    with open(output_path, 'w') as f:
        f.writelines(output_lines)

    print(f"  {os.path.basename(input_path)} -> {os.path.basename(output_path)}"
          f"  ({adjustments} micro-move(s) inserted)")


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Allow specifying files on the command line, otherwise process all .gc
    if len(sys.argv) > 1:
        input_files = []
        for arg in sys.argv[1:]:
            if os.path.isabs(arg):
                candidates = [arg]
            else:
                candidates = [arg, os.path.join(INPUT_DIR, arg)]
            found = False
            for path in candidates:
                if os.path.isfile(path):
                    input_files.append(os.path.abspath(path))
                    found = True
                    break
            if not found:
                print(f"Warning: file not found: {arg}")
    else:
        input_files = sorted(glob.glob(os.path.join(INPUT_DIR, "*.gc")))

    if not input_files:
        print(f"No .gc files found in {INPUT_DIR}")
        sys.exit(1)

    print(f"Processing {len(input_files)} file(s)...")
    for input_path in input_files:
        filename = os.path.basename(input_path)
        output_path = os.path.join(OUTPUT_DIR, filename)
        process_gcode(input_path, output_path)

    print("Done.")


if __name__ == "__main__":
    main()
