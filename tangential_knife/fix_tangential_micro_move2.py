#!/usr/bin/env python3
"""
Tangential knife micro-move pre-adjustment for UCCNC (v2 — no G0 rapids).

Same micro-move approach as fix_tangential_micro_move.py, but additionally:
  - Converts ALL G0 (rapid) commands to G1 (linear) with explicit feedrates
  - Removes all comments from the G-code (except the header section)
  - Removes S (spindle/laser power) values from the G-code

Usage:
    python fix_tangential_micro_move2.py [options] [file ...]

Options:
    --fxy SPEED   Feedrate for XY rapid moves converted to G1 (default: 5000)
    --fz  SPEED   Feedrate for Z-axis moves (default: 200)

Examples:
    python fix_tangential_micro_move2.py                       # defaults
    python fix_tangential_micro_move2.py --fz 300 --fxy 8000
    python fix_tangential_micro_move2.py --fz 150 input/1.gc
"""

import re
import math
import os
import sys
import glob
import argparse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
INPUT_DIR = os.path.join(SCRIPT_DIR, "input")
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")

# Size of the micro-step in mm
MICRO_STEP = 0.001

# Default feedrates for converted G0 → G1 moves
DEFAULT_F_Z = 200    # mm/min for Z-axis movements
DEFAULT_F_XY = 5000  # mm/min for XY rapid movements

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
    # Also strip parenthesized comments
    code_part = re.sub(r'\([^)]*\)', '', code_part).strip()
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


def _has_g0(line):
    """Return True if the line contains a G0/G00 motion command."""
    code_part = line.split(';')[0]
    code_part = re.sub(r'\([^)]*\)', '', code_part)
    return bool(re.search(r'\bG0*0\b', code_part))


def _replace_g0_with_g1(line, feedrate):
    """
    Replace G0/G00 with G1 in a line and append the feedrate.
    """
    # Replace G00 or G0 (but not G01, G02, etc.)
    new_line = re.sub(r'\bG0*0\b', 'G1', line.rstrip('\n'))
    # Check if line already has an F value
    code_part = new_line.split(';')[0]
    code_part = re.sub(r'\([^)]*\)', '', code_part)
    if not re.search(r'F\s*\d', code_part):
        new_line = new_line.rstrip() + f" F{feedrate}"
    return new_line + "\n"


def _line_has_z(line):
    """Return True if the line has a Z parameter."""
    params = parse_gcode_params(line)
    return 'Z' in params


def _is_header_line(line):
    """
    Return True if the line is a pure header comment (starts with ;).
    These are kept in the output.
    """
    return line.lstrip().startswith(';')


def _strip_comments(line):
    """
    Remove inline ;comments and (parenthesized comments) from a G-code line.
    """
    # Remove parenthesized comments
    line = re.sub(r'\s*\([^)]*\)', '', line)
    # Remove ; comments (everything after ;)
    if ';' in line:
        line = line[:line.index(';')]
    return line.rstrip() + '\n'


def _strip_s_values(line):
    """
    Remove S (spindle/laser power) values from a G-code line.
    """
    return re.sub(r'S-?\d*\.?\d+', '', line)


def process_gcode(input_path, output_path, f_xy, f_z):
    """Read a G-code file, insert micro-move adjustments, write result."""
    with open(input_path, 'r') as f:
        lines = f.readlines()

    output_lines = []
    adjustments = 0
    g0_conversions = 0
    header_done = False

    for i, line in enumerate(lines):
        # Detect end of header: first non-comment, non-blank line
        if not header_done:
            if _is_header_line(line) or line.strip() == '':
                output_lines.append(line)
                continue
            else:
                header_done = True

        # Skip pure comment lines (not in header)
        if _is_header_line(line):
            continue

        # Strip S values
        line = _strip_s_values(line)

        # Strip inline comments
        line = _strip_comments(line)

        # Skip lines that became empty after stripping
        if line.strip() == '':
            continue

        if is_g0_z_only(line.strip()):
            # Micro-move + plunge: build combined G1 line with f_z
            direction = find_next_move_direction(lines, i + 1)
            if direction is not None:
                ux, uy = direction

                # Extract the Z value from the original line
                z_params = parse_gcode_params(line.strip())
                z_val = z_params['Z']

                # Build combined line: micro-move XY + Z as G1 with Z feedrate
                parts = ["G1"]
                if abs(ux) > 1e-9:
                    parts.append(f"X{_format_coord(math.copysign(MICRO_STEP, ux))}")
                if abs(uy) > 1e-9:
                    parts.append(f"Y{_format_coord(math.copysign(MICRO_STEP, uy))}")
                parts.append(f"Z{_format_coord(z_val)}")
                parts.append(f"F{f_z}")

                combined_line = " ".join(parts) + "\n"
                output_lines.append(combined_line)
                adjustments += 1
            else:
                # No next move found — still convert G0 → G1 with f_z
                new_line = _replace_g0_with_g1(line, f_z)
                output_lines.append(new_line)
                g0_conversions += 1
        elif _has_g0(line.strip()):
            # Convert any other G0 to G1 with appropriate feedrate
            if _line_has_z(line.strip()):
                new_line = _replace_g0_with_g1(line, f_z)
            else:
                new_line = _replace_g0_with_g1(line, f_xy)
            output_lines.append(new_line)
            g0_conversions += 1
        else:
            output_lines.append(line)

    with open(output_path, 'w') as f:
        f.writelines(output_lines)

    print(f"  {os.path.basename(input_path)} -> {os.path.basename(output_path)}"
          f"  ({adjustments} micro-move(s), {g0_conversions} G0→G1 conversion(s))")


def main():
    parser = argparse.ArgumentParser(
        description='Tangential knife micro-move pre-adjustment for UCCNC (v2).')
    parser.add_argument('files', nargs='*',
                        help='G-code files to process (default: all .gc in input/)')
    parser.add_argument('--fxy', type=int, default=DEFAULT_F_XY,
                        help=f'Feedrate for XY rapid moves (default: {DEFAULT_F_XY})')
    parser.add_argument('--fz', type=int, default=DEFAULT_F_Z,
                        help=f'Feedrate for Z-axis moves (default: {DEFAULT_F_Z})')
    args = parser.parse_args()

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    if args.files:
        input_files = []
        for arg in args.files:
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

    print(f"Processing {len(input_files)} file(s) (FXY={args.fxy}, FZ={args.fz})...")
    for input_path in input_files:
        filename = os.path.basename(input_path)
        output_path = os.path.join(OUTPUT_DIR, filename)
        process_gcode(input_path, output_path, args.fxy, args.fz)

    print("Done.")


if __name__ == "__main__":
    main()
