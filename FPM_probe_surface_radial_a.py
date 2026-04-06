import csv
import math
import re
import time
from datetime import datetime

import serial

# -- config -------------------------------------------------------------------

PORT = "/dev/pts/9"
BAUD = 115200

ORIGIN_X = 0.0
ORIGIN_Y = 0.0
ORIGIN_Z = 0.0

HEMISPHERE_DIAMETER = 50.0  # mm (configured to include apex point at Z=50)
PROBE_COUNT = 200            # total probes

SAFE_Z = 50.0              # mm
INTER_PROBE_Z = 50.0        # mm, used before each probe after the first
TRAVEL_FEED = 10000          # mm/min
ROTATE_FEED = 10000          # deg/min (or controller units for A)
PROBE_FEED = 10000            # mm/min

PROBE_TIMEOUT = 45          # s
SETTLE_TIME = 0.2           # s
A_POSITIVE_CLOCKWISE = True  # set False if your controller uses CCW-positive angles

UNCAP_MOTION_LIMITS = True
UNCAP_COMMANDS = [
    # Max feedrate per axis (units/min). Tune if your controller rejects these values.
    "M203 X100000 Y100000 Z100000 A100000",
    # Max acceleration per axis (units/s^2).
    "M201 X100000 Y100000 Z100000 A100000",
    # Printing/travel acceleration (Marlin-style).
    "M204 P100000 T100000",
    # Jerk / junction tuning (Marlin-style).
    "M205 X1000 Y1000 Z1000 A1000",
]

TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_FILE = f"radial_probe_a_{TIMESTAMP}.csv"


# -- serial helpers ------------------------------------------------------------

_M114_POS_RE = re.compile(
    r"\bX:\s*(-?\d+(?:\.\d+)?)\s+Y:\s*(-?\d+(?:\.\d+)?)\s+Z:\s*(-?\d+(?:\.\d+)?)",
    re.I,
)


def _read_until_ok(ser, timeout_s=30.0):
    lines = []
    deadline = time.time() + timeout_s

    while True:
        if time.time() >= deadline:
            raise TimeoutError(f"Timeout waiting for ok. Last lines: {lines[-3:]}")

        raw = ser.readline().decode(errors="ignore").strip()
        if not raw:
            continue

        lines.append(raw)
        lowered = raw.lower()
        if lowered.startswith("error"):
            raise RuntimeError(f"Firmware error: {raw}")
        if lowered.startswith("ok"):
            return "\n".join(lines)


def send(ser, cmd, timeout_s=30.0):
    print(f">> {cmd}")
    ser.write((cmd.strip() + "\n").encode())
    ser.flush()
    return _read_until_ok(ser, timeout_s=timeout_s)


def query_position(ser):
    resp = send(ser, "M114", timeout_s=5)
    m = _M114_POS_RE.search(resp)
    if not m:
        raise ValueError(f"Could not parse M114 response: {resp!r}")
    return tuple(float(v) for v in m.groups())


def apply_motion_uncap(ser):
    if not UNCAP_MOTION_LIMITS:
        return

    print("Applying high motion limits...")
    for cmd in UNCAP_COMMANDS:
        try:
            send(ser, cmd, timeout_s=5)
        except (RuntimeError, TimeoutError) as exc:
            # Some firmwares or builds may reject specific tuning commands.
            print(f"  Skipping unsupported limit command '{cmd}': {exc}")


# -- path planning -------------------------------------------------------------

def normalize_angle_deg(angle):
    return angle % 360.0


def probe_heading_deg(start_x, start_y, fallback_deg=0.0):
    dx = ORIGIN_X - start_x
    dy = ORIGIN_Y - start_y
    if math.hypot(dx, dy) <= 1e-9:
        # At the apex directly above origin, XY heading is undefined.
        return normalize_angle_deg(fallback_deg)
    heading_ccw = math.degrees(math.atan2(dy, dx))
    if A_POSITIVE_CLOCKWISE:
        return normalize_angle_deg(-heading_ccw)
    return normalize_angle_deg(heading_ccw)


def hemisphere_points(count, radius_mm):
    """
    Generate approximately uniform points on an upper hemisphere (z >= 0),
    centered at (ORIGIN_X, ORIGIN_Y, ORIGIN_Z).
    """
    points = []
    if count < 1:
        return points

    # Ensure the exact apex is part of the set, e.g. (0, 0, 50).
    points.append((ORIGIN_X, ORIGIN_Y, ORIGIN_Z + radius_mm))

    if count == 1:
        return points

    golden_angle = math.pi * (3.0 - math.sqrt(5.0))

    for k in range(1, count):
        # Start near the top of the hemisphere and work downward.
        z_unit = 1.0 - ((k + 0.5) / count)  # (1, 0)
        r_xy_unit = math.sqrt(max(0.0, 1.0 - z_unit * z_unit))
        theta = k * golden_angle

        x = ORIGIN_X + radius_mm * r_xy_unit * math.cos(theta)
        y = ORIGIN_Y + radius_mm * r_xy_unit * math.sin(theta)
        z = ORIGIN_Z + radius_mm * z_unit
        points.append((x, y, z))

    return points


# -- main ----------------------------------------------------------------------

def main():
    radius = HEMISPHERE_DIAMETER
    points = hemisphere_points(PROBE_COUNT, radius)

    print("Radial probe test with A-axis orientation")
    print(f"Port={PORT}  Baud={BAUD}")
    print(f"Origin=({ORIGIN_X:.3f}, {ORIGIN_Y:.3f}, {ORIGIN_Z:.3f})")
    print(f"Hemisphere diameter={HEMISPHERE_DIAMETER:.1f} mm  probes={PROBE_COUNT}")
    print(f"Results file: {CSV_FILE}\n")

    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(2.5)
        ser.reset_input_buffer()
        ser.read_all()

        send(ser, "G90")
        apply_motion_uncap(ser)

        print("Homing all axes (G28)...")
        send(ser, "G28", timeout_s=120)

        print(f"Moving to Z{SAFE_Z:.3f} and waiting for confirmation...")
        send(ser, f"G1 Z{SAFE_Z:.3f} F{TRAVEL_FEED}")

        input("Confirm clearance and probe setup, then press Enter to continue... ")

        with open(CSV_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "index",
                    "start_x",
                    "start_y",
                    "start_z",
                    "a_heading_deg",
                    "hit_x",
                    "hit_y",
                    "hit_z",
                    "status",
                ]
            )

            successes = 0
            last_a_heading = 0.0

            for idx, (sx, sy, sz) in enumerate(points, start=1):
                a_heading = probe_heading_deg(sx, sy, fallback_deg=last_a_heading)
                last_a_heading = a_heading
                print(
                    f"\n[{idx:>2}/{PROBE_COUNT}] "
                    f"start=({sx:.3f}, {sy:.3f}, {sz:.3f}) "
                    f"A={a_heading:.2f} deg"
                )

                if idx > 1:
                    send(ser, f"G1 Z{INTER_PROBE_Z:.3f} F{TRAVEL_FEED}")
                send(ser, f"G1 A{a_heading:.3f} F{ROTATE_FEED}")
                send(ser, f"G1 X{sx:.3f} Y{sy:.3f} Z{sz:.3f} F{TRAVEL_FEED}")
                send(ser, "M400")
                time.sleep(SETTLE_TIME)
                # Reassert heading right before probing so A is guaranteed aligned.
                send(ser, f"G1 A{a_heading:.3f} F{ROTATE_FEED}")
                send(ser, "M400")

                status = "OK"
                hit_x = float("nan")
                hit_y = float("nan")
                hit_z = float("nan")

                try:
                    send(
                        ser,
                        (
                            f"G38.2 X{ORIGIN_X:.3f} "
                            f"Y{ORIGIN_Y:.3f} "
                            f"Z{ORIGIN_Z:.3f} "
                            f"F{PROBE_FEED}"
                        ),
                        timeout_s=PROBE_TIMEOUT,
                    )
                    hit_x, hit_y, hit_z = query_position(ser)
                    successes += 1
                    print(
                        f"    Triggered at X{hit_x:.3f} Y{hit_y:.3f} Z{hit_z:.3f}"
                    )
                except (RuntimeError, TimeoutError, ValueError) as exc:
                    status = f"FAIL: {exc}"
                    print(f"    Probe failed: {exc}")

                # Retract back out along the same probe vector to the start point.
                try:
                    send(ser, f"G1 A{a_heading:.3f} F{ROTATE_FEED}")
                    send(ser, f"G1 X{sx:.3f} Y{sy:.3f} Z{sz:.3f} F{TRAVEL_FEED}")
                    send(ser, "M400")
                except (RuntimeError, TimeoutError) as exc:
                    print(f"    Retract failed: {exc}")
                    if status == "OK":
                        status = f"FAIL: retract {exc}"
                    else:
                        status = f"{status}; retract {exc}"

                writer.writerow(
                    [
                        idx,
                        round(sx, 4),
                        round(sy, 4),
                        round(sz, 4),
                        round(a_heading, 3),
                        hit_x,
                        hit_y,
                        hit_z,
                        status,
                    ]
                )

        print("\nParking at safe Z and origin XY...")
        send(ser, f"G1 Z{SAFE_Z:.3f} F{TRAVEL_FEED}")
        send(ser, f"G1 X{ORIGIN_X:.3f} Y{ORIGIN_Y:.3f} F{TRAVEL_FEED}")
        send(ser, "M400")

    print("\nDone.")
    print(f"Successful probes: {successes}/{PROBE_COUNT}")
    print(f"CSV written: {CSV_FILE}")


if __name__ == "__main__":
    main()
