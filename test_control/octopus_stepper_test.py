#!/usr/bin/env python3
"""
BTT Octopus stepper test.

Sends G-code over USB serial to exercise each configured motor back
and forth a few times. Assumes the board is running Marlin or
RepRapFirmware. (Klipper does not accept raw G-code on the USB port the
same way; use Moonraker's HTTP API instead.)

Usage:
    pip install pyserial
    python octopus_stepper_test.py --port /dev/ttyACM0
    python octopus_stepper_test.py --port COM5 --axes X,Y,Z --distance 10 --cycles 2
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")


def find_port():
    """Best-effort auto-detect of the Octopus USB CDC port."""
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "0483" in hwid:                  # STMicroelectronics VID
            return p.device
        if any(k in desc for k in ("stm32", "octopus", "usb serial", "cdc")):
            return p.device
    return None


class Board:
    def __init__(self, port, baud=115200, timeout=2.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2.0)                     # board resets when USB opens
        self.ser.reset_input_buffer()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # Lines from kinematic firmwares (FPM, polar, SCARA, etc.) that flood the
    # console but aren't meaningful for our test. We still capture them, just
    # don't print them.
    NOISE_PREFIXES = ("FPM IK", "echo:DEBUG")

    def send(self, cmd, wait_ok=True, log=True, timeout=30.0, quiet=False):
        if log:
            print(f">> {cmd}")
        self.ser.write((cmd.strip() + "\n").encode())
        self.ser.flush()
        if not wait_ok:
            return []
        replies = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="replace").strip()
            if not line:
                continue
            replies.append(line)
            is_noise = quiet and any(line.startswith(p) for p in self.NOISE_PREFIXES)
            if log and not is_noise:
                print(f"<< {line}")
            low = line.lower()
            if low.startswith("ok") or low.startswith("error"):
                return replies
        print("!! timeout waiting for ok/error")
        return replies


def parse_m114_counts(replies):
    """
    Extract raw stepper step counts from an M114 response.
    Marlin reports them like:
        X:0.00 Y:0.00 Z:0.00 A:-360.00 Count X:-55445 Y:0 Z:-3200 A:-32000
    We parse everything after the literal 'Count'.
    """
    for line in replies:
        if "Count" in line:
            after = line.split("Count", 1)[1]
            counts = {}
            for part in after.split():
                if ":" in part:
                    name, val = part.split(":", 1)
                    try:
                        counts[name] = int(val)
                    except ValueError:
                        pass
            return counts
    return {}


def diagnose_xy(board, span, feedrate, quiet=True):
    """
    Read step counters, drive XY through a small pattern, read counters
    again, and report deltas. This tells you whether the firmware actually
    emitted step pulses for each motor — even if nothing visibly moved.
    """
    print("\n--- XY step-count diagnostic ---")
    before = parse_m114_counts(board.send("M114", quiet=quiet))
    print(f"  initial counts: {before}")

    pattern = [(span, 0), (0, span), (-span, 0), (0, -span), (0, 0)]
    for (x, y) in pattern:
        board.send(f"G0 X{x:.2f} Y{y:.2f} F{feedrate}", quiet=quiet)
    board.send("M400", quiet=quiet, timeout=120.0)

    after = parse_m114_counts(board.send("M114", quiet=quiet))
    print(f"  final counts:   {after}")

    print("\n  step deltas (firmware-issued pulses, regardless of motion):")
    for axis in sorted(set(before) | set(after)):
        b = before.get(axis, 0)
        a = after.get(axis, 0)
        d = a - b
        verdict = "STEPPED" if d != 0 else "no pulses"
        print(f"    {axis}: {b:>10d} -> {a:>10d}   Δ {d:+d}   [{verdict}]")


def test_xy_waypoints(board, waypoints, feedrate, cycles, quiet=True):
    """
    Drive XY through a series of absolute waypoints. On kinematic firmware
    (FPM, SCARA, polar, etc.) this is the only way to reliably force the IK
    solver to compute *different* joint angles and therefore actually step
    the motors. Relative jogs from a single point can produce zero step
    pulses if the IK considers them degenerate.
    """
    print(f"\n--- XY waypoint sweep ({len(waypoints)} points, {cycles} cycles) ---")
    for c in range(cycles):
        print(f"  cycle {c+1}/{cycles}")
        for (x, y) in waypoints:
            print(f"    -> X{x:.2f} Y{y:.2f}")
            board.send(f"G0 X{x:.2f} Y{y:.2f} F{feedrate}", quiet=quiet)
            board.send("M400", quiet=quiet)


def test_direct_axis(board, axis, distance, feedrate, cycles, quiet=True):
    """
    Drive a single direct-drive axis (e.g. A) which on most kinematic Marlin
    builds is NOT routed through the IK transform. Use this to step one
    motor at a time independent of kinematics — but only works if a driver
    is actually populated in the MOTOR slot Marlin maps that axis to.
    """
    print(f"\n--- Direct axis {axis} (kinematics bypass) ---")
    for i in range(cycles):
        print(f"  cycle {i+1}/{cycles}: {axis} +{distance}")
        board.send(f"G0 {axis}{distance:.2f} F{feedrate}", quiet=quiet)
        board.send("M400", quiet=quiet)
        print(f"  cycle {i+1}/{cycles}: {axis} -{distance}")
        board.send(f"G0 {axis}-{distance:.2f} F{feedrate}", quiet=quiet)
        board.send("M400", quiet=quiet)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/tty.usbmodem397B398A30311",
                    help="serial port (default is this user's known Octopus port)")
    ap.add_argument("--baud", type=int, default=250000,
                    help="serial baud (Marlin on Octopus is usually 250000; "
                         "try 115200 if this gives garbage)")
    ap.add_argument("--span", type=float, default=30.0,
                    help="half-extent of the XY waypoint sweep, in firmware units (mm)")
    ap.add_argument("--feedrate", type=int, default=600,
                    help="mm/min (drop to 300 for first run)")
    ap.add_argument("--cycles", type=int, default=2,
                    help="how many times to repeat the waypoint pattern")
    ap.add_argument("--current", type=int, default=800,
                    help="run current in mA for X/Y drivers via M906")
    ap.add_argument("--test-a", type=float, default=0.0,
                    help="if >0, also exercise direct-drive A axis by this many "
                         "units back and forth (kinematics bypass)")
    ap.add_argument("--test-z", type=float, default=0.0,
                    help="if >0, also exercise Z axis by this many mm back and "
                         "forth (Z is typically direct-drive on kinematic builds)")
    ap.add_argument("--diagnose", action="store_true",
                    help="read M114 step counts before and after a small XY "
                         "pattern, report per-motor step deltas. Use this to "
                         "verify the firmware is issuing step pulses for X and "
                         "Y even on a kinematic build where you can't bypass IK.")
    ap.add_argument("--skip-xy", action="store_true",
                    help="skip the XY waypoint sweep (useful when only testing "
                         "Z or A with --test-z / --test-a)")
    ap.add_argument("--verbose", action="store_true",
                    help="show ALL firmware output including FPM IK debug lines")
    args = ap.parse_args()

    quiet = not args.verbose
    s = args.span
    # A diamond of waypoints around (0,0): forces IK to compute clearly different
    # joint angles between each move, which on a kinematic build is what actually
    # produces step pulses.
    waypoints = [
        ( s,  0),
        ( 0,  s),
        (-s,  0),
        ( 0, -s),
        ( s,  s),
        (-s, -s),
        ( 0,  0),
    ]

    port = args.port
    print(f"Opening {port} @ {args.baud}")
    board = Board(port, args.baud)

    try:
        # Drain anything the board printed on reset
        time.sleep(0.5)
        while board.ser.in_waiting:
            print(f"<< {board.ser.readline().decode(errors='replace').strip()}")

        # Init
        board.send("M111 S0")               # silence Marlin debug output
        board.send("M115")                  # firmware report
        board.send("M211 S0")               # disable software endstops
        board.send(f"M906 X{args.current} Y{args.current}")  # set TMC2209 current
        board.send("G90")                   # ABSOLUTE positioning (key change)
        board.send("G92 X0 Y0 Z0")          # call current pose the origin
        board.send("M17")                   # explicitly energize steppers

        # Diagnostic first if requested
        if args.diagnose:
            diagnose_xy(board, span=min(args.span, 20.0),
                        feedrate=args.feedrate, quiet=quiet)

        # The actual XY exercise
        if not args.skip_xy:
            test_xy_waypoints(board, waypoints, args.feedrate, args.cycles, quiet=quiet)

        # Optional Z test (direct-drive on most kinematic builds)
        if args.test_z > 0:
            test_direct_axis(board, "Z", args.test_z, args.feedrate, args.cycles, quiet=quiet)

        # Optional direct-axis test (only useful if a driver is plugged into A's slot)
        if args.test_a > 0:
            test_direct_axis(board, "A", args.test_a, args.feedrate, args.cycles, quiet=quiet)

        # Return to origin and release
        board.send("G0 X0 Y0 F600", quiet=quiet)
        board.send("M400", quiet=quiet)
        board.send("M84")
        print("\nDone.")
    except KeyboardInterrupt:
        print("\nInterrupted — disabling steppers.")
        board.send("M84", log=False)
    finally:
        board.close()


if __name__ == "__main__":
    main()