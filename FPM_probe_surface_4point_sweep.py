import csv
import os
import re
import time
from datetime import datetime

import numpy as np
import serial

# ── config ────────────────────────────────────────────────────────────────────

PORT = "COM5"
BAUD = 115200

SAFE_Z = 10.0
PROBE_START = 5.0

TRAVEL_FEED_SWEEP_START = 5000
TRAVEL_FEED_SWEEP_END = 20000
TRAVEL_FEED_SWEEP_COUNT = 5

PROBE_FEED_SWEEP_START = 80
PROBE_FEED_SWEEP_END = 300
PROBE_FEED_SWEEP_COUNT = 5

PROBE_POINTS = [
    (0.0, 0.0),
    (20.0, 0.0),
    (0.0, 20.0),
    (20.0, 20.0),
]
REPEATS_PER_SETTING = 3

TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
RESULTS_DIR = f"probe4_sweep_{TIMESTAMP}"

# ── serial helpers ─────────────────────────────────────────────────────────────

_PAUSED_RE = re.compile(r"paused for user", re.I)
_M114_POS_RE = re.compile(
    r"\bX:\s*(-?\d+(?:\.\d+)?)\s+Y:\s*(-?\d+(?:\.\d+)?)\s+Z:\s*(-?\d+(?:\.\d+)?)",
    re.I,
)


class PauseStateError(RuntimeError):
    """Raised when firmware reports it is paused and awaiting user action."""


def _read_until_ok(ser, timeout_s=None, log_rx=False):
    lines = []
    deadline = None if timeout_s is None else (time.time() + timeout_s)

    while True:
        if deadline is not None and time.time() >= deadline:
            raise TimeoutError(f"Timeout waiting for 'ok' (last lines: {lines[-3:]})")

        raw = ser.readline().decode(errors="ignore").strip()
        if not raw:
            continue

        lines.append(raw)
        if log_rx:
            print(f"    [rx] {raw}", flush=True)

        lowered = raw.lower()
        if _PAUSED_RE.search(lowered):
            raise PauseStateError(raw)
        if lowered.startswith("error"):
            raise RuntimeError(f"Firmware error: {raw}")
        if lowered.startswith("ok"):
            return "\n".join(lines)


def send(ser, cmd, timeout_s=None, log_rx=False, auto_unpause=True):
    for attempt in range(2):
        ser.write((cmd.strip() + "\n").encode())
        ser.flush()
        try:
            return _read_until_ok(ser, timeout_s=timeout_s, log_rx=log_rx)
        except PauseStateError as exc:
            if not auto_unpause or attempt == 1:
                raise RuntimeError(f"Paused while executing '{cmd}': {exc}") from exc

            print(
                f"    Firmware paused during '{cmd}'. Attempting auto-resume...",
                flush=True,
            )
            clear_pause_state(ser)

    raise RuntimeError(f"Command '{cmd}' failed after auto-resume attempts")


def send_and_drain(ser, cmd, timeout_s=None):
    return send(ser, cmd, timeout_s=timeout_s, log_rx=True)


def clear_pause_state(ser):
    for cmd in ("M108", "M24", "M876 S0"):
        try:
            resp = send(ser, cmd, log_rx=True, auto_unpause=False)
        except (TimeoutError, RuntimeError):
            continue
        if resp and not _PAUSED_RE.search(resp):
            print("    Pause state cleared.", flush=True)
            return resp

    print("    Pause state persists after resume attempts.", flush=True)
    return ""


def parse_m114_position(response):
    match = _M114_POS_RE.search(response or "")
    if not match:
        return None
    return tuple(float(v) for v in match.groups())


def query_m114_position(ser, attempts=3):
    for _ in range(attempts):
        try:
            resp = send_and_drain(ser, "M114")
        except (TimeoutError, RuntimeError):
            continue
        pos = parse_m114_position(resp)
        if pos is not None:
            return pos
        time.sleep(0.1)
    return None


def build_feed_values(start, end, count):
    return [int(round(v)) for v in np.linspace(start, end, count)]


def probe_point(ser, x, y, travel_feed, probe_feed):
    """Move to XY at PROBE_START, then probe with G30 and read M114."""
    send(ser, f"G1 X{x:.3f} Y{y:.3f} Z{PROBE_START:.3f} F{travel_feed}")
    send(ser, f"G1 F{probe_feed}")
    send(ser, "M400")

    try:
        send_and_drain(ser, "G30")
    except (TimeoutError, RuntimeError) as exc:
        print(f"    Probe failed at X{x:.3f} Y{y:.3f}: {exc}", flush=True)
        clear_pause_state(ser)
        return None

    pos = query_m114_position(ser)
    send(ser, f"G1 Z{SAFE_Z:.3f} F{travel_feed}")
    return pos


def run_setting(ser, out_dir, phase, setting_index, travel_feed, probe_feed):
    """Run 3 repeats of the 4-point square and write one CSV for this setting."""
    file_name = (
        f"{phase}_{setting_index:02d}_" f"tf{travel_feed:05d}_pf{probe_feed:04d}.csv"
    )
    file_path = os.path.join(out_dir, file_name)

    print(
        f"\n{phase} setting {setting_index}: "
        f"travel_feed={travel_feed}, probe_feed={probe_feed}"
    )
    print(f"  Writing {file_name}")

    with open(file_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "phase",
                "setting_index",
                "repeat_index",
                "point_index",
                "x_target",
                "y_target",
                "travel_feed",
                "probe_feed",
                "x_m114",
                "y_m114",
                "z_m114",
                "status",
            ]
        )

        for repeat_idx in range(1, REPEATS_PER_SETTING + 1):
            print(f"  Repeat {repeat_idx}/{REPEATS_PER_SETTING}")
            for point_idx, (x, y) in enumerate(PROBE_POINTS, start=1):
                print(
                    f"    P{point_idx} target X{x:.1f} Y{y:.1f} -> ",
                    end="",
                    flush=True,
                )
                pos = probe_point(ser, x, y, travel_feed, probe_feed)

                if pos is not None:
                    px, py, pz = pos
                    print(f"M114 X={px:.3f} Y={py:.3f} Z={pz:.3f}")
                    writer.writerow(
                        [
                            phase,
                            setting_index,
                            repeat_idx,
                            point_idx,
                            x,
                            y,
                            travel_feed,
                            probe_feed,
                            px,
                            py,
                            pz,
                            "OK",
                        ]
                    )
                else:
                    print("FAIL")
                    writer.writerow(
                        [
                            phase,
                            setting_index,
                            repeat_idx,
                            point_idx,
                            x,
                            y,
                            travel_feed,
                            probe_feed,
                            "",
                            "",
                            "",
                            "FAIL",
                        ]
                    )


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    travel_values = build_feed_values(
        TRAVEL_FEED_SWEEP_START,
        TRAVEL_FEED_SWEEP_END,
        TRAVEL_FEED_SWEEP_COUNT,
    )
    probe_values = build_feed_values(
        PROBE_FEED_SWEEP_START,
        PROBE_FEED_SWEEP_END,
        PROBE_FEED_SWEEP_COUNT,
    )

    min_travel = travel_values[0]
    min_probe = probe_values[0]

    total_files = len(travel_values) + len(probe_values)
    probes_per_file = REPEATS_PER_SETTING * len(PROBE_POINTS)

    print("4-point feed sweep probe")
    print(f"Port={PORT} Baud={BAUD}")
    print(f"Results dir: {RESULTS_DIR}")
    print(f"Square points: {PROBE_POINTS}")
    print(f"Per file probes: {probes_per_file}")
    print(f"Expected CSV files: {total_files}\n")

    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(3)
        ser.reset_input_buffer()
        ser.read_all()

        clear_pause_state(ser)
        send_and_drain(ser, "G90")

        # 1) Home all axes, then move to safe Z.
        send_and_drain(ser, "G28")
        send(ser, f"G1 Z{SAFE_Z:.3f} F{min_travel}")

        # 2) Set both feeds to minimum.
        send(ser, f"G1 F{min_travel}")
        send(ser, f"G1 F{min_probe}")

        # 3-5) Sweep travel feed only (probe feed fixed at min).
        for setting_index, travel_feed in enumerate(travel_values, start=1):
            run_setting(
                ser,
                RESULTS_DIR,
                "travel_sweep",
                setting_index,
                travel_feed,
                min_probe,
            )

        # 2-5 again) Reset to mins, then sweep probe feed only.
        send(ser, f"G1 F{min_travel}")
        send(ser, f"G1 F{min_probe}")

        for setting_index, probe_feed in enumerate(probe_values, start=1):
            run_setting(
                ser,
                RESULTS_DIR,
                "probe_sweep",
                setting_index,
                min_travel,
                probe_feed,
            )

        send(ser, f"G1 Z{SAFE_Z:.3f} F{min_travel}")
        send(ser, f"G1 X0.000 Y0.000 F{min_travel}")

    print("\nDone.")
    print(f"CSV folder: {RESULTS_DIR}")


if __name__ == "__main__":
    main()
