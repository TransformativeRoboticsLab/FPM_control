import serial
import time
import csv
import numpy as np
from datetime import datetime

# ── config ────────────────────────────────────────────────────────────────────

PORT        = "/dev/tty.usbmodem2048327E55531"
BAUD        = 115200

# Origin — where the tool currently sits when the script starts.
START_X     = 0.0    # mm
START_Y     = 0.0    # mm
START_Z     = 10.0   # mm

X_POINTS    = [0, 25, 50, 75, 100]   # mm
Y_POINTS    = [0, 25, 50, 75, 100]   # mm

SAFE_Z      = 10.0   # mm — travel height; we stay here the whole sweep
TRAVEL_FEED = 30000  # mm/min — default travel speed (used for the initial Z lift and parking)

# Sweep travel speed across runs so you can see how feed rate affects motion.
TRAVEL_FEED_SWEEP_START = 30000
TRAVEL_FEED_SWEEP_END   = 100000
TRAVEL_FEED_SWEEP_COUNT = 20

TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_FILE  = f"travel_{TIMESTAMP}.csv"

# ── serial helpers ────────────────────────────────────────────────────────────

def wait_ok(ser, timeout=10):
    """Block until firmware sends 'ok' or we time out."""
    start = time.time()
    buf = ""
    while time.time() - start < timeout:
        chunk = ser.read_all().decode(errors="ignore")
        if chunk:
            buf += chunk
            if "ok" in buf.lower():
                return buf
        time.sleep(0.05)
    print(f"    [warn] wait_ok timed out after {timeout}s", flush=True)
    return buf


def send_wait(ser, cmd, timeout=30):
    """Send command and block until 'ok'."""
    ser.write((cmd.strip() + "\n").encode())
    ser.flush()
    return wait_ok(ser, timeout=timeout)

# ── run plan ──────────────────────────────────────────────────────────────────

def build_run_plan():
    """One run per travel feed value — no probing."""
    runs = []
    travel_feeds = np.linspace(
        TRAVEL_FEED_SWEEP_START,
        TRAVEL_FEED_SWEEP_END,
        TRAVEL_FEED_SWEEP_COUNT,
    )
    for travel_feed in travel_feeds:
        runs.append({
            "phase": "travel_feed_sweep",
            "travel_feed": int(round(travel_feed)),
        })
    return runs

# ── motion ────────────────────────────────────────────────────────────────────

def move_xy(ser, x, y, travel_feed):
    """Swift XY move at the given feed. Z is left alone."""
    send_wait(ser, f"G1 X{x:.3f} Y{y:.3f} F{travel_feed}")
    send_wait(ser, "M400")   # wait for motion to complete

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    grid = [(x, y) for y in Y_POINTS for x in X_POINTS]
    run_plan = build_run_plan()
    points_per_run = len(grid)
    total = points_per_run * len(run_plan)

    print(f"XY traverse — {total} moves over "
          f"X{min(X_POINTS)}–{max(X_POINTS)} Y{min(Y_POINTS)}–{max(Y_POINTS)}")
    print(f"Opening {PORT} at {BAUD} baud …\n")

    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(3)                       # let the board wake up
        ser.reset_input_buffer()
        ser.read_all()

        # absolute mode, define current position as origin
        send_wait(ser, "G90")
        send_wait(ser, f"G92 X{START_X:.3f} Y{START_Y:.3f} Z{START_Z:.3f}")
        print(f"Origin set → X{START_X} Y{START_Y} Z{START_Z}\n")

        # raise to safe Z once; stay there for the whole sweep
        send_wait(ser, f"G1 Z{SAFE_Z:.3f} F{TRAVEL_FEED}")

        with open(CSV_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "run_index",
                "phase",
                "travel_feed",
                "point_index",
                "x_target",
                "y_target",
                "elapsed_s",
            ])

            for run_idx, run in enumerate(run_plan, start=1):
                run_phase = run["phase"]
                run_travel_feed = run["travel_feed"]

                print(f"\nRun {run_idx}/{len(run_plan)}: {run_phase} "
                      f"(travel={run_travel_feed})")

                for point_idx, (x, y) in enumerate(grid, start=1):
                    overall_idx = (run_idx - 1) * points_per_run + point_idx
                    print(f"[{overall_idx}/{total}] X{x:.3f} Y{y:.3f} -> ",
                          end="", flush=True)

                    t0 = time.time()
                    move_xy(ser, x, y, run_travel_feed)
                    elapsed = time.time() - t0
                    print(f"OK ({elapsed:.2f}s)")

                    writer.writerow([
                        run_idx,
                        run_phase,
                        run_travel_feed,
                        point_idx,
                        x,
                        y,
                        f"{elapsed:.3f}",
                    ])
                    f.flush()

        # park
        send_wait(ser, f"G1 X0 Y0 F{TRAVEL_FEED}")

    print(f"\n── Done ──────────────────────────────")
    print(f"CSV : {CSV_FILE}")


if __name__ == "__main__":
    main()