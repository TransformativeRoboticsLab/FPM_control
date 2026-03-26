import serial
import time
import csv
import re
from dataclasses import dataclass, fields, astuple
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from scipy.interpolate import griddata

# ── config ────────────────────────────────────────────────────────────────────

PORT         = "/dev/tty.usbmodem2048327E55531"
BAUD         = 115200
OUTPUT_FILE  = f"probe_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

START_X      = 0.0      # mm — where to begin the sweep
START_Y      = 0.0
SAFE_Z       = 10.0     # mm — Z height to travel between probe points
PROBE_Z_MIN  = 0.0      # mm — lowest Z to descend before giving up
PROBE_FEED   = 100      # mm/min — how fast to descend during probe
TRAVEL_FEED  = 5000     # mm/min — XY travel speed

# Grid definition — 50mm cube workspace
WORKSPACE    = 50.0                              # mm — cube side length
GRID_STEPS   = 6                                 # points per axis (6 → ~10mm spacing)
X_POINTS     = list(np.linspace(0, WORKSPACE, GRID_STEPS))
Y_POINTS     = list(np.linspace(0, WORKSPACE, GRID_STEPS))

Z_STEP       = 0.1      # mm — increment per probe step down
SETTLE_TIME  = 0.1      # seconds — pause after each Z step before reading endstop

# ── data ──────────────────────────────────────────────────────────────────────

@dataclass
class ProbeResult:
    timestamp:      float
    x:              float
    y:              float
    z_commanded:    float
    x_actual:       float
    y_actual:       float
    z_actual:       float
    z_endstop:      str   # "open" | "TRIGGERED"
    triggered:      bool

# ── serial helpers ─────────────────────────────────────────────────────────────

def send(ser: serial.Serial, cmd: str, delay: float = 0.05) -> str:
    ser.write((cmd.strip() + "\n").encode())
    time.sleep(delay)
    return ser.read_all().decode(errors="ignore").strip()

def wait_for_ok(ser: serial.Serial, timeout: float = 30) -> str:
    """Accumulate response until 'ok' appears. Returns full response buffer."""
    start = time.time()
    buf = ""
    while time.time() - start < timeout:
        buf += ser.read_all().decode(errors="ignore")
        if "ok" in buf.lower():
            return buf
        time.sleep(0.05)
    raise TimeoutError(f"No 'ok' within {timeout}s — last buffer: {buf!r}")

# ── position parsing ───────────────────────────────────────────────────────────

_POS_RE = re.compile(
    r"X:\s*([\d.+-]+)\s+Y:\s*([\d.+-]+)\s+Z:\s*([\d.+-]+)", re.IGNORECASE
)

def get_position(ser: serial.Serial) -> tuple[float, float, float]:
    """Send M114, parse and return (x, y, z)."""
    send(ser, "M400")           # flush motion queue first
    wait_for_ok(ser)
    raw = send(ser, "M114", delay=0.15)
    wait_for_ok(ser)
    m = _POS_RE.search(raw)
    if not m:
        raise ValueError(f"Could not parse M114 response: {raw!r}")
    return float(m.group(1)), float(m.group(2)), float(m.group(3))

# ── endstop parsing ────────────────────────────────────────────────────────────

_ZEND_RE = re.compile(r"z_min:\s*(\S+)", re.IGNORECASE)

def get_z_endstop(ser: serial.Serial) -> str:
    """Send M119, return z_min state string ('open' or 'TRIGGERED')."""
    raw = send(ser, "M119", delay=0.15)
    wait_for_ok(ser)
    m = _ZEND_RE.search(raw)
    if not m:
        raise ValueError(f"Could not parse M119 response: {raw!r}")
    return m.group(1).strip()

# ── probe routine ──────────────────────────────────────────────────────────────

def probe_point(
    ser: serial.Serial,
    x: float,
    y: float,
    writer: csv.writer,
    t0: float,
    all_readings: list[ProbeResult],
) -> ProbeResult | None:
    """
    Travel to (x, y) at SAFE_Z, then descend in Z_STEP increments.
    Record endstop state at each step. Stop when triggered or PROBE_Z_MIN reached.
    Returns the ProbeResult at trigger (or None if never triggered).
    """
    # travel to position at safe height
    send(ser, f"G1 Z{SAFE_Z} F{TRAVEL_FEED}")
    wait_for_ok(ser)
    send(ser, f"G1 X{x} Y{y} F{TRAVEL_FEED}")
    wait_for_ok(ser)
    send(ser, "M400")
    wait_for_ok(ser)

    z = SAFE_Z
    triggered_result = None

    while z >= PROBE_Z_MIN:
        # step down
        send(ser, f"G1 Z{z:.3f} F{PROBE_FEED}")
        wait_for_ok(ser)
        time.sleep(SETTLE_TIME)

        # read state
        endstop = get_z_endstop(ser)
        ax, ay, az = get_position(ser)
        triggered = endstop.lower() != "open"
        ts = time.time() - t0

        result = ProbeResult(
            timestamp=round(ts, 3),
            x=x, y=y, z_commanded=round(z, 3),
            x_actual=ax, y_actual=ay, z_actual=az,
            z_endstop=endstop,
            triggered=triggered,
        )

        all_readings.append(result)
        writer.writerow(astuple(result))
        print(
            f"  t={ts:6.2f}s  X{ax:7.3f} Y{ay:7.3f} Z{az:7.3f}  "
            f"endstop={endstop:<12}{'  ← TRIGGERED' if triggered else ''}"
        )

        if triggered:
            triggered_result = result
            break

        z = round(z - Z_STEP, 4)

    # retract to safe Z before next move
    send(ser, f"G1 Z{SAFE_Z} F{TRAVEL_FEED}")
    wait_for_ok(ser)

    if not triggered_result:
        print(f"  !! No trigger at X{x} Y{y} — reached Z floor ({PROBE_Z_MIN}mm)")

    return triggered_result

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(2)
        ser.read_all()  # flush startup noise

        print("Absolute positioning...")
        send(ser, "G90")
        wait_for_ok(ser)

        print(f"Moving to start position X{START_X} Y{START_Y} Z{SAFE_Z}...")
        send(ser, f"G1 Z{SAFE_Z} F{TRAVEL_FEED}")
        wait_for_ok(ser)
        send(ser, f"G1 X{START_X} Y{START_Y} F{TRAVEL_FEED}")
        wait_for_ok(ser)

        grid = [(x, y) for y in Y_POINTS for x in X_POINTS]
        print(f"\nProbing {len(grid)} points. Writing to {OUTPUT_FILE}\n")

        t0 = time.time()
        triggered_points = []

        with open(OUTPUT_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([field.name for field in fields(ProbeResult)])

            for i, (x, y) in enumerate(grid):
                print(f"[{i+1}/{len(grid)}] Probing X{x} Y{y}")
                result = probe_point(ser, x, y, writer, t0)
                if result:
                    triggered_points.append(result)

        # summary
        print(f"\n── Summary ──────────────────────────────")
        print(f"Total points probed : {len(grid)}")
        print(f"Total readings      : {len(all_readings)}")
        print(f"Triggered           : {len(triggered_points)}")
        if triggered_points:
            zvals = [r.z_actual for r in triggered_points]
            print(f"Z trigger range     : {min(zvals):.3f} – {max(zvals):.3f} mm")
            print(f"Z variation         : {max(zvals) - min(zvals):.3f} mm")
        print(f"Output file         : {OUTPUT_FILE}")

        # park
        send(ser, f"G1 Z{SAFE_Z} F{TRAVEL_FEED}")
        send(ser, f"G1 X0 Y0 F{TRAVEL_FEED}")
        wait_for_ok(ser)
        print("Done — head parked at origin.")

# ── 3D plot ───────────────────────────────────────────────────────────────────

def plot_results(all_readings: list[ProbeResult], triggered: list[ProbeResult]):
    xs = np.array([r.x_actual for r in all_readings])
    ys = np.array([r.y_actual for r in all_readings])
    zs = np.array([r.z_actual for r in all_readings])

    fig = plt.figure(figsize=(14, 6))
    fig.suptitle("Workspace Probe — 50mm³", fontsize=13, fontweight="bold")

    # ── left: motion path coloured by Z height ────────────────────────────────
    ax1 = fig.add_subplot(121, projection="3d")
    ax1.set_title("Motion path (colour = Z height)")

    # build line segments so we can colour by Z continuously
    pts = np.array([xs, ys, zs]).T.reshape(-1, 1, 3)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    norm = plt.Normalize(zs.min(), zs.max())
    lc = Line3DCollection(segs, cmap="plasma", norm=norm, linewidth=0.8, alpha=0.7)
    lc.set_array(zs[:-1])
    ax1.add_collection3d(lc)
    fig.colorbar(lc, ax=ax1, shrink=0.5, label="Z actual (mm)")

    # overlay trigger points
    if triggered:
        tx = [r.x_actual for r in triggered]
        ty = [r.y_actual for r in triggered]
        tz = [r.z_actual for r in triggered]
        ax1.scatter(tx, ty, tz, c="cyan", s=40, zorder=5, label="Trigger")
        ax1.legend(loc="upper left", fontsize=8)

    ax1.set_xlabel("X (mm)"); ax1.set_ylabel("Y (mm)"); ax1.set_zlabel("Z (mm)")
    ax1.set_xlim(0, WORKSPACE); ax1.set_ylim(0, WORKSPACE)
    ax1.set_zlim(max(0, zs.min() - 2), SAFE_Z + 2)

    # ── right: interpolated surface map of trigger heights ────────────────────
    ax2 = fig.add_subplot(122, projection="3d")
    ax2.set_title("Surface map (trigger Z per grid point)")

    if len(triggered) >= 4:
        tx = np.array([r.x_actual for r in triggered])
        ty = np.array([r.y_actual for r in triggered])
        tz = np.array([r.z_actual for r in triggered])

        xi = np.linspace(0, WORKSPACE, 60)
        yi = np.linspace(0, WORKSPACE, 60)
        xi, yi = np.meshgrid(xi, yi)
        zi = griddata((tx, ty), tz, (xi, yi), method="cubic")

        surf = ax2.plot_surface(
            xi, yi, zi,
            cmap="viridis", edgecolor="none", alpha=0.85
        )
        fig.colorbar(surf, ax=ax2, shrink=0.5, label="Trigger Z (mm)")
        ax2.scatter(tx, ty, tz, c="white", s=20, zorder=5)
    else:
        ax2.text(
            WORKSPACE / 2, WORKSPACE / 2, SAFE_Z / 2,
            "Not enough\ntrigger points\nto interpolate",
            ha="center", va="center", fontsize=10, color="gray"
        )

    ax2.set_xlabel("X (mm)"); ax2.set_ylabel("Y (mm)"); ax2.set_zlabel("Trigger Z (mm)")
    ax2.set_xlim(0, WORKSPACE); ax2.set_ylim(0, WORKSPACE)

    plt.tight_layout()
    plot_file = OUTPUT_FILE.replace(".csv", ".png")
    plt.savefig(plot_file, dpi=150, bbox_inches="tight")
    print(f"Plot saved to {plot_file}")
    plt.show()


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(2)
        ser.read_all()

        print("Homing all axes...")
        send(ser, "G28")
        wait_for_ok(ser, timeout=120)

        print("Absolute positioning...")
        send(ser, "G90")
        wait_for_ok(ser)

        print(f"Moving to start position X{START_X} Y{START_Y} Z{SAFE_Z}...")
        send(ser, f"G1 Z{SAFE_Z} F{TRAVEL_FEED}")
        wait_for_ok(ser)
        send(ser, f"G1 X{START_X} Y{START_Y} F{TRAVEL_FEED}")
        wait_for_ok(ser)

        grid = [(x, y) for y in Y_POINTS for x in X_POINTS]
        print(f"\nProbing {len(grid)} points. Writing to {OUTPUT_FILE}\n")

        t0 = time.time()
        all_readings     = []
        triggered_points = []

        with open(OUTPUT_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([field.name for field in fields(ProbeResult)])

            for i, (x, y) in enumerate(grid):
                print(f"[{i+1}/{len(grid)}] Probing X{x:.1f} Y{y:.1f}")
                result = probe_point(ser, x, y, writer, t0, all_readings)
                if result:
                    triggered_points.append(result)

        # summary
        print(f"\n── Summary ──────────────────────────────")
        print(f"Total points probed : {len(grid)}")
        print(f"Total readings      : {len(all_readings)}")
        print(f"Triggered           : {len(triggered_points)}")
        if triggered_points:
            zvals = [r.z_actual for r in triggered_points]
            print(f"Z trigger range     : {min(zvals):.3f} – {max(zvals):.3f} mm")
            print(f"Z variation         : {max(zvals) - min(zvals):.3f} mm")
        print(f"Output file         : {OUTPUT_FILE}")

        # park
        send(ser, f"G1 Z{SAFE_Z} F{TRAVEL_FEED}")
        send(ser, f"G1 X0 Y0 F{TRAVEL_FEED}")
        wait_for_ok(ser)
        print("Done — head parked at origin.")

    plot_results(all_readings, triggered_points)


if __name__ == "__main__":
    main()