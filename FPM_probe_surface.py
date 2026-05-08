import serial
import time
import csv
import re
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.interpolate import griddata
from datetime import datetime

# ── config ────────────────────────────────────────────────────────────────────

# PORT        = "/dev/tty.usbmodem2048327E55531"
PORT = "/dev/ttys001"  # edit as needed
BAUD        = 115200

# Origin — where the tool currently sits when the script starts.
# G92 will zero the coordinate system here.
START_X     = 0.0    # mm
START_Y     = 0.0    # mm
START_Z     = 10.0   # mm

X_POINTS    = [0, 5, 10, 15, 20]   # mm — edit freely
Y_POINTS    = [0, 5, 10, 15, 20]   # mm — edit freely

SAFE_Z      = 10.0   # mm — travel height between points
PROBE_START = 9.8    # mm — Z height to begin descending from
PROBE_Z_MIN = 5.0    # mm — hard floor; abort descent if reached
PROBE_FEED  = 2000     # mm/min — slow descent
TRAVEL_FEED = 3000   # mm/min
Z_STEP      = 0.01   # mm — resolution of each probe step
SETTLE_TIME = 0.3    # seconds — pause after each step before reading endstop

TIMESTAMP   = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_FILE    = f"surface_{TIMESTAMP}.csv"
PLOT_FILE   = f"surface_{TIMESTAMP}.png"

# ── serial helpers ─────────────────────────────────────────────────────────────

def send(ser, cmd, delay=0.1):
    """Send a G-code command and return whatever the board echoes back."""
    ser.write((cmd.strip() + "\n").encode())
    ser.flush()
    time.sleep(delay)
    return ser.read_all().decode(errors="ignore").strip()


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
    """Send command and block until 'ok'. Use for moves + M400."""
    ser.write((cmd.strip() + "\n").encode())
    ser.flush()
    return wait_ok(ser, timeout=timeout)


def read_endstop(ser, endstop="z_min"):
    """
    Query M119 and return True if the specified endstop is TRIGGERED.
    Marlin reports lines like:  z_min: TRIGGERED  /  z_min: open
    """
    ser.reset_input_buffer()
    ser.write(b"M119\n")
    ser.flush()
    time.sleep(0.3)
    resp = ser.read_all().decode(errors="ignore")
    # print(f"    [M119] {resp.strip()}", flush=True)
    # look for the specific endstop line
    for line in resp.lower().splitlines():
        if endstop in line:
            return "triggered" in line
    return False

# ── single-point probe using Z endstop ─────────────────────────────────────────

def probe_z(ser, x, y):
    """
    Move to (x, y) at SAFE_Z, then descend in Z_STEP increments.
    At each step, read M119 to check whether the Z endstop has triggered.
    Returns the Z coordinate at trigger, or None if PROBE_Z_MIN is reached.
    """
    # travel to position
    send_wait(ser, f"G1 Z{SAFE_Z:.3f} F{TRAVEL_FEED}")
    send_wait(ser, f"G1 X{x:.3f} Y{y:.3f} F{TRAVEL_FEED}")
    send_wait(ser, "M400")

    z = PROBE_START
    while z >= PROBE_Z_MIN:
        send_wait(ser, f"G1 Z{z:.3f} F{PROBE_FEED}")
        send_wait(ser, "M400")          # ensure move is complete
        time.sleep(SETTLE_TIME)

        if read_endstop(ser, endstop="z_probe"):  # some boards use z_probe for the probe trigger
            print(f"TRIGGERED at Z{z:.3f}", flush=True)
            send_wait(ser, f"G1 Z{SAFE_Z:.3f} F{TRAVEL_FEED}")
            return z

        z = round(z - Z_STEP, 4)

    # no trigger — retract and report failure
    send_wait(ser, f"G1 Z{SAFE_Z:.3f} F{TRAVEL_FEED}")
    return None


# ── plotting ──────────────────────────────────────────────────────────────────

def plot_surface(results):
    xs = np.array([r[0] for r in results])
    ys = np.array([r[1] for r in results])
    zs = np.array([r[2] for r in results])

    xi = np.linspace(xs.min(), xs.max(), 120)
    yi = np.linspace(ys.min(), ys.max(), 120)
    xi, yi = np.meshgrid(xi, yi)
    zi = griddata((xs, ys), zs, (xi, yi), method="cubic")

    z_range = zs.max() - zs.min()

    fig = plt.figure(figsize=(13, 5))
    fig.suptitle("Bed Surface Map", fontsize=13, fontweight="bold")

    # 3D surface
    ax3d = fig.add_subplot(121, projection="3d")
    surf = ax3d.plot_surface(xi, yi, zi, cmap="viridis", edgecolor="none", alpha=0.9)
    ax3d.scatter(xs, ys, zs, c="white", s=25, zorder=5, label="Samples")
    fig.colorbar(surf, ax=ax3d, shrink=0.5, pad=0.1, label="Z trigger (mm)")
    ax3d.set_xlabel("X (mm)")
    ax3d.set_ylabel("Y (mm)")
    ax3d.set_zlabel("Z (mm)")
    ax3d.set_title(f"3D surface  (Δ{z_range:.3f} mm)")
    ax3d.legend(fontsize=8)

    # 2D heatmap
    ax2d = fig.add_subplot(122)
    hm = ax2d.contourf(xi, yi, zi, levels=30, cmap="viridis")
    fig.colorbar(hm, ax=ax2d, label="Z trigger (mm)")
    ax2d.scatter(xs, ys, c="white", s=20, zorder=5)
    for xv, yv, zv in zip(xs, ys, zs):
        ax2d.annotate(
            f"{zv:.2f}", (xv, yv),
            textcoords="offset points", xytext=(4, 4),
            fontsize=7, color="white",
        )
    ax2d.set_xlabel("X (mm)")
    ax2d.set_ylabel("Y (mm)")
    ax2d.set_title("Top-down heatmap")
    ax2d.set_aspect("equal")

    plt.tight_layout()
    plt.savefig(PLOT_FILE, dpi=150, bbox_inches="tight")
    print(f"Plot saved → {PLOT_FILE}")
    plt.show()


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    grid   = [(x, y) for y in Y_POINTS for x in X_POINTS]
    total  = len(grid)
    hits   = []
    misses = []

    print(f"Surface map — {total} points over "
          f"X{min(X_POINTS)}–{max(X_POINTS)} Y{min(Y_POINTS)}–{max(Y_POINTS)}")
    print(f"Opening {PORT} at {BAUD} baud …\n")

    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(3)                       # let the board wake up
        ser.reset_input_buffer()
        ser.read_all()

        # set absolute mode and define current position as origin
        send_wait(ser, "G90")
        send_wait(ser, f"G92 X{START_X:.3f} Y{START_Y:.3f} Z{START_Z:.3f}")
        print(f"Origin set → X{START_X} Y{START_Y} Z{START_Z}\n")

        # sanity check: can we read the endstop?
        if read_endstop(ser, "z_probe"):
            print("[warn] Z endstop is already triggered — check wiring.\n")

        with open(CSV_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "z_trigger"])

            for i, (x, y) in enumerate(grid):
                print(f"[{i+1:>3}/{total}]  X{x:6.2f}  Y{y:6.2f}  →  ", end="", flush=True)
                z = probe_z(ser, x, y)

                if z is not None:
                    writer.writerow([x, y, z])
                    f.flush()
                    hits.append((x, y, z))
                else:
                    print(f"NO TRIGGER (floor at Z{PROBE_Z_MIN})")
                    misses.append((x, y))

        # park
        send_wait(ser, f"G1 Z{SAFE_Z:.3f} F{TRAVEL_FEED}")
        send_wait(ser, f"G1 X0 Y0 F{TRAVEL_FEED}")

    # summary
    print(f"\n── Results ──────────────────────────────")
    print(f"Triggered  : {len(hits)}/{total}")
    if misses:
        print(f"No trigger : {misses}")
    if hits:
        zv = [r[2] for r in hits]
        print(f"Z range    : {min(zv):.3f} – {max(zv):.3f} mm")
        print(f"Δ Z        : {max(zv) - min(zv):.3f} mm")
    print(f"CSV        : {CSV_FILE}")

    if len(hits) < 4:
        print("Not enough trigger points for a surface plot (need ≥ 4).")
        return

    plot_surface(hits)


if __name__ == "__main__":
    main()