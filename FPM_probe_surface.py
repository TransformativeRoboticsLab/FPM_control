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

PORT = "COM5"
BAUD = 115200
USE_MANUAL_START = True
START_X = 0.0  # mm — current physical X when the script begins
START_Y = 0.0  # mm — current physical Y when the script begins
START_Z = 10.0  # mm — current physical Z when the script begins

X_POINTS = [0, 10, 20, 30, 40, 50]  # mm — edit freely
Y_POINTS = [0, 10, 20, 30, 40, 50]

SAFE_Z = 10.0  # mm — travel height between points
PROBE_START = 8.0  # mm — Z height to begin descending from
PROBE_Z_MIN = 0.2  # mm — hard floor; abort descent if reached
PROBE_FEED = 80  # mm/min — slow and steady
TRAVEL_FEED = 3000  # mm/min
Z_STEP = 0.15  # mm — resolution of each probe step
SETTLE_TIME = 1.0  # seconds — wait after each step before prompting

TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_FILE = f"surface_{TIMESTAMP}.csv"
PLOT_FILE = f"surface_{TIMESTAMP}.png"

# ── serial helpers ─────────────────────────────────────────────────────────────


def _read_until_ok(ser, timeout_s=None, log_rx=False):
    """Read serial lines until Marlin returns 'ok' or 'error'."""
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
    """Compatibility wrapper: send command and wait for final 'ok'."""
    return send(ser, cmd, timeout_s=timeout_s, log_rx=True)


_PAUSED_RE = re.compile(r"paused for user", re.I)


class PauseStateError(RuntimeError):
    """Raised when firmware reports it is paused and awaiting user action."""


_M114_POS_RE = re.compile(
    r"\bX:\s*(-?\d+(?:\.\d+)?)\s+Y:\s*(-?\d+(?:\.\d+)?)\s+Z:\s*(-?\d+(?:\.\d+)?)",
    re.I,
)


def clear_pause_state(ser):
    """Attempt to clear common firmware pause states before querying position."""
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
    """Extract (X, Y, Z) from a Marlin M114 response string."""
    match = _M114_POS_RE.search(response or "")
    if not match:
        return None
    return tuple(float(v) for v in match.groups())


def query_m114_position(ser, attempts=3):
    """Request M114 and retry a few times until XYZ is parseable."""
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


def initialize_position(ser):
    """Optionally seed the controller with a known starting coordinate."""
    if not USE_MANUAL_START:
        return
    send_and_drain(ser, f"G92 X{START_X:.3f} Y{START_Y:.3f} Z{START_Z:.3f}")
    print(
        f"Manual start position set to X{START_X:.3f} Y{START_Y:.3f} Z{START_Z:.3f}",
        flush=True,
    )


# ── single-point probe ─────────────────────────────────────────────────────────


def probe_z(ser, x, y):
    """
    Move to (x, y), run Marlin's built-in single probe (G30), then query M114.
    Returns the reported (x, y, z) position tuple, or None on failure.
    """
    send(ser, f"G1 Z{SAFE_Z} F{TRAVEL_FEED}")
    send(ser, f"G1 X{x:.3f} Y{y:.3f} F{TRAVEL_FEED}")
    send(ser, f"G1 Z{PROBE_START:.3f} F{TRAVEL_FEED}")
    send(ser, "M400")

    try:
        send_and_drain(ser, "G30")
    except (TimeoutError, RuntimeError) as exc:
        print(f"    Probe failed at X{x:.3f} Y{y:.3f}: {exc}", flush=True)
        clear_pause_state(ser)
        return None

    pos = query_m114_position(ser)
    send(ser, f"G1 Z{SAFE_Z} F{TRAVEL_FEED}")
    return pos


# ── plotting ──────────────────────────────────────────────────────────────────


def plot_surface(results):
    """
    results: list of (x, y, z) tuples where z is the trigger height.
    Renders a smooth interpolated surface + scatter of raw samples.
    """
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

    # ── 3D surface ────────────────────────────────────────────────────────────
    ax3d = fig.add_subplot(121, projection="3d")
    surf = ax3d.plot_surface(xi, yi, zi, cmap="viridis", edgecolor="none", alpha=0.9)
    ax3d.scatter(xs, ys, zs, c="white", s=25, zorder=5, label="Samples")
    fig.colorbar(surf, ax=ax3d, shrink=0.5, pad=0.1, label="Z trigger (mm)")
    ax3d.set_xlabel("X (mm)")
    ax3d.set_ylabel("Y (mm)")
    ax3d.set_zlabel("Z (mm)")
    ax3d.set_title(f"3D surface  (Δ{z_range:.3f} mm)")
    ax3d.legend(fontsize=8)

    # ── 2D heatmap ────────────────────────────────────────────────────────────
    ax2d = fig.add_subplot(122)
    hm = ax2d.contourf(xi, yi, zi, levels=30, cmap="viridis")
    fig.colorbar(hm, ax=ax2d, label="Z trigger (mm)")
    ax2d.scatter(xs, ys, c="white", s=20, zorder=5)

    # annotate each sample point with its Z value
    for x, y, z in zip(xs, ys, zs):
        ax2d.annotate(
            f"{z:.2f}",
            (x, y),
            textcoords="offset points",
            xytext=(4, 4),
            fontsize=7,
            color="white",
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
    grid = [(x, y) for y in Y_POINTS for x in X_POINTS]
    total = len(grid)
    hits = []
    misses = []

    print(
        f"Surface map — {total} points over "
        f"X{min(X_POINTS)}–{max(X_POINTS)} Y{min(Y_POINTS)}–{max(Y_POINTS)}"
    )
    print(f"Opening {PORT} at {BAUD} baud...\n")

    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(3)  # longer wake — boards can be slow to handshake
        ser.reset_input_buffer()
        ser.read_all()

        clear_pause_state(ser)
        send_and_drain(ser, "G90")  # script uses absolute coordinates
        send_and_drain(ser, "G28")  # home axes before probing
        initialize_position(ser)

        with open(CSV_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x_m114", "y_m114", "z_m114"])

            for i, (x, y) in enumerate(grid):
                print(
                    f"[{i+1:>3}/{total}]  X{x:6.2f}  Y{y:6.2f}  →  ", end="", flush=True
                )
                pos = probe_z(ser, x, y)

                if pos is not None:
                    px, py, pz = pos
                    print(f"M114 X={px:.3f} Y={py:.3f} Z={pz:.3f} mm")
                    writer.writerow([px, py, pz])
                    hits.append((px, py, pz))
                else:
                    print("PROBE FAILED (no parseable M114 position)")
                    misses.append((x, y))

        # park
        send(ser, f"G1 Z{SAFE_Z} F{TRAVEL_FEED}")
        send(ser, f"G1 X0 Y0 F{TRAVEL_FEED}")

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
        print("Not enough trigger points to plot surface (need ≥ 4).")
        return

    plot_surface(hits)


if __name__ == "__main__":
    main()
