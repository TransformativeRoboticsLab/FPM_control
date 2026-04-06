import argparse
import csv
import glob
import os
import webbrowser

import plotly.graph_objects as go


def latest_csv(pattern):
    matches = glob.glob(pattern)
    if not matches:
        raise FileNotFoundError(f"No files matched: {pattern}")
    return max(matches, key=os.path.getmtime)


def load_probe_data(csv_path):
    hits = []
    starts = []

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            status = (row.get("status") or "").strip().upper()
            if not status.startswith("OK"):
                continue

            sx = float(row["start_x"])
            sy = float(row["start_y"])
            sz = float(row["start_z"])
            hx = float(row["hit_x"])
            hy = float(row["hit_y"])
            hz = float(row["hit_z"])

            starts.append((sx, sy, sz))
            hits.append((hx, hy, hz))

    if not hits:
        raise ValueError(f"No successful probe hits found in {csv_path}")

    return starts, hits


def build_figure(starts, hits, title):
    sx = [p[0] for p in starts]
    sy = [p[1] for p in starts]
    sz = [p[2] for p in starts]

    hx = [p[0] for p in hits]
    hy = [p[1] for p in hits]
    hz = [p[2] for p in hits]

    fig = go.Figure()

    fig.add_trace(
        go.Scatter3d(
            x=sx,
            y=sy,
            z=sz,
            mode="markers",
            name="Hemisphere Start Points",
            marker={"size": 3, "color": "lightgray", "opacity": 0.8},
        )
    )

    fig.add_trace(
        go.Scatter3d(
            x=hx,
            y=hy,
            z=hz,
            mode="markers",
            name="Probe Hits",
            marker={
                "size": 5,
                "color": hz,
                "colorscale": "Viridis",
                "colorbar": {"title": "Hit Z"},
            },
        )
    )

    # Draw probe vectors from each start point to each hit point.
    lx = []
    ly = []
    lz = []
    for s, h in zip(starts, hits):
        lx.extend([s[0], h[0], None])
        ly.extend([s[1], h[1], None])
        lz.extend([s[2], h[2], None])

    fig.add_trace(
        go.Scatter3d(
            x=lx,
            y=ly,
            z=lz,
            mode="lines",
            name="Probe Vectors",
            line={"width": 2, "color": "rgba(80,80,80,0.45)"},
            hoverinfo="skip",
        )
    )

    fig.add_trace(
        go.Scatter3d(
            x=[0.0],
            y=[0.0],
            z=[0.0],
            mode="markers",
            name="Origin",
            marker={"size": 7, "color": "red", "symbol": "diamond"},
        )
    )

    fig.update_layout(
        title=title,
        scene={
            "xaxis_title": "X (mm)",
            "yaxis_title": "Y (mm)",
            "zaxis_title": "Z (mm)",
            "aspectmode": "data",
        },
        template="plotly_white",
        legend={"orientation": "h", "y": 1.02, "x": 0},
        margin={"l": 0, "r": 0, "t": 50, "b": 0},
    )
    return fig


def main():
    parser = argparse.ArgumentParser(
        description="Render an interactive 3D HTML plot from radial probe CSV data."
    )
    parser.add_argument(
        "--csv",
        default=None,
        help="Path to a radial CSV. If omitted, the newest radial_probe_a_*.csv is used.",
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
        help="Generate HTML but do not open it in a browser.",
    )
    args = parser.parse_args()

    csv_path = args.csv or latest_csv("radial_probe_a_*.csv")
    starts, hits = load_probe_data(csv_path)

    output_html = os.path.splitext(csv_path)[0] + "_interactive.html"
    title = f"Interactive Radial Probe Render - {os.path.basename(csv_path)}"
    fig = build_figure(starts, hits, title)
    fig.write_html(output_html, include_plotlyjs="cdn")

    abs_html = os.path.abspath(output_html)
    print(f"Wrote interactive plot: {abs_html}")

    if not args.no_open:
        webbrowser.open(f"file://{abs_html}")
        print("Opened in browser.")


if __name__ == "__main__":
    main()