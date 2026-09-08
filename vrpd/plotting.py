"""Route plots in the style of Fig. 8 of the paper."""

from __future__ import annotations

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .instance import Instance  # noqa: E402

TRUCK_COLORS = ["#2a9d8f", "#1d6f5f", "#43aa8b", "#0b5351", "#57cc99", "#137547"]
DRONE_COLORS = ["#f4a261", "#e76f51", "#f77f00", "#d62828", "#ffb703", "#fb8500",
                "#e85d04", "#dc2f02", "#faa307", "#f48c06"]


def plot_solution(inst: Instance, truck_routes, drone_routes, title: str, path: str) -> str:
    fig, ax = plt.subplots(figsize=(8, 7))
    pts = inst.points

    for k, r in enumerate(truck_routes):
        xs, ys = [pts[i][0] for i in r], [pts[i][1] for i in r]
        ax.plot(xs, ys, "-", color=TRUCK_COLORS[k % len(TRUCK_COLORS)], lw=2.0,
                label=f"Truck {k + 1}" if k < 6 else None, zorder=2)
    for d, r in enumerate(drone_routes):
        xs, ys = [pts[i][0] for i in r], [pts[i][1] for i in r]
        ax.plot(xs, ys, "--", color=DRONE_COLORS[d % len(DRONE_COLORS)], lw=1.8,
                label=f"Drone {d + 1}" if d < 10 else None, zorder=3)

    cx = [pts[i][0] for i in inst.customers]
    cy = [pts[i][1] for i in inst.customers]
    ax.scatter(cx, cy, s=32, c="#264653", zorder=4)
    for i in inst.customers:
        ax.annotate(str(i), (pts[i][0], pts[i][1]), textcoords="offset points", xytext=(3, 3), fontsize=7)
    ax.scatter([pts[0][0]], [pts[0][1]], s=140, marker="s", c="#d62828", zorder=5, label="Depot (0)")

    lo, hi = inst.params.range_coordinate
    ax.set_xlim(lo - 0.05, hi + 0.05)
    ax.set_ylim(lo - 0.05, hi + 0.05)
    ax.set_aspect("equal")
    ax.set_xlabel("x (km)")
    ax.set_ylabel("y (km)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8, frameon=False)
    fig.tight_layout()
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path
