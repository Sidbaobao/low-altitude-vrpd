"""Route plots in the style of Fig. 8 of the paper, drawn in a journal-style look (serif fonts,
boxed axes, restrained colours).  ``JOURNAL_RC`` is shared with ``scripts/make_figures.py`` so
every figure of the repository uses the same typography and frame."""

from __future__ import annotations

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .instance import Instance  # noqa: E402

#: matplotlib rcParams used by every figure in this repository.
JOURNAL_RC = {
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "Nimbus Roman", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 9, "axes.labelsize": 9.5, "axes.titlesize": 9.5,
    "xtick.labelsize": 8.5, "ytick.labelsize": 8.5, "legend.fontsize": 8.5,
    "axes.spines.top": True, "axes.spines.right": True,
    "axes.linewidth": 0.8, "axes.edgecolor": "black",
    "xtick.direction": "in", "ytick.direction": "in",
    "xtick.major.size": 3.5, "ytick.major.size": 3.5,
    "xtick.major.width": 0.8, "ytick.major.width": 0.8,
    "xtick.top": False, "ytick.right": False,
    "axes.grid": False, "axes.axisbelow": True,
    "grid.color": "#c8c8c8", "grid.linestyle": (0, (4, 3)), "grid.linewidth": 0.6,
    "lines.linewidth": 1.3, "lines.markersize": 5,
    "legend.frameon": True, "legend.fancybox": False, "legend.edgecolor": "black",
    "savefig.dpi": 300, "savefig.facecolor": "white",
    "pdf.fonttype": 42, "ps.fonttype": 42,
    "figure.constrained_layout.use": True,
}

# (colour, line style) per vehicle: trucks solid in dark hues, drones dashed / dash-dot in warm hues
TRUCK_STYLES = [("#000000", "-"), ("#1f4e79", "-"), ("#2e7d32", "-"), ("#6a3d9a", "-"),
                ("#7f4f24", "-"), ("#00695c", "-")]
DRONE_STYLES = [("#a51c1c", "--"), ("#d2691e", "--"), ("#b8860b", "--"), ("#c71585", "--"),
                ("#8b0000", "-."), ("#e07020", "-."), ("#9c6b00", "-."), ("#a0206b", "-."),
                ("#5c1010", (0, (1, 1.2))), ("#c04000", (0, (1, 1.2)))]


def plot_solution(inst: Instance, truck_routes, drone_routes, title: str, path: str) -> str:
    with plt.rc_context(JOURNAL_RC):
        fig, ax = plt.subplots(figsize=(6.2, 5.4))
        pts = inst.points

        for k, r in enumerate(truck_routes):
            color, ls = TRUCK_STYLES[k % len(TRUCK_STYLES)]
            xs, ys = [pts[i][0] for i in r], [pts[i][1] for i in r]
            ax.plot(xs, ys, color=color, ls=ls, lw=1.3, label=f"Truck {k + 1}", zorder=2)
        for d, r in enumerate(drone_routes):
            color, ls = DRONE_STYLES[d % len(DRONE_STYLES)]
            xs, ys = [pts[i][0] for i in r], [pts[i][1] for i in r]
            ax.plot(xs, ys, color=color, ls=ls, lw=1.1, label=f"Drone {d + 1}", zorder=3)

        cx = [pts[i][0] for i in inst.customers]
        cy = [pts[i][1] for i in inst.customers]
        ax.plot(cx, cy, ls="none", marker="o", markersize=4.5, markerfacecolor="white", markeredgecolor="black",
                markeredgewidth=0.8, zorder=4, label="Customer")
        for i in inst.customers:
            ax.annotate(str(i), (pts[i][0], pts[i][1]), textcoords="offset points", xytext=(3, 3), fontsize=7)
        ax.plot([pts[0][0]], [pts[0][1]], ls="none", marker="s", markersize=9, color="black", zorder=5,
                label="Depot (0)")

        lo, hi = inst.params.range_coordinate
        ax.set_xlim(lo - 0.05, hi + 0.05)
        ax.set_ylim(lo - 0.05, hi + 0.05)
        ax.set_aspect("equal")
        ax.set_xlabel("$x$ (km)")
        ax.set_ylabel("$y$ (km)")
        ax.set_title(title)
        ax.grid(True)
        ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8, borderaxespad=0.0)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fig.savefig(path)
        plt.close(fig)
    return path
