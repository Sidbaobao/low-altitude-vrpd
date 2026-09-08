"""Analysis figures for the Shenzhen case study.

Reads the output of ``run_all.py`` (``results/routes.json``) and, if present,
``results/robustness.csv`` from ``scripts/robustness.py``, and writes to ``results/figures/``
(PNG 300 dpi + PDF):

    fig_sensitivity   ranking of the three groups as the objective weights / value of time vary
    fig_tradeoff      time-cost plane with iso-efficiency lines; decomposition of T and of cost
    fig_convergence   two-stage solution of the collaborative group: GA generations -> MILP
    fig_robustness    Z of the three groups on random instances (needs robustness.csv)

With ``--routes`` the three route maps in ``results/`` are re-drawn from ``routes.json`` as well.
The GA curve is regenerated deterministically (same seed as the run) instead of being stored.

    python scripts/make_figures.py
    python scripts/make_figures.py --skip-ga --skip-robustness
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from matplotlib.ticker import FuncFormatter, MultipleLocator, NullFormatter  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vrpd.config import DEFAULT, FLEETS  # noqa: E402
from vrpd.ga import Evaluator, run_ga, warm_start_seeds  # noqa: E402
from vrpd.instance import build_instance  # noqa: E402
from vrpd.metrics import evaluate  # noqa: E402
from vrpd.plotting import JOURNAL_RC, plot_solution  # noqa: E402

GROUPS = ("UAVRP", "CVRP", "VRPD")
# Restrained journal palette: the collaborative (proposed) mode in black, the two single-mode
# baselines in dark blue / dark red; identity is carried by line style and marker as well.
COLOR = {"VRPD": "#000000", "CVRP": "#1f4e79", "UAVRP": "#a51c1c"}
FILL = {"VRPD": "#404040", "CVRP": "#5b86b5", "UAVRP": "#c96a6a"}
LS = {"VRPD": "-", "CVRP": "--", "UAVRP": "-."}
MARKER = {"VRPD": "o", "CVRP": "s", "UAVRP": "^"}
GRID = "#c8c8c8"
LEGEND = dict(frameon=True, fancybox=False, edgecolor="black", framealpha=1.0, borderpad=0.5, handlelength=2.4)


def fleet_text(k, d):
    parts = [f"{k} truck{'s' if k != 1 else ''}" if k else "", f"{d} drone{'s' if d != 1 else ''}" if d else ""]
    return " + ".join(t for t in parts if t)


def label(g):
    return f"{g} ({fleet_text(*FLEETS[g])})"


def style():
    plt.rcParams.update(JOURNAL_RC)


def line_kw(g, hollow=None, ms=5.5):
    """Line + marker style of a group; baselines get hollow markers, the proposed mode filled."""
    if hollow is None:
        hollow = g != "VRPD"
    return dict(color=COLOR[g], ls=LS[g], marker=MARKER[g], markerfacecolor="white" if hollow else COLOR[g],
                markeredgecolor=COLOR[g], markeredgewidth=1.0, markersize=ms)


def ygrid(ax):
    ax.grid(axis="y")


def sublabel(ax, xlabel, tag):
    ax.set_xlabel(f"{xlabel}\n{tag}")


def save(fig, out, name):
    fig.savefig(os.path.join(out, name + ".png"))
    fig.savefig(os.path.join(out, name + ".pdf"))
    plt.close(fig)
    print(f"  wrote {name}.png / .pdf")


# ------------------------------------------------------------------------------ data

def load(res_dir):
    with open(os.path.join(res_dir, "routes.json"), encoding="utf-8") as f:
        routes = json.load(f)
    inst = build_instance(DEFAULT)
    metrics = {g: evaluate(inst, routes[g]["truck_routes"], routes[g]["drone_routes"], *FLEETS[g]) for g in GROUPS}
    for g in GROUPS:  # the stored solver objective must be what we recompute
        assert abs(metrics[g].efficiency - routes[g]["objective"]) < 1e-3, g
    return inst, routes, metrics


# ------------------------------------------------------------------------------ fig 1

def fig_sensitivity(metrics, p, out):
    T = {g: metrics[g].overall_time for g in GROUPS}
    C = {g: metrics[g].total_cost for g in GROUPS}

    def Z(g, wt, tv):
        return wt * tv * T[g] + (1 - wt) * C[g]

    fig, (a, b) = plt.subplots(1, 2, figsize=(7.0, 3.1))

    # (a) time weight sweep, value of time fixed at the paper's value
    w = np.linspace(0, 1, 501)
    wm = np.linspace(0, 1, 11)
    ygrid(a)
    for g in ("UAVRP", "CVRP"):
        kw = line_kw(g)
        a.plot(w, Z(g, w, p.time_value) - Z("VRPD", w, p.time_value), color=kw["color"], ls=kw["ls"], zorder=3)
        a.plot(wm, Z(g, wm, p.time_value) - Z("VRPD", wm, p.time_value), ls="none", zorder=4,
               **{k: v for k, v in kw.items() if k != "ls"})
    a.axhline(0, color="black", lw=1.3, zorder=3)
    dz_c = Z("CVRP", w, p.time_value) - Z("VRPD", w, p.time_value)
    wc = w[int(np.argmax(dz_c > 0))]
    a.axvline(wc, color="black", lw=0.7, ls=(0, (2, 2)))
    a.axvline(p.time_weight, color="black", lw=0.7, ls=(0, (6, 3)))
    a.text(wc - 0.015, -2.3, f"$w_t$ = {wc:.2f}", ha="right", va="bottom", fontsize=8)
    a.text(wc + 0.015, -2.3, "VRPD most\nefficient $\\rightarrow$", ha="left", va="bottom", fontsize=8)
    a.text(p.time_weight + 0.015, 4.9, "paper\n$w_t$ = 0.8", ha="left", va="top", fontsize=8)
    a.set_xlim(0, 1)
    a.set_ylim(-2.5, 8.2)
    a.xaxis.set_major_locator(MultipleLocator(0.2))
    sublabel(a, "Time weight $w_t$ (cost weight $1-w_t$)", "(a)")
    a.set_ylabel("$Z - Z_{\\mathrm{VRPD}}$ (10$^4$ CNY)")
    handles = [Line2D([], [], **line_kw(g)) if g != "VRPD" else Line2D([], [], color="black", lw=1.3)
               for g in GROUPS]
    a.legend(handles, [label(g) + (" (reference)" if g == "VRPD" else "") for g in GROUPS],
             loc="upper right", **LEGEND)

    # (b) value-of-time sweep, weights fixed at the paper's 0.8 / 0.2
    tv = np.geomspace(0.02, 0.5, 400)
    tvm = np.array([200, 300, 500, 700, 1000, 1500, 2000, 3000, 5000]) / 1e4
    ygrid(b)
    for g in ("UAVRP", "CVRP"):
        kw = line_kw(g)
        b.plot(tv * 1e4, Z(g, p.time_weight, tv) - Z("VRPD", p.time_weight, tv), color=kw["color"], ls=kw["ls"],
               zorder=3)
        b.plot(tvm * 1e4, Z(g, p.time_weight, tvm) - Z("VRPD", p.time_weight, tvm), ls="none", zorder=4,
               **{k: v for k, v in kw.items() if k != "ls"})
    b.axhline(0, color="black", lw=1.3, zorder=3)
    dz_cb = Z("CVRP", p.time_weight, tv) - Z("VRPD", p.time_weight, tv)
    tvc = tv[int(np.argmax(dz_cb > 0))] * 1e4
    b.axvline(tvc, color="black", lw=0.7, ls=(0, (2, 2)))
    b.axvline(p.time_value * 1e4, color="black", lw=0.7, ls=(0, (6, 3)))
    b.text(tvc * 1.04, -0.9, f"{tvc:,.0f} CNY/min", ha="left", va="bottom", fontsize=8)
    b.text(tvc * 1.04, -0.5, "VRPD most efficient $\\rightarrow$", ha="left", va="bottom", fontsize=8)
    b.text(p.time_value * 1e4 * 1.04, 4.0, "paper\n1,000 CNY/min", ha="left", va="top", fontsize=8)
    b.set_xscale("log")
    b.set_xlim(200, 5000)
    b.set_ylim(-1.0, 6.0)
    b.set_xticks([200, 500, 1000, 2000, 5000])
    b.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:,.0f}"))
    b.xaxis.set_minor_formatter(NullFormatter())
    sublabel(b, "Value of time (CNY per minute)", "(b)")
    b.set_ylabel("$Z - Z_{\\mathrm{VRPD}}$ (10$^4$ CNY)")
    b.legend(handles, [label(g) + (" (reference)" if g == "VRPD" else "") for g in GROUPS],
             loc="upper left", **LEGEND)

    fig.get_layout_engine().set(w_pad=0.08)
    save(fig, out, "fig_sensitivity")


# ------------------------------------------------------------------------------ fig 2

def fig_tradeoff(metrics, p, out):
    m = metrics
    T = {g: m[g].overall_time for g in GROUPS}
    C = {g: m[g].total_cost for g in GROUPS}
    Zv = {g: m[g].efficiency for g in GROUPS}
    wt, wc, tv = p.time_weight, p.cost_weight, p.time_value

    fig, (a, b, c) = plt.subplots(1, 3, figsize=(7.0, 3.0), gridspec_kw={"width_ratios": [1.2, 1.1, 0.9]})

    # (a) time-cost plane with iso-efficiency lines  Z = wt*tv*T + wc*C
    xlim, ylim = (256, 271), (47, 71)
    xs = np.array(xlim, dtype=float)
    lab = dict(fontsize=7.5, color="#555555", bbox=dict(boxstyle="square,pad=0.1", fc="white", ec="none"), zorder=2)
    for z in np.arange(55, 61):
        ys = (z - wc * xs) / (wt * tv)
        a.plot(xs, ys, color="#9a9a9a", lw=0.6, ls=(0, (4, 3)), zorder=1)
        # label where the line meets the top or the right edge
        x_top = (z - wt * tv * ylim[1]) / wc
        if xlim[0] + 0.3 <= x_top <= xlim[1] - 1.2:
            a.text(x_top + 0.25, ylim[1] - 0.6, f"$Z$ = {z}", ha="left", va="top", **lab)
        else:
            y_right = (z - wc * xlim[1]) / (wt * tv)
            x_bot = (z - wt * tv * ylim[0]) / wc
            if ylim[0] + 1.0 <= y_right <= ylim[1] - 1.0:
                a.text(xlim[1] - 0.25, y_right + 0.4, f"$Z$ = {z}", ha="right", va="bottom", **lab)
            elif xlim[0] + 1.0 <= x_bot <= xlim[1] - 1.0:
                a.text(x_bot + 0.25, ylim[0] + 0.5, f"$Z$ = {z}", ha="left", va="bottom", **lab)
    for g in GROUPS:
        kw = line_kw(g, ms=8)
        a.plot([C[g]], [T[g]], ls="none", zorder=5, **{k: v for k, v in kw.items() if k != "ls"})
    offs = {"UAVRP": (8, 0, "left", "center"), "CVRP": (-8, 0, "right", "center"), "VRPD": (8, -3, "left", "center")}
    for g in GROUPS:
        dx, dy, ha, va = offs[g]
        a.annotate(f"{g}\n$Z$ = {Zv[g]:.2f}", (C[g], T[g]), xytext=(dx, dy), textcoords="offset points",
                   fontsize=8, ha=ha, va=va, zorder=6)
    a.annotate("", xy=(257.0, 58.5), xytext=(258.6, 62.8),
               arrowprops=dict(arrowstyle="-|>", color="black", lw=0.8, mutation_scale=9))
    a.text(256.5, 48.3, "lower $Z$ = better", fontsize=8, ha="left", va="bottom")
    a.set_xlim(*xlim)
    a.set_ylim(*ylim)
    a.xaxis.set_major_locator(MultipleLocator(5))
    a.yaxis.set_major_locator(MultipleLocator(5))
    sublabel(a, "Total cost $C$ (10$^4$ CNY)", "(a)")
    a.set_ylabel("Total time $T$ (min)")

    # (b) decomposition of T: travel time (solid) + fixed loading / hand-over time (hatched)
    du = m["UAVRP"].drone_distance / p.drone_speed * 60
    tc = m["CVRP"].truck_distance / p.truck_speed * 60
    tv_t = m["VRPD"].truck_distance / p.truck_speed * 60
    tv_d = m["VRPD"].drone_distance / p.drone_speed * 60
    bars = [(0.0, "UAVRP", du, m["UAVRP"].drone_time - du, "UAVRP\n10 drones"),
            (1.0, "CVRP", tc, m["CVRP"].truck_time - tc, "CVRP\n5 trucks"),
            (2.3, "VRPD", tv_t, m["VRPD"].truck_time - tv_t, "VRPD\n4 trucks"),
            (3.25, "VRPD", tv_d, m["VRPD"].drone_time - tv_d, "VRPD\n2 drones")]
    width = 0.62
    ygrid(b)
    for x, g, trav, hand, _ in bars:
        b.bar(x, trav, width, color=FILL[g], edgecolor="black", lw=0.6, zorder=3)
        b.bar(x, hand, width, bottom=trav, color="white", edgecolor=COLOR[g], hatch="////", lw=0.6, zorder=3)
        b.text(x, trav + hand + 1.5, f"{trav + hand:.1f}", ha="center", va="bottom", fontsize=8)
    b.plot([2.3 - width / 2, 3.25 + width / 2], [T["VRPD"] + 9.0] * 2, color="black", lw=0.7)
    b.text(2.775, T["VRPD"] + 10.0, "$T$ = max = " + f"{T['VRPD']:.1f}", ha="center", va="bottom", fontsize=8)
    b.set_xticks([x for x, *_ in bars])
    b.set_xticklabels([lab for *_, lab in bars], fontsize=8)
    b.set_xlim(-0.55, 3.8)
    b.set_ylim(0, 92)
    b.yaxis.set_major_locator(MultipleLocator(20))
    sublabel(b, "", "(b)")
    b.set_ylabel("Time (min)")
    b.legend(handles=[Patch(facecolor="#7a7a7a", edgecolor="black", lw=0.6, label="travel time"),
                      Patch(facecolor="white", edgecolor="black", hatch="////", lw=0.6, label="loading / hand-over")],
             loc="upper left", **{**LEGEND, "handlelength": 1.6})

    # (c) travel cost (the fixed cost is 250 in every group by design)
    trav = {g: m[g].travel_cost for g in GROUPS}
    ygrid(c)
    xc = {"UAVRP": 0.0, "CVRP": 1.15, "VRPD": 2.3}
    c.bar(xc["UAVRP"], trav["UAVRP"], width, color=FILL["UAVRP"], edgecolor="black", lw=0.6, zorder=3)
    c.bar(xc["CVRP"], trav["CVRP"], width, color=FILL["CVRP"], edgecolor="black", lw=0.6, zorder=3)
    tk, dr = m["VRPD"].truck_travel_cost, m["VRPD"].drone_travel_cost
    c.bar(xc["VRPD"], tk, width, color=FILL["VRPD"], edgecolor="black", lw=0.6, zorder=3)
    c.bar(xc["VRPD"], dr, width, bottom=tk, color="white", edgecolor="black", hatch="\\\\\\\\", lw=0.6, zorder=3)
    c.text(xc["VRPD"], tk / 2, f"trucks\n{tk:.1f}", ha="center", va="center", fontsize=7.5, color="white")
    c.text(xc["VRPD"], tk + dr / 2, f"drones {dr:.1f}", ha="center", va="center", fontsize=7,
           bbox=dict(boxstyle="square,pad=0.15", fc="white", ec="none"), zorder=4)
    for g in GROUPS:
        c.text(xc[g], trav[g] + 0.4, f"{trav[g]:.2f}", ha="center", va="bottom", fontsize=8)
    c.set_xticks(list(xc.values()))
    c.set_xticklabels(list(xc.keys()), fontsize=8)
    c.set_xlim(-0.65, 2.95)
    c.set_ylim(0, 21)
    c.yaxis.set_major_locator(MultipleLocator(5))
    sublabel(c, "", "(c)")
    c.set_ylabel("Travel cost (10$^4$ CNY)")
    c.set_title("fixed cost = 250 in every group", fontsize=8, pad=4)

    fig.get_layout_engine().set(w_pad=0.08)
    save(fig, out, "fig_tradeoff")


# ------------------------------------------------------------------------------ fig 3

def fig_convergence(inst, routes, metrics, out):
    K, Dn = FLEETS["VRPD"]
    seeds = warm_start_seeds(inst, K, Dn, uavrp_routes=routes["UAVRP"]["drone_routes"])
    ev = Evaluator(inst, K, Dn)
    seed_z = [ev(s)[0] for s in seeds]
    print("  re-running the GA (deterministic) ...", flush=True)
    ga = run_ga(inst, K, Dn, seeds=seeds)
    ws = routes["VRPD"].get("warm_start_objective")
    if ws is not None and abs(ws - ga.fitness) > 1e-3:
        print(f"  WARNING: GA re-run gives {ga.fitness:.4f}, run stored {ws:.4f}")
    hist = np.array(ga.history)
    gens = np.arange(len(hist))
    milp, bound, gap = routes["VRPD"]["objective"], routes["VRPD"]["bound"], routes["VRPD"]["gap"]
    zc, zu = metrics["CVRP"].efficiency, metrics["UAVRP"].efficiency

    fig, ax = plt.subplots(figsize=(7.0, 3.1))
    g_end = int(gens[-1])
    x_max = g_end * 1.32
    ymax = max(max(seed_z), zu) + 0.55
    ymin = bound - 0.6
    ygrid(ax)
    # single-mode references
    ax.axhline(zu, color=COLOR["UAVRP"], ls=LS["UAVRP"], lw=1.1, zorder=2)
    ax.axhline(zc, color=COLOR["CVRP"], ls=LS["CVRP"], lw=1.1, zorder=2)
    # stage 1: seeds and GA
    ax.plot([0] * len(seed_z), seed_z, ls="none", marker="D", markersize=5.5, markerfacecolor="white",
            markeredgecolor="black", markeredgewidth=0.9, zorder=5)
    ax.step(gens, hist, where="post", color="black", lw=1.4, zorder=4)
    ax.plot([g_end], [hist[-1]], ls="none", marker="o", markersize=5.5, color="black", zorder=6)
    ax.annotate(f"GA: {hist[-1]:.2f}", (g_end, hist[-1]), xytext=(-5, 7), textcoords="offset points",
                fontsize=8, ha="right")
    ax.annotate(f"best seed: {min(seed_z):.2f}", (0, min(seed_z)), xytext=(12, -16), textcoords="offset points",
                fontsize=8, arrowprops=dict(arrowstyle="-", color="black", lw=0.6, shrinkB=3))
    # stage 2: MILP incumbent and proven bound
    ax.fill_between([g_end, x_max], bound, milp, facecolor="none", edgecolor="#9a9a9a", hatch="////", lw=0,
                    zorder=1)
    ax.hlines(milp, g_end, x_max, color="black", lw=2.0, zorder=4)
    ax.hlines(bound, g_end, x_max, color="black", lw=0.9, ls=(0, (2, 2)), zorder=4)
    xm = (g_end + x_max) / 2
    ax.text(xm, milp + 0.08, f"MILP incumbent: {milp:.2f}", ha="center", va="bottom", fontsize=8)
    ax.text(xm, (bound + milp) / 2, f"MIP gap {100 * gap:.1f}%", ha="center", va="center", fontsize=8,
            bbox=dict(boxstyle="square,pad=0.25", fc="white", ec="none"))
    ax.text(xm, bound - 0.08, f"lower bound: {bound:.2f}", ha="center", va="top", fontsize=8)
    ax.axvline(g_end, color="black", lw=0.7, ls=(0, (6, 3)))
    ax.text(g_end / 2, ymax - 0.06, "Stage 1: warm-start seeds + genetic algorithm", ha="center", va="top",
            fontsize=8.5)
    ax.text(xm, ymax - 0.06, "Stage 2: warm-started MILP", ha="center", va="top", fontsize=8.5)
    ax.set_xlim(-1.5, x_max)
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel("Generation")
    ax.set_ylabel("Efficiency $Z$ (10$^4$ CNY)")
    ax.set_xticks([t for t in ax.get_xticks() if 0 <= t <= g_end])
    ax.set_xlim(-1.5, x_max)
    handles = [Line2D([], [], color="black", lw=1.4, label="GA best-so-far"),
               Line2D([], [], ls="none", marker="D", markerfacecolor="white", markeredgecolor="black",
                      markersize=5.5, label="warm-start seeds"),
               Line2D([], [], color="black", lw=2.0, label="MILP incumbent"),
               Line2D([], [], color="black", lw=0.9, ls=(0, (2, 2)), label="MILP lower bound"),
               Line2D([], [], color=COLOR["CVRP"], ls=LS["CVRP"], lw=1.1, label=f"CVRP ({zc:.2f})"),
               Line2D([], [], color=COLOR["UAVRP"], ls=LS["UAVRP"], lw=1.1, label=f"UAVRP ({zu:.2f})")]
    ax.legend(handles=handles, loc="center left", bbox_to_anchor=(0.30, 0.30), ncol=2, **LEGEND)
    save(fig, out, "fig_convergence")
    return ga


# ------------------------------------------------------------------------------ fig 4

def fig_robustness(csv_path, out):
    rows = list(csv.DictReader(open(csv_path, encoding="utf-8")))
    if not rows:
        print("  robustness.csv is empty, skipping")
        return
    seeds = sorted({int(r["seed"]) for r in rows})
    Z = {g: {int(r["seed"]): float(r["efficiency"]) for r in rows if r["group"] == g} for g in GROUPS}
    seeds = [s for s in seeds if all(s in Z[g] for g in GROUPS)]
    n = len(seeds)
    fleet = {g: (int(next(r["trucks"] for r in rows if r["group"] == g)),
                 int(next(r["drones"] for r in rows if r["group"] == g))) for g in GROUPS}
    ncust = rows[0]["customers"]

    fig, (a, b) = plt.subplots(1, 2, figsize=(7.0, 3.0), gridspec_kw={"width_ratios": [0.85, 1.15]})

    # (a) distribution of Z per group over the instances: black-and-white box plots + points
    data = [[Z[g][s] for s in seeds] for g in GROUPS]
    ygrid(a)
    a.boxplot(data, positions=[0, 1, 2], widths=0.5, showfliers=False, zorder=3,
              medianprops=dict(color="black", lw=1.2), boxprops=dict(color="black", lw=0.8),
              whiskerprops=dict(color="black", lw=0.8), capprops=dict(color="black", lw=0.8))
    rng = np.random.default_rng(0)
    for i, g in enumerate(GROUPS):
        kw = line_kw(g, ms=4.5)
        a.plot(i + rng.uniform(-0.1, 0.1, n), data[i], ls="none", zorder=4,
               **{k: v for k, v in kw.items() if k != "ls"})
    a.set_xticks([0, 1, 2])
    a.set_xticklabels([f"{g}\n{fleet_text(*fleet[g])}" for g in GROUPS], fontsize=8)
    a.set_xlim(-0.6, 2.6)
    sublabel(a, "", "(a)")
    a.set_ylabel("Efficiency $Z$ (10$^4$ CNY)")
    a.set_title(f"{n} random instances, {ncust} customers each", fontsize=8, pad=4)

    # (b) difference to VRPD per instance
    x = np.arange(n)
    bw = 0.38
    ygrid(b)
    for j, g in enumerate(("UAVRP", "CVRP")):
        d = [Z[g][s] - Z["VRPD"][s] for s in seeds]
        b.bar(x + (j - 0.5) * bw, d, bw, color=FILL[g], edgecolor="black", lw=0.6, zorder=3,
              label=f"$Z_{{\\mathrm{{{g}}}}} - Z_{{\\mathrm{{VRPD}}}}$")
    b.axhline(0, color="black", lw=0.9, ls=(0, (4, 3)), zorder=4)
    wins = sum(1 for s in seeds if Z["VRPD"][s] < min(Z["CVRP"][s], Z["UAVRP"][s]))
    b.set_xticks(x)
    b.set_xticklabels([str(s) for s in seeds])
    b.set_xlim(-0.7, n - 0.3)
    ymax = max(Z[g][s] - Z["VRPD"][s] for g in ("UAVRP", "CVRP") for s in seeds)
    b.set_ylim(-0.25, ymax * 1.28)
    sublabel(b, "Instance", "(b)")
    b.set_ylabel("Difference to VRPD (10$^4$ CNY)")
    b.set_title(f"VRPD has the lowest $Z$ in {wins} of {n} instances", fontsize=8, pad=4)
    b.legend(loc="upper left", ncol=2, **LEGEND)

    fig.get_layout_engine().set(w_pad=0.08)
    save(fig, out, "fig_robustness")


# ------------------------------------------------------------------------------ route maps

def redraw_routes(inst, routes, res_dir):
    titles = {"CVRP": "CVRP - pure ground vehicles", "UAVRP": "UAVRP - pure drones",
              "VRPD": "VRPD - truck + drone collaboration"}
    for g in GROUPS:
        path = os.path.join(res_dir, f"{g.lower()}.png")
        plot_solution(inst, routes[g]["truck_routes"], routes[g]["drone_routes"], titles[g], path)
        print(f"  redrew {os.path.relpath(path)}")


# ------------------------------------------------------------------------------ main

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", default="results")
    ap.add_argument("--out", default="results/figures")
    ap.add_argument("--skip-ga", action="store_true", help="do not regenerate the GA convergence figure")
    ap.add_argument("--skip-robustness", action="store_true")
    ap.add_argument("--routes", action="store_true", help="also redraw the three route maps in results/")
    args = ap.parse_args(argv)

    style()
    os.makedirs(args.out, exist_ok=True)
    inst, routes, metrics = load(args.results)
    p = inst.params
    print("figures:")
    fig_sensitivity(metrics, p, args.out)
    fig_tradeoff(metrics, p, args.out)
    if not args.skip_ga:
        fig_convergence(inst, routes, metrics, args.out)
    rob = os.path.join(args.results, "robustness.csv")
    if not args.skip_robustness and os.path.exists(rob):
        fig_robustness(rob, args.out)
    elif not args.skip_robustness:
        print("  no robustness.csv - run scripts/robustness.py first")
    if args.routes:
        redraw_routes(inst, routes, args.results)


if __name__ == "__main__":
    main()
