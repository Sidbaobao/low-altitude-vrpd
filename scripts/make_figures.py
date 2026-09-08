"""Analysis figures for the Shenzhen case study.

Reads the output of ``run_all.py`` (``results/routes.json``, ``results/instance.json``) and,
if present, ``results/robustness.csv`` from ``scripts/robustness.py``, and writes to
``results/figures/`` (PNG 300 dpi + PDF):

    fig_sensitivity   ranking of the three groups as the objective weights / value of time vary
    fig_tradeoff      time-cost plane with iso-Z lines; decomposition of T and of the travel cost
    fig_convergence   two-stage solution of the collaborative group: GA generations -> MILP
    fig_robustness    Z of the three groups on random instances (needs robustness.csv)

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

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vrpd.config import DEFAULT, FLEETS  # noqa: E402
from vrpd.ga import Evaluator, run_ga, warm_start_seeds  # noqa: E402
from vrpd.instance import build_instance  # noqa: E402
from vrpd.metrics import evaluate  # noqa: E402

GROUPS = ("UAVRP", "CVRP", "VRPD")
# Okabe-Ito, validated colour-blind safe; identity is also carried by marker and line style.
COLOR = {"CVRP": "#009E73", "UAVRP": "#E69F00", "VRPD": "#0072B2"}
LIGHT = {"CVRP": "#8fd6c2", "UAVRP": "#f5d59a", "VRPD": "#8fbde0"}
MARKER = {"CVRP": "s", "UAVRP": "^", "VRPD": "o"}
LS = {"CVRP": "--", "UAVRP": ":", "VRPD": "-"}
INK, MUTED, GRID = "#222222", "#6b6b6b", "#e2e2e2"


def fleet_text(k, d):
    parts = [f"{k} truck{'s' if k != 1 else ''}" if k else "", f"{d} drone{'s' if d != 1 else ''}" if d else ""]
    return " + ".join(t for t in parts if t)


def label(g):
    return f"{g} ({fleet_text(*FLEETS[g])})"


def style():
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
        "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8.5,
        "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7,
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.linewidth": 0.6, "xtick.major.width": 0.6, "ytick.major.width": 0.6,
        "xtick.major.size": 2.5, "ytick.major.size": 2.5,
        "lines.linewidth": 1.4, "legend.frameon": False,
        "axes.edgecolor": INK, "text.color": INK, "axes.labelcolor": INK,
        "xtick.color": INK, "ytick.color": INK,
        "savefig.dpi": 300, "pdf.fonttype": 42, "ps.fonttype": 42,
        "figure.constrained_layout.use": True,
    })


def panel_label(ax, s, x=-0.16, y=1.04):
    ax.text(x, y, s, transform=ax.transAxes, fontsize=10, fontweight="bold", va="bottom", ha="left")


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

    fig, (a, b) = plt.subplots(1, 2, figsize=(7.2, 3.0))

    # (a) time weight sweep, value of time fixed at the paper's value
    w = np.linspace(0, 1, 501)
    dz = {g: Z(g, w, p.time_value) - Z("VRPD", w, p.time_value) for g in ("UAVRP", "CVRP")}
    best = np.minimum(dz["UAVRP"], dz["CVRP"]) > 0
    lo, hi = -2.5, 6.5
    a.fill_between(w, lo, hi, where=best, color=COLOR["VRPD"], alpha=0.08, lw=0)
    a.axhline(0, color=COLOR["VRPD"], lw=1.4, ls=LS["VRPD"])
    for g in ("UAVRP", "CVRP"):
        a.plot(w, dz[g], color=COLOR[g], ls=LS[g])
        a.plot(w[::50], dz[g][::50], marker=MARKER[g], ls="none", color=COLOR[g], ms=3.5)
    a.axvline(p.time_weight, color=MUTED, ls=(0, (2, 2)), lw=0.8)
    a.text(p.time_weight - 0.015, hi - 0.25, "paper\n$w_t$ = 0.8", ha="right", va="top", fontsize=6.5, color=MUTED)
    # crossover with CVRP
    wc = w[np.argmax(dz["CVRP"] > 0)]
    a.annotate(f"$w_t$ = {wc:.2f}", xy=(wc, 0), xytext=(wc - 0.05, 2.2), fontsize=6.5, color=MUTED,
               ha="right", arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.6))
    a.text(0.5 * (wc + 1), lo + 0.35, "VRPD most efficient", ha="center", va="bottom", fontsize=6.5,
           color=COLOR["VRPD"])
    a.text(wc / 2, lo + 0.35, "CVRP most efficient", ha="center", va="bottom", fontsize=6.5, color=COLOR["CVRP"])
    a.set_xlim(0, 1)
    a.set_ylim(lo, hi)
    a.set_xlabel("Time weight $w_t$  (cost weight $1-w_t$)")
    a.set_ylabel("$Z - Z_{\\mathrm{VRPD}}$  (10k CNY)")
    panel_label(a, "a")

    # (b) value-of-time sweep, weights fixed at the paper's 0.8 / 0.2
    tv = np.geomspace(0.02, 0.5, 400)                       # 10k CNY per minute
    dzb = {g: Z(g, p.time_weight, tv) - Z("VRPD", p.time_weight, tv) for g in ("UAVRP", "CVRP")}
    bestb = np.minimum(dzb["UAVRP"], dzb["CVRP"]) > 0
    lob, hib = -1.0, 6.0
    b.fill_between(tv * 1e4, lob, hib, where=bestb, color=COLOR["VRPD"], alpha=0.08, lw=0)
    b.axhline(0, color=COLOR["VRPD"], lw=1.4, ls=LS["VRPD"])
    for g in ("UAVRP", "CVRP"):
        b.plot(tv * 1e4, dzb[g], color=COLOR[g], ls=LS[g])
        b.plot(tv[::40] * 1e4, dzb[g][::40], marker=MARKER[g], ls="none", color=COLOR[g], ms=3.5)
    b.axvline(p.time_value * 1e4, color=MUTED, ls=(0, (2, 2)), lw=0.8)
    b.text(p.time_value * 1e4 * 0.95, hib - 0.25, "paper\n1,000 CNY/min", ha="right", va="top", fontsize=6.5, color=MUTED)
    tvc = tv[np.argmax(dzb["CVRP"] > 0)] * 1e4
    b.annotate(f"{tvc:,.0f} CNY/min", xy=(tvc, 0), xytext=(tvc * 1.6, 1.6), fontsize=6.5, color=MUTED,
               arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.6))
    b.set_xscale("log")
    b.set_xlim(200, 5000)
    b.set_ylim(lob, hib)
    b.set_xlabel("Value of time  (CNY per minute)")
    b.set_ylabel("$Z - Z_{\\mathrm{VRPD}}$  (10k CNY)")
    b.set_xticks([200, 500, 1000, 2000, 5000])
    b.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:,.0f}"))
    b.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
    panel_label(b, "b")

    handles = [Line2D([], [], color=COLOR[g], ls=LS[g], marker=MARKER[g] if g != "VRPD" else None, ms=3.5,
                      label=label(g) + (" - reference" if g == "VRPD" else "")) for g in GROUPS]
    fig.legend(handles=handles, loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.01), columnspacing=1.6)
    fig.get_layout_engine().set(h_pad=0.02, w_pad=0.04, rect=(0, 0.07, 1, 0.92))
    save(fig, out, "fig_sensitivity")


# ------------------------------------------------------------------------------ fig 2

def fig_tradeoff(metrics, p, out):
    m = metrics
    T = {g: m[g].overall_time for g in GROUPS}
    C = {g: m[g].total_cost for g in GROUPS}
    Zv = {g: m[g].efficiency for g in GROUPS}
    wt, wc, tv = p.time_weight, p.cost_weight, p.time_value

    fig, (a, b, c) = plt.subplots(1, 3, figsize=(7.2, 2.7), gridspec_kw={"width_ratios": [1.2, 1.15, 0.85]})

    # (a) time-cost plane with iso-efficiency lines  Z = wt*tv*T + wc*C
    xs = np.array([256.0, 271.0])
    for g in GROUPS:
        a.plot(xs, (Zv[g] - wc * xs) / (wt * tv), color=COLOR[g], ls=LS[g], lw=0.9, alpha=0.9)
        a.scatter([C[g]], [T[g]], color=COLOR[g], marker=MARKER[g], s=42, zorder=4, edgecolor="white", lw=0.7)
    offs = {"UAVRP": (7, 0, "left"), "CVRP": (-7, 0, "right"), "VRPD": (7, -5, "left")}
    for g in GROUPS:
        dx, dy, ha = offs[g]
        a.annotate(f"{g}\n$Z$ = {Zv[g]:.2f}", (C[g], T[g]), xytext=(dx, dy), textcoords="offset points",
                   fontsize=6.8, ha=ha, va="center", color=INK)
    a.annotate("", xy=(257.6, 49.5), xytext=(260.6, 57.0),
               arrowprops=dict(arrowstyle="-|>", color=MUTED, lw=0.8, mutation_scale=8))
    a.text(258.1, 53.6, "lower $Z$", fontsize=6.5, color=MUTED, rotation=68, ha="left", va="center")
    a.text(270.6, 48.6, "lines: constant $Z$", fontsize=6.5, color=MUTED, ha="right", va="bottom")
    a.set_xlim(256, 271)
    a.set_ylim(48, 71)
    a.set_xlabel("Total cost $C$  (10k CNY)")
    a.set_ylabel("Total time $T$  (min)")
    panel_label(a, "a")

    # (b) decomposition of T: travel time + fixed loading / hand-over time, per mode
    bars = []  # (x, group, travel, handling, tick label)
    du = m["UAVRP"].drone_distance / p.drone_speed * 60
    tc = m["CVRP"].truck_distance / p.truck_speed * 60
    tv_t = m["VRPD"].truck_distance / p.truck_speed * 60
    tv_d = m["VRPD"].drone_distance / p.drone_speed * 60
    bars.append((0.0, "UAVRP", du, m["UAVRP"].drone_time - du, "UAVRP\n10 drones"))
    bars.append((1.0, "CVRP", tc, m["CVRP"].truck_time - tc, "CVRP\n5 trucks"))
    bars.append((2.15, "VRPD", tv_t, m["VRPD"].truck_time - tv_t, "VRPD\n4 trucks"))
    bars.append((2.85, "VRPD", tv_d, m["VRPD"].drone_time - tv_d, "VRPD\n2 drones"))
    width = 0.62
    for x, g, trav, hand, _ in bars:
        b.bar(x, trav, width, color=COLOR[g], lw=0)
        b.bar(x, hand, width, bottom=trav + 0.4, color=LIGHT[g], hatch="////", edgecolor=COLOR[g], lw=0)
        b.text(x, trav + hand + 1.6, f"{trav + hand:.1f}", ha="center", va="bottom", fontsize=6.8)
    b.plot([2.15 - width / 2, 2.85 + width / 2], [T["VRPD"] + 7.5] * 2, color=MUTED, lw=0.7)
    b.text(2.5, T["VRPD"] + 8.3, "$T$ = max = " + f"{T['VRPD']:.1f}", ha="center", va="bottom", fontsize=6.5, color=MUTED)
    b.set_xticks([x for x, *_ in bars])
    b.set_xticklabels([lab for *_, lab in bars])
    b.set_ylim(0, 92)
    b.yaxis.set_major_locator(matplotlib.ticker.MultipleLocator(20))
    b.set_ylabel("Time  (min)")
    b.legend(handles=[Patch(color="#9a9a9a", label="travel"),
                      Patch(facecolor="#dcdcdc", hatch="////", edgecolor="#9a9a9a", lw=0, label="loading / hand-over")],
             loc="upper left", handlelength=1.4, borderaxespad=0.2)
    panel_label(b, "b")

    # (c) travel cost (the fixed cost is 250 in every group by design)
    trav = {g: m[g].travel_cost for g in GROUPS}
    c.bar(0, trav["UAVRP"], width, color=COLOR["UAVRP"], lw=0)
    c.bar(1, trav["CVRP"], width, color=COLOR["CVRP"], lw=0)
    tk = m["VRPD"].truck_distance * p.truck_cost
    dr = m["VRPD"].drone_distance * p.drone_cost
    c.bar(2, tk, width, color=COLOR["VRPD"], lw=0)
    c.bar(2, dr, width, bottom=tk + 0.18, color=LIGHT["VRPD"], lw=0)
    c.text(2, tk / 2, f"trucks\n{tk:.1f}", ha="center", va="center", fontsize=6.2, color="white")
    c.text(2, tk + 0.18 + dr / 2, f"drones {dr:.1f}", ha="center", va="center", fontsize=6.2, color=INK)
    for x, g in ((0, "UAVRP"), (1, "CVRP"), (2, "VRPD")):
        c.text(x, trav[g] + 0.5, f"{trav[g]:.2f}", ha="center", va="bottom", fontsize=6.8)
    c.set_xticks([0, 1, 2])
    c.set_xticklabels(["UAVRP", "CVRP", "VRPD"])
    c.set_ylim(0, 21)
    c.yaxis.set_major_locator(matplotlib.ticker.MultipleLocator(5))
    c.set_ylabel("Travel cost  (10k CNY)")
    c.set_title("fixed cost = 250 in every group", fontsize=6.8, color=MUTED, pad=3)
    panel_label(c, "c", x=-0.28)

    fig.get_layout_engine().set(h_pad=0.02, w_pad=0.05)
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

    fig, ax = plt.subplots(figsize=(7.2, 2.8))
    g_end = gens[-1]
    x_max = g_end * 1.3
    ax.axvspan(g_end, x_max, color="#f3f3f3", lw=0)
    # references
    ax.axhline(zu, color=COLOR["UAVRP"], ls=LS["UAVRP"], lw=1.1)
    ax.axhline(zc, color=COLOR["CVRP"], ls=LS["CVRP"], lw=1.1)
    ax.text(x_max * 0.995, zu + 0.06, f"UAVRP  {zu:.2f}", ha="right", va="bottom", fontsize=6.5, color=COLOR["UAVRP"])
    ax.text(x_max * 0.995, zc + 0.06, f"CVRP  {zc:.2f}", ha="right", va="bottom", fontsize=6.5, color=COLOR["CVRP"])
    # stage 1: seeds and GA
    ax.scatter([0] * len(seed_z), seed_z, marker="D", s=14, facecolor="white", edgecolor=MUTED, lw=0.8, zorder=4)
    ax.step(gens, hist, where="post", color=COLOR["VRPD"], lw=1.5, zorder=3)
    ax.scatter([g_end], [hist[-1]], color=COLOR["VRPD"], s=22, zorder=5, edgecolor="white", lw=0.6)
    ax.annotate(f"GA  {hist[-1]:.2f}", (g_end, hist[-1]), xytext=(-4, 9), textcoords="offset points",
                fontsize=6.5, ha="right", color=COLOR["VRPD"])
    ax.annotate(f"best warm-start seed  {min(seed_z):.2f}", (0, min(seed_z)), xytext=(10, -16),
                textcoords="offset points", fontsize=6.5, color=MUTED,
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.6))
    # stage 2: MILP incumbent and proven bound
    ax.hlines(milp, g_end, x_max, color=COLOR["VRPD"], lw=1.8)
    ax.hlines(bound, g_end, x_max, color=MUTED, lw=0.9, ls=(0, (3, 2)))
    ax.fill_between([g_end, x_max], bound, milp, color=COLOR["VRPD"], alpha=0.08, lw=0)
    ax.text((g_end + x_max) / 2, milp + 0.08, f"MILP  {milp:.2f}", ha="center", va="bottom", fontsize=6.5,
            color=COLOR["VRPD"])
    ax.text((g_end + x_max) / 2, (bound + milp) / 2, f"MIP gap {100 * gap:.1f} %", ha="center", va="center",
            fontsize=6.5, color=MUTED)
    ax.text((g_end + x_max) / 2, bound - 0.08, f"proven lower bound  {bound:.2f}", ha="center", va="top",
            fontsize=6.5, color=MUTED)
    ax.axvline(g_end, color=MUTED, lw=0.6)
    ymax = max(max(seed_z), zu) + 0.6
    ymin = bound - 0.7
    ax.text(g_end / 2, ymax - 0.05, "stage 1 - warm-start seeds + genetic algorithm", ha="center", va="top",
            fontsize=7, color=INK)
    ax.text((g_end + x_max) / 2, ymax - 0.05, "stage 2 - warm-started MILP", ha="center", va="top", fontsize=7,
            color=INK)
    ax.set_xlim(-1.5, x_max)
    ax.set_ylim(ymin, ymax)
    ax.set_xlabel("Generation")
    ax.set_ylabel("Efficiency $Z$  (10k CNY)")
    ax.set_xticks([t for t in ax.get_xticks() if 0 <= t <= g_end])
    ax.set_xlim(-1.5, x_max)
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

    fig, (a, b) = plt.subplots(1, 2, figsize=(7.2, 2.8), gridspec_kw={"width_ratios": [1.1, 1]})

    # (a) paired values: one thin line per instance through its three groups
    xpos = {"UAVRP": 0, "CVRP": 1, "VRPD": 2}
    for s in seeds:
        a.plot([xpos[g] for g in GROUPS], [Z[g][s] for g in GROUPS], color="#b8b8b8", lw=0.6, zorder=1)
    for g in GROUPS:
        a.scatter([xpos[g]] * n, [Z[g][s] for s in seeds], color=COLOR[g], marker=MARKER[g], s=22, zorder=3,
                  edgecolor="white", lw=0.5)
    a.set_xticks([0, 1, 2])
    a.set_xticklabels([f"{g}\n{fleet_text(*fleet[g])}" for g in GROUPS])
    a.set_xlim(-0.4, 2.4)
    a.set_ylabel("Efficiency $Z$  (10k CNY)")
    a.set_title(f"{n} random instances, {ncust} customers each", fontsize=7, color=MUTED, pad=3)
    panel_label(a, "a")

    # (b) paired differences to VRPD with mean and 95 % CI
    rng = np.random.default_rng(0)
    wins = sum(1 for s in seeds if Z["VRPD"][s] < min(Z["CVRP"][s], Z["UAVRP"][s]))
    from scipy import stats
    for i, g in enumerate(("UAVRP", "CVRP")):
        d = np.array([Z[g][s] - Z["VRPD"][s] for s in seeds])
        jitter = rng.uniform(-0.12, 0.12, n)
        b.scatter(i + jitter, d, color=COLOR[g], marker=MARKER[g], s=20, alpha=0.85, edgecolor="white", lw=0.5, zorder=3)
        mean, sd = d.mean(), d.std(ddof=1)
        ci = stats.t.ppf(0.975, n - 1) * sd / math.sqrt(n) if n > 1 else 0.0
        b.errorbar(i + 0.3, mean, yerr=ci, fmt="_", color=INK, ms=10, mew=1.2, capsize=3, elinewidth=0.9, zorder=4)
        b.text(i + 0.38, mean, f"{mean:+.2f}", fontsize=6.5, va="center", ha="left", color=INK)
    b.axhline(0, color=COLOR["VRPD"], lw=1.2)
    b.set_xticks([0, 1])
    b.set_xticklabels(["UAVRP $-$ VRPD", "CVRP $-$ VRPD"])
    b.set_xlim(-0.5, 1.7)
    b.set_ylabel("$Z - Z_{\\mathrm{VRPD}}$  (10k CNY)")
    b.set_title(f"VRPD has the lowest $Z$ in {wins} of {n} instances", fontsize=7, color=MUTED, pad=3)
    ymin, ymax = b.get_ylim()
    b.set_ylim(min(ymin, -0.3), ymax)
    b.text(1.68, 0.02 * (ymax - ymin), "above 0: VRPD better", fontsize=6, color=MUTED, ha="right", va="bottom")
    panel_label(b, "b")

    save(fig, out, "fig_robustness")


# ------------------------------------------------------------------------------ main

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", default="results")
    ap.add_argument("--out", default="results/figures")
    ap.add_argument("--skip-ga", action="store_true", help="do not regenerate the GA convergence figure")
    ap.add_argument("--skip-robustness", action="store_true")
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


if __name__ == "__main__":
    main()
