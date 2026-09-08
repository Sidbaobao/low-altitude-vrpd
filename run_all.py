"""Reproduce the Shenzhen case study (paper Section 5) for all three delivery groups.

    python run_all.py                 # full 50-customer case study (Gurobi, up to ~1 h)
    python run_all.py --quick         # 12-customer smoke run, finishes in about a minute
    python run_all.py --log           # show Gurobi's own log
    python run_all.py --out my_dir    # write results somewhere else

Pipeline (paper Section 4.1)
----------------------------
1. Build the shared instance (same customers, demands, costs for every group).
2. CVRP  - MILP with a nearest-neighbour warm start.
3. UAVRP - MILP with a nearest-neighbour warm start.
4. VRPD  - warm-start seeds from the two baselines -> genetic algorithm -> full MILP
           warm-started with the GA plan (which verifies and, if possible, improves it).
5. Verify every plan against Eq. (7)-(16), evaluate Eq. (1)-(6), write tables and figures.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time

from vrpd.config import DEFAULT, FLEETS, FLEETS_QUICK, QUICK
from vrpd.ga import run_ga, warm_start_seeds
from vrpd.heuristics import best_sweep_routes
from vrpd.instance import build_instance
from vrpd.metrics import evaluate
from vrpd.models import solve_cvrp, solve_uavrp, solve_vrpd
from vrpd.plotting import plot_solution
from vrpd.verify import verify

T0 = time.time()


def log(msg: str):
    print(f"[{time.time() - T0:7.1f}s] {msg}", flush=True)


def fmt_route(r):
    return "->".join(str(i) for i in r)


def pct(new, base):
    return 100.0 * (base - new) / base


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true", help="12-customer smoke run")
    ap.add_argument("--log", action="store_true", help="show Gurobi output")
    ap.add_argument("--out", default=None, help="output directory (default: results/ or results/quick/)")
    args = ap.parse_args(argv)

    params = QUICK if args.quick else DEFAULT
    fleets = FLEETS_QUICK if args.quick else FLEETS
    out = args.out or ("results/quick" if args.quick else "results")
    os.makedirs(out, exist_ok=True)

    inst = build_instance(params)
    log(f"instance: {inst.n} customers, area {params.range_coordinate[1]:.2f} km square, "
        f"demand {params.demand_per_customer:g} kg each, seed {params.random_seed}")

    results, metrics = {}, {}

    # ---- CVRP -----------------------------------------------------------------------
    K, _ = fleets["CVRP"]
    log(f"CVRP: {K} trucks, nearest-neighbour warm start + MILP (limit {params.time_limit_cvrp:g}s)")
    warm = best_sweep_routes(inst.customers, inst.points, inst.manhattan, K)
    res = solve_cvrp(inst, K, warm_routes=warm, log=args.log)
    results["CVRP"] = res
    metrics["CVRP"] = evaluate(inst, res.truck_routes, [], K, 0)
    log(f"CVRP done: {res.status}, Z = {res.objective:.4f}, gap {100 * res.gap:.2f}%, {res.runtime:.1f}s")

    # ---- UAVRP ----------------------------------------------------------------------
    _, Dn = fleets["UAVRP"]
    log(f"UAVRP: {Dn} drones, nearest-neighbour warm start + MILP (limit {params.time_limit_uavrp:g}s)")
    warm = best_sweep_routes(inst.customers, inst.points, inst.euclid, Dn)
    res = solve_uavrp(inst, Dn, warm_routes=warm, log=args.log)
    results["UAVRP"] = res
    metrics["UAVRP"] = evaluate(inst, [], res.drone_routes, 0, Dn)
    log(f"UAVRP done: {res.status}, Z = {res.objective:.4f}, gap {100 * res.gap:.2f}%, {res.runtime:.1f}s")

    # ---- VRPD -----------------------------------------------------------------------
    K, Dn = fleets["VRPD"]
    log(f"VRPD: {K} trucks + {Dn} drones - stage 1: warm-start seeds from the baselines, genetic algorithm")
    seeds = warm_start_seeds(inst, K, Dn, uavrp_routes=results["UAVRP"].drone_routes)
    ga = run_ga(inst, K, Dn, seeds=seeds, log=log)
    verify(inst, ga.truck_routes, ga.drone_routes, K, Dn)
    log(f"GA done after {ga.generations} generations: Z = {ga.fitness:.4f}")
    log(f"VRPD - stage 2: full MILP warm-started with the GA plan (limit {params.time_limit_vrpd:g}s)")
    res = solve_vrpd(inst, K, Dn, warm_truck_routes=ga.truck_routes, warm_drone_routes=ga.drone_routes,
                     log=args.log)
    results["VRPD"] = res
    metrics["VRPD"] = evaluate(inst, res.truck_routes, res.drone_routes, K, Dn)
    log(f"VRPD done: {res.status}, Z = {res.objective:.4f} (GA start {ga.fitness:.4f}), "
        f"gap {100 * res.gap:.2f}%, {res.runtime:.1f}s")

    # ---- report ---------------------------------------------------------------------
    for g in ("CVRP", "UAVRP", "VRPD"):
        r, mt = results[g], metrics[g]
        # the value Gurobi optimised must be the value the evaluator reports
        assert abs(r.objective - mt.efficiency) < 1e-4, (g, r.objective, mt.efficiency)

    lines = []
    lines.append("## Table 1 - Routes\n")
    lines.append("| Group | Vehicle | Route | Distance (km) | Load (kg) |")
    lines.append("|---|---|---|---:|---:|")
    for g in ("VRPD", "CVRP", "UAVRP"):
        r = results[g]
        for k, rt in enumerate(r.truck_routes):
            d = sum(inst.manhattan[a][b] for a, b in zip(rt, rt[1:]))
            lines.append(f"| {g} | Truck {k + 1} | {fmt_route(rt)} | {d:.3f} | {sum(inst.demand[i] for i in rt):g} |")
        for d_, rt in enumerate(r.drone_routes):
            d = sum(inst.euclid[a][b] for a, b in zip(rt, rt[1:]))
            lines.append(f"| {g} | Drone {d_ + 1} | {fmt_route(rt)} | {d:.3f} | {sum(inst.demand[i] for i in rt):g} |")

    lines.append("\n## Table 2 - Delivery effect of the three groups\n")
    lines.append("| Group | Fleet | Distance (km) | Fixed cost | Travel cost | Total cost | Total time (min) | Efficiency Z | Solver | Gap | Time (s) |")
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|---|---:|---:|")
    for g in ("UAVRP", "CVRP", "VRPD"):
        r, mt = results[g], metrics[g]
        fleet = " + ".join(s for s in [f"{mt.num_trucks} trucks" if mt.num_trucks else "",
                                       f"{mt.num_drones} drones" if mt.num_drones else ""] if s)
        lines.append(f"| {g} | {fleet} | {mt.total_distance:.2f} | {mt.fixed_cost:.2f} | {mt.travel_cost:.2f} | "
                     f"{mt.total_cost:.2f} | {mt.overall_time:.2f} | **{mt.efficiency:.2f}** | {r.status} | "
                     f"{100 * r.gap:.2f}% | {r.runtime:.1f} |")

    v, c, u = metrics["VRPD"], metrics["CVRP"], metrics["UAVRP"]
    lines.append("\n## VRPD relative to the single-mode groups (positive = VRPD is better)\n")
    lines.append("| Indicator | vs CVRP | vs UAVRP |")
    lines.append("|---|---:|---:|")
    for name, attr in [("Total distance", "total_distance"), ("Total time", "overall_time"),
                       ("Travel cost", "travel_cost"), ("Total cost", "total_cost"),
                       ("Efficiency Z", "efficiency")]:
        lines.append(f"| {name} | {pct(getattr(v, attr), getattr(c, attr)):+.1f}% | "
                     f"{pct(getattr(v, attr), getattr(u, attr)):+.1f}% |")
    lines.append(f"\nGA: {ga.generations} generations, best Z = {ga.fitness:.4f}; "
                 f"MILP improved it to {results['VRPD'].objective:.4f}.\n")
    # Same routes, alternative accounting of the fixed service time (see Params.fixed_time_per).
    alt_name = "mode" if params.fixed_time_per == "vehicle" else "vehicle"
    inst_alt = build_instance(params.replace(fixed_time_per=alt_name))
    metrics_alt = {g: evaluate(inst_alt, results[g].truck_routes, results[g].drone_routes,
                               metrics[g].num_trucks, metrics[g].num_drones) for g in results}
    lines.append(f"## Supplement - fixed service time charged per {params.fixed_time_per} (tables above) "
                 f"vs per {alt_name} (below), identical routes\n")
    lines.append("| Group | Total time (min) | Efficiency Z |")
    lines.append("|---|---:|---:|")
    for g in ("UAVRP", "CVRP", "VRPD"):
        lines.append(f"| {g} | {metrics_alt[g].overall_time:.2f} | {metrics_alt[g].efficiency:.2f} |")
    lines.append("")
    report = "\n".join(lines)
    print("\n" + report)

    # ---- files ----------------------------------------------------------------------
    with open(os.path.join(out, "summary.md"), "w", encoding="utf-8") as f:
        f.write(report)
    with open(os.path.join(out, "summary.csv"), "w", newline="", encoding="utf-8") as f:
        w = None
        for g in ("UAVRP", "CVRP", "VRPD"):
            row = {"group": g, **metrics[g].as_dict(), "solver_status": results[g].status,
                   "solver_gap": results[g].gap, "solver_runtime_s": results[g].runtime,
                   "solver_bound": results[g].bound, "fixed_time_per": params.fixed_time_per,
                   f"overall_time_per_{alt_name}": metrics_alt[g].overall_time,
                   f"efficiency_per_{alt_name}": metrics_alt[g].efficiency}
            if w is None:
                w = csv.DictWriter(f, fieldnames=list(row))
                w.writeheader()
            w.writerow(row)
    with open(os.path.join(out, "routes.json"), "w", encoding="utf-8") as f:
        json.dump({g: {"truck_routes": results[g].truck_routes, "drone_routes": results[g].drone_routes,
                       "objective": results[g].objective, "bound": results[g].bound, "gap": results[g].gap,
                       "runtime": results[g].runtime, "status": results[g].status,
                       "warm_start_objective": results[g].warm_start_objective}
                   for g in results} | {"ga": {"fitness": ga.fitness, "generations": ga.generations,
                                               "history": ga.history}},
                  f, indent=1)
    with open(os.path.join(out, "instance.json"), "w", encoding="utf-8") as f:
        json.dump({"points": inst.points, "demand": inst.demand,
                   "params": inst.params.__dict__, "fleets": fleets}, f, indent=1)
    for g, title in [("CVRP", "CVRP - pure ground vehicles"), ("UAVRP", "UAVRP - pure drones"),
                     ("VRPD", "VRPD - truck + drone collaboration")]:
        plot_solution(inst, results[g].truck_routes, results[g].drone_routes, title,
                      os.path.join(out, f"{g.lower()}.png"))
    log(f"written to {out}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
