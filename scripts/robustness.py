"""Ranking robustness across random instances.

Re-runs the three delivery groups on several random customer layouts of a smaller size
(default: 10 instances of 20 customers in the same 1.54 km x 1.54 km area) with fleets scaled
so that every group still carries the same fixed cost, and records the efficiency Z of each
group per instance.  Used by ``scripts/make_figures.py`` for the robustness figure.

    python scripts/robustness.py                      # 10 seeds x 20 customers, 60 s per model
    python scripts/robustness.py --seeds 5 --customers 15 --time-limit 30

Every plan is passed through ``vrpd.verify`` and the solver objective is asserted to equal the
value of Eq. (1) computed by ``vrpd.metrics``.  Results are appended to the CSV after every
instance, so an interrupted run keeps what it has finished.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vrpd.config import DEFAULT
from vrpd.ga import run_ga, warm_start_seeds
from vrpd.heuristics import best_sweep_routes
from vrpd.instance import build_instance
from vrpd.metrics import evaluate
from vrpd.models import solve_cvrp, solve_uavrp, solve_vrpd
from vrpd.verify import verify

FIELDS = ["seed", "customers", "group", "trucks", "drones", "distance", "travel_cost", "total_cost",
          "overall_time", "efficiency", "status", "gap", "runtime"]


def parse_fleets(text: str) -> dict:
    """'2,0;0,4;1,2' -> {'CVRP': (2, 0), 'UAVRP': (0, 4), 'VRPD': (1, 2)}"""
    parts = [tuple(int(v) for v in p.split(",")) for p in text.split(";")]
    if len(parts) != 3:
        raise SystemExit("--fleets needs three 'trucks,drones' pairs: CVRP;UAVRP;VRPD")
    return {"CVRP": parts[0], "UAVRP": parts[1], "VRPD": parts[2]}


def run_instance(params, fleets, log):
    inst = build_instance(params)
    rows = {}

    K, _ = fleets["CVRP"]
    warm = best_sweep_routes(inst.customers, inst.points, inst.manhattan, K)
    res = solve_cvrp(inst, K, warm_routes=warm)
    verify(inst, res.truck_routes, [], K, 0)
    rows["CVRP"] = (res, evaluate(inst, res.truck_routes, [], K, 0))

    _, Dn = fleets["UAVRP"]
    warm = best_sweep_routes(inst.customers, inst.points, inst.euclid, Dn)
    res = solve_uavrp(inst, Dn, warm_routes=warm)
    verify(inst, [], res.drone_routes, 0, Dn)
    rows["UAVRP"] = (res, evaluate(inst, [], res.drone_routes, 0, Dn))

    K, Dn = fleets["VRPD"]
    seeds = warm_start_seeds(inst, K, Dn, uavrp_routes=rows["UAVRP"][0].drone_routes)
    ga = run_ga(inst, K, Dn, seeds=seeds)
    res = solve_vrpd(inst, K, Dn, warm_truck_routes=ga.truck_routes, warm_drone_routes=ga.drone_routes)
    verify(inst, res.truck_routes, res.drone_routes, K, Dn)
    rows["VRPD"] = (res, evaluate(inst, res.truck_routes, res.drone_routes, K, Dn))

    for g, (res, mt) in rows.items():
        assert abs(res.objective - mt.efficiency) < 1e-4, (g, res.objective, mt.efficiency)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=10, help="number of random instances (seeds 1..N)")
    ap.add_argument("--customers", type=int, default=20)
    ap.add_argument("--fleets", default="2,0;0,4;1,2",
                    help="trucks,drones for CVRP;UAVRP;VRPD (default keeps the fixed cost equal: 100)")
    ap.add_argument("--time-limit", type=float, default=60.0, help="seconds per MILP")
    ap.add_argument("--out", default="results/robustness.csv")
    args = ap.parse_args(argv)

    fleets = parse_fleets(args.fleets)
    fixed = {g: k * DEFAULT.fixed_cost_truck + d * DEFAULT.fixed_cost_drone for g, (k, d) in fleets.items()}
    if len(set(fixed.values())) != 1:
        raise SystemExit(f"fleets must carry the same fixed cost, got {fixed}")
    demand = args.customers * DEFAULT.demand_per_customer
    for g, (k, d) in fleets.items():
        cap = k * DEFAULT.truck_capacity + d * DEFAULT.drone_capacity
        if cap < demand:
            raise SystemExit(f"{g}: fleet capacity {cap} kg < demand {demand} kg")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    new_file = not os.path.exists(args.out)
    t0 = time.time()

    def log(msg):
        print(f"[{time.time() - t0:7.1f}s] {msg}", flush=True)

    log(f"{args.seeds} instances x {args.customers} customers, fleets {fleets}, "
        f"fixed cost {next(iter(fixed.values())):g} per group, {args.time_limit:g}s per MILP")
    with open(args.out, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new_file:
            w.writeheader()
        for seed in range(1, args.seeds + 1):
            params = DEFAULT.replace(num_node=args.customers, random_seed=seed,
                                     time_limit_cvrp=args.time_limit, time_limit_uavrp=args.time_limit,
                                     time_limit_vrpd=args.time_limit,
                                     ga_population=60, ga_generations=150, ga_patience=30)
            rows = run_instance(params, fleets, log)
            for g in ("UAVRP", "CVRP", "VRPD"):
                res, mt = rows[g]
                k, d = fleets[g]
                w.writerow({"seed": seed, "customers": args.customers, "group": g, "trucks": k, "drones": d,
                            "distance": f"{mt.total_distance:.4f}", "travel_cost": f"{mt.travel_cost:.4f}",
                            "total_cost": f"{mt.total_cost:.4f}", "overall_time": f"{mt.overall_time:.4f}",
                            "efficiency": f"{mt.efficiency:.4f}", "status": res.status,
                            "gap": f"{res.gap:.4f}", "runtime": f"{res.runtime:.1f}"})
            f.flush()
            z = {g: rows[g][1].efficiency for g in rows}
            winner = min(z, key=z.get)
            log(f"seed {seed}/{args.seeds} done: UAVRP {z['UAVRP']:.3f}  CVRP {z['CVRP']:.3f}  "
                f"VRPD {z['VRPD']:.3f}  -> best {winner}")
    log(f"written to {args.out}")


if __name__ == "__main__":
    main()
