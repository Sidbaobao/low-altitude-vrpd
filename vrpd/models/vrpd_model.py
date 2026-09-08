"""Collaborative group – VRP with drones, parallel independent operation (paper Eq. 1–17).

This is the complete model of Section 3.4: trucks ``x[k, i, j]`` on Manhattan distances,
drones ``y[d, i, j]`` on Euclidean distances, each customer served exactly once by either
mode, every vehicle leaving and returning to the depot once, MTZ sub-tour elimination
(load form, which also enforces the capacities of Eq. 14/15), the per-drone range limit of
Eq. (16) and the objective of Eq. (1) with

    T_overall = max{T_truck, T_drone}                                                (2)

modelled through a continuous makespan variable ``T >= T_truck`` and ``T >= T_drone``.

The solver is warm-started with the plan produced by the genetic algorithm (Section 4.6),
so the returned incumbent is never worse than the GA plan and comes with a proven bound.
"""

from __future__ import annotations

import gurobipy as gp
from gurobipy import GRB

from ..instance import Instance
from ..metrics import evaluate, fixed_time
from ..verify import verify
from .common import (SolveResult, bound_of, canonical, cumulative_loads, gap_of, make_model,
                     routes_from_arcs, status_name)


def solve_vrpd(inst: Instance, num_trucks: int, num_drones: int, warm_truck_routes=None,
               warm_drone_routes=None, time_limit: float | None = None, log: bool = False) -> SolveResult:
    p = inst.params
    N, C = list(inst.nodes), list(inst.customers)
    M, D, q = inst.manhattan, inst.euclid, inst.demand
    Qt, Qd, R = p.truck_capacity, p.drone_capacity, p.drone_range
    V, U = list(range(num_trucks)), list(range(num_drones))

    m = make_model("VRPD", p, time_limit or p.time_limit_vrpd, log)
    t_arcs = [(k, i, j) for k in V for i in N for j in N if i != j]
    d_arcs = [(d, i, j) for d in U for i in N for j in N if i != j]
    x = m.addVars(t_arcs, vtype=GRB.BINARY, name="x")
    y = m.addVars(d_arcs, vtype=GRB.BINARY, name="y")
    u = m.addVars(((k, i) for k in V for i in C), lb=0.0, ub=Qt, name="u")
    v = m.addVars(((d, i) for d in U for i in C), lb=0.0, ub=Qd, name="v")
    T = m.addVar(lb=0.0, name="T_overall")

    def x_in(k, j):
        return gp.quicksum(x[k, i, j] for i in N if i != j)

    def x_out(k, i):
        return gp.quicksum(x[k, i, j] for j in N if j != i)

    def y_in(d, j):
        return gp.quicksum(y[d, i, j] for i in N if i != j)

    def y_out(d, i):
        return gp.quicksum(y[d, i, j] for j in N if j != i)

    # (7) each customer served exactly once, by a truck or a drone
    m.addConstrs((gp.quicksum(x_in(k, j) for k in V) + gp.quicksum(y_in(d, j) for d in U) == 1
                  for j in C), name="visit")
    # (8)/(9) trucks: depart and return once, flow balance
    m.addConstrs((x_out(k, 0) == 1 for k in V), name="t_depart")
    m.addConstrs((x_in(k, 0) == 1 for k in V), name="t_return")
    m.addConstrs((x_out(k, i) - x_in(k, i) == 0 for k in V for i in C), name="t_flow")
    # (10)/(11) drones: depart and return once, flow balance
    m.addConstrs((y_out(d, 0) == 1 for d in U), name="d_depart")
    m.addConstrs((y_in(d, 0) == 1 for d in U), name="d_return")
    m.addConstrs((y_out(d, i) - y_in(d, i) == 0 for d in U for i in C), name="d_flow")
    # (12)+(14) truck MTZ in load form, (13)+(15) drone MTZ in load form
    m.addConstrs((u[k, i] - u[k, j] + Qt * x[k, i, j] <= Qt - q[j]
                  for k in V for i in C for j in C if i != j), name="t_mtz")
    m.addConstrs((u[k, j] >= q[j] * x_in(k, j) for k in V for j in C), name="t_load_lb")
    m.addConstrs((v[d, i] - v[d, j] + Qd * y[d, i, j] <= Qd - q[j]
                  for d in U for i in C for j in C if i != j), name="d_mtz")
    m.addConstrs((v[d, j] >= q[j] * y_in(d, j) for d in U for j in C), name="d_load_lb")
    # (16) drone range
    m.addConstrs((gp.quicksum(D[i][j] * y[d, i, j] for i in N for j in N if i != j) <= R
                  for d in U), name="range")
    # symmetry breaking inside each class of identical vehicles
    m.addConstrs((x_in(k, j) == 0 for k in V for j in C if j <= k), name="t_symmetry")
    m.addConstrs((y_in(d, j) == 0 for d in U for j in C if j <= d), name="d_symmetry")

    # (3)–(6) and (2)
    truck_dist = gp.quicksum(M[i][j] * x[k, i, j] for k, i, j in t_arcs)
    drone_dist = gp.quicksum(D[i][j] * y[d, i, j] for d, i, j in d_arcs)
    t_truck = (60.0 / p.truck_speed) * truck_dist + fixed_time(p, "truck", num_trucks)
    t_drone = (60.0 / p.drone_speed) * drone_dist + fixed_time(p, "drone", num_drones)
    m.addConstr(T >= t_truck, name="makespan_truck")
    m.addConstr(T >= t_drone, name="makespan_drone")
    c_total = (p.truck_cost * truck_dist + num_trucks * p.fixed_cost_truck
               + p.drone_cost * drone_dist + num_drones * p.fixed_cost_drone)
    # (1)
    m.setObjective(p.time_weight * p.time_value * T + p.cost_weight * c_total, GRB.MINIMIZE)

    warm_obj = None
    if warm_truck_routes is not None and warm_drone_routes is not None:
        for var in list(x.values()) + list(y.values()):
            var.Start = 0.0
        for k, r in enumerate(canonical(warm_truck_routes)[:num_trucks]):
            for a, b in zip(r, r[1:]):
                x[k, a, b].Start = 1.0
            for i, load in cumulative_loads(r, q).items():
                u[k, i].Start = load
        for d, r in enumerate(canonical(warm_drone_routes)[:num_drones]):
            for a, b in zip(r, r[1:]):
                y[d, a, b].Start = 1.0
            for i, load in cumulative_loads(r, q).items():
                v[d, i].Start = load
        warm = evaluate(inst, warm_truck_routes, warm_drone_routes, num_trucks, num_drones)
        T.Start = warm.overall_time
        warm_obj = warm.efficiency

    m.optimize()
    if m.SolCount == 0:
        raise RuntimeError(f"VRPD: no feasible solution ({status_name(m)})")

    truck_routes, drone_routes = [], []
    for k in V:
        active = [(i, j) for (kk, i, j) in t_arcs if kk == k and x[kk, i, j].X > 0.5]
        truck_routes.extend(routes_from_arcs(active, N))
    for d in U:
        active = [(i, j) for (dd, i, j) in d_arcs if dd == d and y[dd, i, j].X > 0.5]
        drone_routes.extend(routes_from_arcs(active, N))
    verify(inst, truck_routes, drone_routes, num_trucks, num_drones)
    return SolveResult(group="VRPD", truck_routes=truck_routes, drone_routes=drone_routes,
                       objective=m.ObjVal, bound=bound_of(m), gap=gap_of(m), runtime=m.Runtime,
                       status=status_name(m), warm_start_objective=warm_obj,
                       extra={"num_vars": m.NumVars, "num_constrs": m.NumConstrs})
