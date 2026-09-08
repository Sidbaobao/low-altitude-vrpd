"""Pure drone group – UAV routing problem (paper Eq. 1, 2, 4, 6, 7, 10, 11, 13, 15, 16, 17).

Formulation notes
-----------------
* The range limit Eq. (16) is per drone, so the drone index ``d`` is kept: ``y[d, i, j]``.
* Sub-tour elimination uses the load form of MTZ, folding Eq. (15) into Eq. (13):
  v_{d,i} - v_{d,j} + Q y_{d,i,j} <= Q - q_j,  q_j * (visited by d) <= v_{d,j} <= Q.
* Drones are interchangeable, so a symmetry-breaking rule is added: customer j can only be
  served by drone d if j > d (0-based d).  Every feasible plan has a relabelling that obeys
  this rule, so no solution is lost.
* Routes are read straight off the y variables – no post-processing re-ordering, so the
  returned sorties satisfy exactly the constraints the solver enforced.
"""

from __future__ import annotations

import gurobipy as gp
from gurobipy import GRB

from ..instance import Instance
from ..metrics import evaluate, fixed_time
from ..verify import verify
from .common import (SolveResult, bound_of, canonical, cumulative_loads, gap_of, make_model,
                     routes_from_arcs, status_name)


def solve_uavrp(inst: Instance, num_drones: int, warm_routes=None, time_limit: float | None = None,
                log: bool = False) -> SolveResult:
    p = inst.params
    N, C = list(inst.nodes), list(inst.customers)
    D, q, Q, R = inst.euclid, inst.demand, p.drone_capacity, p.drone_range
    U = list(range(num_drones))

    m = make_model("UAVRP", p, time_limit or p.time_limit_uavrp, log)
    arcs = [(d, i, j) for d in U for i in N for j in N if i != j]
    y = m.addVars(arcs, vtype=GRB.BINARY, name="y")
    v = m.addVars(((d, i) for d in U for i in C), lb=0.0, ub=Q, name="v")

    def into(d, j):
        return gp.quicksum(y[d, i, j] for i in N if i != j)

    def out(d, i):
        return gp.quicksum(y[d, i, j] for j in N if j != i)

    m.addConstrs((gp.quicksum(into(d, j) for d in U) == 1 for j in C), name="visit")             # (7)
    m.addConstrs((out(d, 0) == 1 for d in U), name="depart")                                       # (10)
    m.addConstrs((into(d, 0) == 1 for d in U), name="return")                                      # (10)
    m.addConstrs((out(d, i) - into(d, i) == 0 for d in U for i in C), name="flow")                 # (11)
    m.addConstrs((v[d, i] - v[d, j] + Q * y[d, i, j] <= Q - q[j]
                  for d in U for i in C for j in C if i != j), name="mtz")                          # (13)+(15)
    m.addConstrs((v[d, j] >= q[j] * into(d, j) for d in U for j in C), name="load_lb")             # (15)
    m.addConstrs((gp.quicksum(D[i][j] * y[d, i, j] for i in N for j in N if i != j) <= R
                  for d in U), name="range")                                                       # (16)
    # symmetry breaking between identical drones
    m.addConstrs((into(d, j) == 0 for d in U for j in C if j <= d), name="symmetry")

    dist = gp.quicksum(D[i][j] * y[d, i, j] for d, i, j in arcs)
    t_drone = (60.0 / p.drone_speed) * dist + fixed_time(p, "drone", num_drones)                    # (4)
    c_total = p.drone_cost * dist + num_drones * p.fixed_cost_drone                                # (6)
    m.setObjective(p.time_weight * p.time_value * t_drone + p.cost_weight * c_total, GRB.MINIMIZE)  # (1)

    warm_obj = None
    if warm_routes:
        for var in y.values():
            var.Start = 0.0
        for d, r in enumerate(canonical(warm_routes)[:num_drones]):
            for a, b in zip(r, r[1:]):
                y[d, a, b].Start = 1.0
            for i, load in cumulative_loads(r, q).items():
                v[d, i].Start = load
        warm_obj = evaluate(inst, [], warm_routes, 0, num_drones).efficiency

    m.optimize()
    if m.SolCount == 0:
        raise RuntimeError(f"UAVRP: no feasible solution ({status_name(m)})")

    routes = []
    for d in U:
        active = [(i, j) for (dd, i, j) in arcs if dd == d and y[dd, i, j].X > 0.5]
        routes.extend(routes_from_arcs(active, N))
    verify(inst, [], routes, 0, num_drones)
    return SolveResult(group="UAVRP", truck_routes=[], drone_routes=routes, objective=m.ObjVal,
                       bound=bound_of(m), gap=gap_of(m), runtime=m.Runtime, status=status_name(m),
                       warm_start_objective=warm_obj,
                       extra={"num_vars": m.NumVars, "num_constrs": m.NumConstrs})
