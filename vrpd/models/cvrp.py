"""Pure ground-vehicle group – Capacitated VRP (paper Eq. 1, 2, 3, 5, 7, 8, 9, 12, 14, 17).

Formulation notes
-----------------
* All trucks are identical, so the vehicle index ``k`` of Eq. (8), (9), (12), (14) is
  aggregated: ``x[i, j]`` = 1 if *some* truck drives i -> j, and the depot has exactly
  ``num_trucks`` outgoing and incoming arcs.  The feasible set is unchanged and the model is
  far smaller (n^2 instead of K n^2 binaries).
* Sub-tour elimination uses the load form of the MTZ constraints, which folds the capacity
  limit Eq. (14) into Eq. (12):  u_i - u_j + Q x_ij <= Q - q_j with q_i <= u_i <= Q.
* The objective is Eq. (1) written out for a single mode; the fixed cost and fixed time are
  constants, so this is the same optimum as minimising travel distance.
"""

from __future__ import annotations

import gurobipy as gp
from gurobipy import GRB

from ..instance import Instance
from ..metrics import evaluate, fixed_time
from ..verify import verify
from .common import SolveResult, bound_of, gap_of, make_model, routes_from_arcs, status_name


def solve_cvrp(inst: Instance, num_trucks: int, warm_routes=None, time_limit: float | None = None,
               log: bool = False) -> SolveResult:
    p = inst.params
    N, C = list(inst.nodes), list(inst.customers)
    M, q, Q = inst.manhattan, inst.demand, p.truck_capacity
    K = num_trucks

    m = make_model("CVRP", p, time_limit or p.time_limit_cvrp, log)
    arcs = [(i, j) for i in N for j in N if i != j]
    x = m.addVars(arcs, vtype=GRB.BINARY, name="x")
    u = m.addVars(C, lb=[q[i] for i in C], ub=Q, name="u")

    m.addConstrs((gp.quicksum(x[i, j] for i in N if i != j) == 1 for j in C), name="visit")        # (7)
    m.addConstrs((gp.quicksum(x[i, j] for j in N if j != i) == 1 for i in C), name="flow")         # (9)
    m.addConstr(gp.quicksum(x[0, j] for j in C) == K, name="depart")                               # (8)
    m.addConstr(gp.quicksum(x[i, 0] for i in C) == K, name="return")                               # (8)
    m.addConstrs((u[i] - u[j] + Q * x[i, j] <= Q - q[j] for i in C for j in C if i != j), name="mtz")  # (12)+(14)

    dist = gp.quicksum(M[i][j] * x[i, j] for i, j in arcs)
    t_truck = (60.0 / p.truck_speed) * dist + fixed_time(p, "truck", K)                              # (3)
    c_total = p.truck_cost * dist + K * p.fixed_cost_truck                                          # (5)
    m.setObjective(p.time_weight * p.time_value * t_truck + p.cost_weight * c_total, GRB.MINIMIZE)  # (1)

    warm_obj = None
    if warm_routes:
        for v in x.values():
            v.Start = 0.0
        for r in warm_routes:
            for a, b in zip(r, r[1:]):
                x[a, b].Start = 1.0
        warm_obj = evaluate(inst, warm_routes, [], K, 0).efficiency

    m.optimize()
    if m.SolCount == 0:
        raise RuntimeError(f"CVRP: no feasible solution ({status_name(m)})")

    active = [(i, j) for (i, j) in arcs if x[i, j].X > 0.5]
    routes = routes_from_arcs(active, N)
    verify(inst, routes, [], K, 0)
    return SolveResult(group="CVRP", truck_routes=routes, drone_routes=[], objective=m.ObjVal,
                       bound=bound_of(m), gap=gap_of(m), runtime=m.Runtime, status=status_name(m),
                       warm_start_objective=warm_obj,
                       extra={"num_vars": m.NumVars, "num_constrs": m.NumConstrs})
