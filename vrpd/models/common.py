"""Shared plumbing for the three Gurobi formulations (paper Section 4.3)."""

from __future__ import annotations

from dataclasses import dataclass, field

import gurobipy as gp
from gurobipy import GRB

from ..config import Params

STATUS = {
    GRB.OPTIMAL: "OPTIMAL",
    GRB.TIME_LIMIT: "TIME_LIMIT",
    GRB.INFEASIBLE: "INFEASIBLE",
    GRB.INF_OR_UNBD: "INF_OR_UNBD",
    GRB.INTERRUPTED: "INTERRUPTED",
    GRB.SUBOPTIMAL: "SUBOPTIMAL",
}


@dataclass
class SolveResult:
    group: str
    truck_routes: list
    drone_routes: list
    objective: float          # value of Eq. (1) for the returned incumbent
    bound: float              # best proven lower bound on Eq. (1)
    gap: float                # relative MIP gap reported by Gurobi
    runtime: float            # seconds
    status: str
    warm_start_objective: float | None = None
    extra: dict = field(default_factory=dict)


def make_model(name: str, params: Params, time_limit: float, log: bool) -> gp.Model:
    m = gp.Model(name)
    m.Params.OutputFlag = 1 if log else 0
    m.Params.TimeLimit = time_limit
    m.Params.MIPGap = params.mip_gap
    if params.solver_threads:
        m.Params.Threads = params.solver_threads
    return m


def status_name(model: gp.Model) -> str:
    return STATUS.get(model.Status, str(model.Status))


def follow(succ, start: int = 0) -> list:
    """Walk successor arcs from the depot until the depot is reached again."""
    route, cur = [start], start
    while True:
        nxt = succ.get(cur)
        if nxt is None:
            break
        route.append(nxt)
        cur = nxt
        if cur == start or len(route) > len(succ) + 2:
            break
    return route


def routes_from_arcs(active_arcs, nodes) -> list:
    """Split a set of active (i, j) arcs on one vehicle class into depot-anchored routes."""
    out_of_depot = sorted(j for (i, j) in active_arcs if i == 0)
    succ = {i: j for (i, j) in active_arcs if i != 0}
    routes = []
    for first in out_of_depot:
        routes.append([0] + follow(succ | {0: first}, 0)[1:])
    return routes


def canonical(routes) -> list:
    """Order routes by their smallest customer id.  Identical vehicles are interchangeable,
    so the models fix this order to break symmetry; a warm start must respect it."""
    nonempty = [r for r in routes if len(r) >= 3]
    empty = [r for r in routes if len(r) < 3]
    return sorted(nonempty, key=lambda r: min(r[1:-1])) + empty


def cumulative_loads(route, demand) -> dict:
    loads, acc = {}, 0.0
    for i in route[1:-1]:
        acc += demand[i]
        loads[i] = acc
    return loads


def gap_of(model: gp.Model) -> float:
    try:
        return float(model.MIPGap)
    except (AttributeError, gp.GurobiError):
        return float("nan")


def bound_of(model: gp.Model) -> float:
    try:
        return float(model.ObjBound)
    except (AttributeError, gp.GurobiError):
        return float("nan")
