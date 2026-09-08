"""Independent feasibility check of a delivery plan against constraints (7)–(16).

Every solver and the genetic algorithm run their output through ``verify`` before a
number is reported, so a route that violates a capacity or range limit can never end
up in a result table.
"""

from __future__ import annotations

from collections import Counter

from .instance import Instance
from .metrics import route_length

TOL = 1e-6


class InfeasibleSolution(Exception):
    pass


def _check_route_shape(route, label, errors):
    if len(route) < 3:
        errors.append(f"{label}: route {route} must visit at least one customer (Eq. 8/10)")
        return
    if route[0] != 0 or route[-1] != 0:
        errors.append(f"{label}: route {route} must start and end at the depot (Eq. 8/10)")
    if 0 in route[1:-1]:
        errors.append(f"{label}: route {route} passes through the depot mid-route")
    inner = route[1:-1]
    if len(set(inner)) != len(inner):
        errors.append(f"{label}: route {route} visits a customer twice")


def verify(inst: Instance, truck_routes, drone_routes, num_trucks: int, num_drones: int,
           raise_on_error: bool = True) -> list:
    """Return a list of violations (empty when feasible). Raises by default."""
    p = inst.params
    errors = []

    if len(truck_routes) != num_trucks:
        errors.append(f"expected {num_trucks} truck routes, got {len(truck_routes)}")
    if len(drone_routes) != num_drones:
        errors.append(f"expected {num_drones} drone routes, got {len(drone_routes)}")

    for k, r in enumerate(truck_routes):
        _check_route_shape(r, f"truck {k}", errors)
        load = sum(inst.demand[i] for i in r[1:-1])
        if load > p.truck_capacity + TOL:
            errors.append(f"truck {k}: load {load:g} kg exceeds capacity {p.truck_capacity:g} kg (Eq. 14)")

    for d, r in enumerate(drone_routes):
        _check_route_shape(r, f"drone {d}", errors)
        load = sum(inst.demand[i] for i in r[1:-1])
        if load > p.drone_capacity + TOL:
            errors.append(f"drone {d}: load {load:g} kg exceeds capacity {p.drone_capacity:g} kg (Eq. 15)")
        dist = route_length(r, inst.euclid)
        if dist > p.drone_range + TOL:
            errors.append(f"drone {d}: flight {dist:.3f} km exceeds range {p.drone_range:g} km (Eq. 16)")

    visits = Counter()
    for r in list(truck_routes) + list(drone_routes):
        visits.update(r[1:-1])
    missing = sorted(set(inst.customers) - set(visits))
    duplicated = sorted(c for c, n in visits.items() if n > 1)
    unknown = sorted(c for c in visits if c not in inst.customers)
    if missing:
        errors.append(f"customers never served: {missing} (Eq. 7)")
    if duplicated:
        errors.append(f"customers served more than once: {duplicated} (Eq. 7)")
    if unknown:
        errors.append(f"unknown node ids in routes: {unknown}")

    if errors and raise_on_error:
        raise InfeasibleSolution("\n".join(errors))
    return errors
