"""Constructive and local-search heuristics (paper Section 4.4 and step 7 of Section 4.5).

* ``nearest_neighbor`` – the TSP nearest-neighbour rule used to build initial routes.
* ``two_opt``          – the local search applied to every constructed route.
* ``sweep_groups``     – splits a customer set into k angular sectors around the depot, which
                          is how a set of customers is distributed over identical vehicles.
* ``repair_drone_route`` – shortens a drone sortie until payload and range limits hold.
"""

from __future__ import annotations

import math

from .metrics import route_length


def nearest_neighbor(customers, matrix, depot: int = 0) -> list:
    """Greedy closed tour depot -> nearest unvisited -> ... -> depot (Section 4.4)."""
    remaining = set(customers)
    if not remaining:
        return [depot, depot]
    route = [depot]
    current = depot
    while remaining:
        nxt = min(remaining, key=lambda j: (matrix[current][j], j))
        route.append(nxt)
        remaining.remove(nxt)
        current = nxt
    route.append(depot)
    return route


def two_opt(route, matrix) -> list:
    """First-improvement 2-opt on a closed route.  Never increases the route length."""
    best = list(route)
    n = len(best)
    if n <= 4:
        return best
    improved = True
    while improved:
        improved = False
        for i in range(1, n - 2):
            for j in range(i + 1, n - 1):
                a, b = best[i - 1], best[i]
                c, d = best[j], best[j + 1]
                delta = matrix[a][c] + matrix[b][d] - matrix[a][b] - matrix[c][d]
                if delta < -1e-9:
                    best[i:j + 1] = best[i:j + 1][::-1]
                    improved = True
    return best


def build_route(customers, matrix) -> list:
    return two_opt(nearest_neighbor(customers, matrix), matrix)


def sweep_groups(customers, points, k: int, offset: float = 0.0, depot: int = 0) -> list:
    """Sort customers by polar angle around the depot and cut into k consecutive groups of
    (almost) equal size.  ``offset`` rotates where the first cut is made."""
    customers = list(customers)
    if k <= 0:
        return []
    cx, cy = points[depot]

    def angle(i):
        return (math.atan2(points[i][1] - cy, points[i][0] - cx) - offset) % (2 * math.pi)

    order = sorted(customers, key=lambda i: (angle(i), i))
    base, rem = divmod(len(order), k)
    groups, start = [], 0
    for g in range(k):
        size = base + (1 if g < rem else 0)
        groups.append(order[start:start + size])
        start += size
    return groups


def best_sweep_routes(customers, points, matrix, k: int, n_offsets: int = 8) -> list:
    """Try several sweep offsets, build NN + 2-opt routes for each, keep the shortest set."""
    customers = list(customers)
    if k <= 0:
        return []
    best, best_len = None, math.inf
    for t in range(max(1, n_offsets)):
        groups = sweep_groups(customers, points, k, offset=2 * math.pi * t / (n_offsets * k))
        routes = [build_route(g, matrix) for g in groups]
        total = sum(route_length(r, matrix) for r in routes)
        if total < best_len - 1e-12:
            best, best_len = routes, total
    return best


def repair_drone_route(route, euclid, demand, capacity, drone_range):
    """Drop customers from a drone sortie until it satisfies Eq. (15) and (16).

    The removed customer is always the one whose removal shortens the tour the most, so
    the tour stays as useful as possible.  Returns ``(feasible_route, removed_customers)``.
    """
    route = list(route)
    removed = []
    while True:
        inner = route[1:-1]
        load = sum(demand[i] for i in inner)
        length = route_length(route, euclid)
        if load <= capacity + 1e-9 and length <= drone_range + 1e-9:
            return route, removed
        if len(inner) <= 1:
            # a single customer that is out of range cannot be served by a drone at all
            removed.extend(inner)
            return [0, 0], removed
        best_i, best_gain = None, -math.inf
        for pos in range(1, len(route) - 1):
            a, b, c = route[pos - 1], route[pos], route[pos + 1]
            gain = euclid[a][b] + euclid[b][c] - euclid[a][c]
            if gain > best_gain:
                best_i, best_gain = pos, gain
        removed.append(route.pop(best_i))
        route = two_opt(route, euclid)
