"""Problem instance shared by all three delivery groups (paper Section 3.1 / 5.1).

The depot sits at the centre of a square study area and ``num_node`` customers are
drawn uniformly at random with a fixed seed.  Distances are pre-computed twice:

* ``manhattan`` – used for ground vehicles (2-D urban road network, Assumption 8)
* ``euclid``    – used for drones (straight-line flight, Assumption 8)

Both matrices are plain nested lists: the heuristics index them millions of times with
scalar subscripts, where lists are several times faster than NumPy arrays.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .config import Params, DEFAULT


@dataclass(frozen=True)
class Instance:
    points: list            # index 0 = depot, 1..n = customers, each (x, y) in km
    demand: list            # demand[0] = 0, demand[i] = kg required by customer i
    euclid: list            # D_ij
    manhattan: list         # M_ij
    params: Params

    @property
    def n(self) -> int:
        return len(self.points) - 1

    @property
    def customers(self) -> range:
        return range(1, self.n + 1)

    @property
    def nodes(self) -> range:
        return range(self.n + 1)


def generate_points(num_node: int, range_coordinate=(0.0, 1.54), random_seed: int = 2025):
    """Depot at the centre of the area, customers uniformly random.  The RNG sequence is the
    one the original scripts used, so the 50 customer coordinates of the case study are
    preserved exactly."""
    lo, hi = range_coordinate
    rng = random.Random(random_seed)
    customers = [(rng.uniform(lo, hi), rng.uniform(lo, hi)) for _ in range(num_node)]
    depot = ((hi - lo) / 2.0, (hi - lo) / 2.0)
    return [depot] + customers


def distance_matrices(points):
    n = len(points)
    euclid = [[0.0] * n for _ in range(n)]
    manhattan = [[0.0] * n for _ in range(n)]
    for i in range(n):
        xi, yi = points[i]
        for j in range(i + 1, n):
            dx, dy = xi - points[j][0], yi - points[j][1]
            e, m = math.hypot(dx, dy), abs(dx) + abs(dy)
            euclid[i][j] = euclid[j][i] = e
            manhattan[i][j] = manhattan[j][i] = m
    return euclid, manhattan


def build_instance(params: Params = DEFAULT) -> Instance:
    points = generate_points(params.num_node, params.range_coordinate, params.random_seed)
    demand = [0.0] + [params.demand_per_customer] * params.num_node
    euclid, manhattan = distance_matrices(points)
    return Instance(points=points, demand=demand, euclid=euclid, manhattan=manhattan, params=params)
