import random

import pytest

from vrpd.config import DEFAULT
from vrpd.heuristics import best_sweep_routes, nearest_neighbor, repair_drone_route, sweep_groups, two_opt
from vrpd.instance import build_instance
from vrpd.metrics import route_length


@pytest.fixture(scope="module")
def inst():
    return build_instance(DEFAULT)


def test_nearest_neighbor_closed_tour(inst):
    r = nearest_neighbor(range(1, 11), inst.manhattan)
    assert r[0] == 0 and r[-1] == 0
    assert sorted(r[1:-1]) == list(range(1, 11))
    assert nearest_neighbor([], inst.manhattan) == [0, 0]


def test_two_opt_never_lengthens(inst):
    rng = random.Random(1)
    for _ in range(20):
        custs = rng.sample(range(1, 51), 12)
        r = [0] + custs + [0]
        r2 = two_opt(r, inst.euclid)
        assert sorted(r2[1:-1]) == sorted(custs)
        assert route_length(r2, inst.euclid) <= route_length(r, inst.euclid) + 1e-12


def test_sweep_groups_partition(inst):
    groups = sweep_groups(inst.customers, inst.points, 4)
    assert len(groups) == 4
    assert sorted(i for g in groups for i in g) == list(inst.customers)
    assert max(map(len, groups)) - min(map(len, groups)) <= 1


def test_best_sweep_routes_cover_everyone(inst):
    routes = best_sweep_routes(inst.customers, inst.points, inst.manhattan, 5)
    assert len(routes) == 5
    assert sorted(i for r in routes for i in r[1:-1]) == list(inst.customers)


def test_repair_drone_route_enforces_limits(inst):
    p = inst.params
    far = sorted(inst.customers, key=lambda i: -inst.euclid[0][i])[:12]   # 48 kg, far too long
    r = nearest_neighbor(far, inst.euclid)
    fixed, removed = repair_drone_route(r, inst.euclid, inst.demand, p.drone_capacity, p.drone_range)
    assert sum(inst.demand[i] for i in fixed[1:-1]) <= p.drone_capacity
    assert route_length(fixed, inst.euclid) <= p.drone_range
    assert sorted(fixed[1:-1] + removed) == sorted(far)
    ok, none = repair_drone_route([0, 1, 0], inst.euclid, inst.demand, p.drone_capacity, p.drone_range)
    assert ok == [0, 1, 0] and none == []
