import math

from vrpd.config import DEFAULT, FLEETS
from vrpd.instance import build_instance, generate_points


def test_case_study_geometry():
    inst = build_instance(DEFAULT)
    assert inst.n == 50
    assert inst.points[0] == (0.77, 0.77)                      # depot at the centre of 1.54 x 1.54 km
    lo, hi = DEFAULT.range_coordinate
    assert all(lo <= x <= hi and lo <= y <= hi for x, y in inst.points[1:])
    assert abs((hi - lo) ** 2 - 2.37) < 0.01                    # 2.37 km^2 study area


def test_fixed_uniform_demand():
    inst = build_instance(DEFAULT)
    assert inst.demand[0] == 0
    assert set(inst.demand[1:]) == {4.0}
    assert sum(inst.demand) == 200


def test_same_fixed_cost_in_every_group():
    cost = {g: k * DEFAULT.fixed_cost_truck + d * DEFAULT.fixed_cost_drone for g, (k, d) in FLEETS.items()}
    assert cost == {"CVRP": 250, "UAVRP": 250, "VRPD": 250}


def test_deterministic_and_independent_of_call_order():
    a = generate_points(50, (0, 1.54), 2025)
    b = generate_points(50, (0, 1.54), 2025)
    assert a == b
    # regenerating a different instance in between must not disturb the sequence
    generate_points(7, (0, 1.0), 1)
    assert generate_points(50, (0, 1.54), 2025) == a


def test_distance_matrices():
    inst = build_instance(DEFAULT)
    n = inst.n + 1
    for i in range(n):
        assert inst.euclid[i][i] == 0 and inst.manhattan[i][i] == 0
        for j in range(i + 1, n):
            assert inst.euclid[i][j] == inst.euclid[j][i]
            assert inst.manhattan[i][j] == inst.manhattan[j][i]
            assert inst.manhattan[i][j] >= inst.euclid[i][j] - 1e-12
            (xi, yi), (xj, yj) = inst.points[i], inst.points[j]
            assert math.isclose(inst.euclid[i][j], math.hypot(xi - xj, yi - yj))
