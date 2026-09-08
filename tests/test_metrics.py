import pytest

from vrpd.config import DEFAULT
from vrpd.instance import build_instance
from vrpd.metrics import evaluate

from .paper_tables import (PAPER_UAVRP_DRONES, PAPER_UAVRP_TABLE2, PAPER_VRPD_DRONES, PAPER_VRPD_TABLE2,
                           PAPER_VRPD_TRUCKS)


@pytest.fixture(scope="module")
def inst():
    return build_instance(DEFAULT)


@pytest.fixture(scope="module")
def inst_paper():
    """The paper's Table 2 adds the fixed service time once per mode."""
    return build_instance(DEFAULT.replace(fixed_time_per="mode"))


def test_reproduces_paper_table2_for_vrpd(inst_paper):
    """The published collaborative plan evaluates to exactly the published numbers."""
    m = evaluate(inst_paper, PAPER_VRPD_TRUCKS, PAPER_VRPD_DRONES, 4, 2)
    for key, want in PAPER_VRPD_TABLE2.items():
        assert getattr(m, key) == pytest.approx(want, abs=0.011), key
    assert m.fixed_cost == 250
    assert m.overall_time == max(m.truck_time, m.drone_time)


def test_reproduces_paper_table2_for_uavrp(inst_paper):
    m = evaluate(inst_paper, [], PAPER_UAVRP_DRONES, 0, 10)
    for key, want in PAPER_UAVRP_TABLE2.items():
        # the paper rounds intermediate values, so allow one unit in the second decimal
        assert getattr(m, key) == pytest.approx(want, abs=0.011), key
    assert m.truck_time == 0 and m.overall_time == m.drone_time


def test_efficiency_is_eq1(inst):
    m = evaluate(inst, PAPER_VRPD_TRUCKS, PAPER_VRPD_DRONES, 4, 2)
    p = inst.params
    assert abs(m.efficiency - (p.time_weight * p.time_value * m.overall_time + p.cost_weight * m.total_cost)) < 1e-9


def test_fixed_time_per_vehicle_is_default(inst):
    assert inst.params.fixed_time_per == "vehicle"
    p = inst.params
    one = evaluate(inst, [[0, 1, 0]], [], 1, 0)
    two = evaluate(inst, [[0, 1, 0], [0, 2, 0]], [], 2, 0)
    assert one.truck_time == pytest.approx(60 / p.truck_speed * one.truck_distance + 1 * p.fixed_time_truck)
    assert two.truck_time == pytest.approx(60 / p.truck_speed * two.truck_distance + 2 * p.fixed_time_truck)


def test_fixed_time_per_mode_reproduces_paper_arithmetic(inst_paper):
    p = inst_paper.params
    two = evaluate(inst_paper, [[0, 1, 0], [0, 2, 0]], [], 2, 0)
    assert two.truck_time == pytest.approx(60 / p.truck_speed * two.truck_distance + p.fixed_time_truck)


def test_fixed_time_readings_differ_only_by_fleet_size(inst, inst_paper):
    p = inst.params
    a = evaluate(inst_paper, PAPER_VRPD_TRUCKS, PAPER_VRPD_DRONES, 4, 2)   # per mode
    b = evaluate(inst, PAPER_VRPD_TRUCKS, PAPER_VRPD_DRONES, 4, 2)         # per vehicle
    assert b.truck_time == pytest.approx(a.truck_time + 3 * p.fixed_time_truck)   # 4 trucks instead of 1 term
    assert b.drone_time == pytest.approx(a.drone_time + 1 * p.fixed_time_drone)   # 2 drones instead of 1 term
    assert b.total_cost == a.total_cost                                             # costs untouched
    assert b.total_distance == a.total_distance
