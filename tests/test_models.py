"""End-to-end checks of the three Gurobi formulations on the 12-customer QUICK configuration.

Skipped automatically when no usable Gurobi license is present."""

import pytest

from vrpd.config import FLEETS_QUICK, QUICK
from vrpd.ga import run_ga, warm_start_seeds
from vrpd.heuristics import best_sweep_routes
from vrpd.instance import build_instance
from vrpd.metrics import evaluate
from vrpd.verify import verify

gp = pytest.importorskip("gurobipy")


def _license_ok():
    try:
        env = gp.Env(params={"OutputFlag": 0})
        env.dispose()
        return True
    except gp.GurobiError:
        return False


pytestmark = [pytest.mark.gurobi, pytest.mark.skipif(not _license_ok(), reason="no Gurobi license")]

from vrpd.models import solve_cvrp, solve_uavrp, solve_vrpd  # noqa: E402


@pytest.fixture(scope="module")
def inst():
    return build_instance(QUICK)


def test_cvrp(inst):
    K, _ = FLEETS_QUICK["CVRP"]
    warm = best_sweep_routes(inst.customers, inst.points, inst.manhattan, K)
    res = solve_cvrp(inst, K, warm_routes=warm)
    assert verify(inst, res.truck_routes, [], K, 0) == []
    m = evaluate(inst, res.truck_routes, [], K, 0)
    assert res.objective == pytest.approx(m.efficiency, abs=1e-6)     # solver objective == Eq. (1)
    assert res.objective <= res.warm_start_objective + 1e-6


def test_uavrp(inst):
    _, D = FLEETS_QUICK["UAVRP"]
    warm = best_sweep_routes(inst.customers, inst.points, inst.euclid, D)
    res = solve_uavrp(inst, D, warm_routes=warm)
    assert verify(inst, [], res.drone_routes, 0, D) == []
    m = evaluate(inst, [], res.drone_routes, 0, D)
    assert res.objective == pytest.approx(m.efficiency, abs=1e-6)


def test_vrpd_two_stage(inst):
    K, D = FLEETS_QUICK["VRPD"]
    seeds = warm_start_seeds(inst, K, D)
    ga = run_ga(inst, K, D, seeds=seeds)
    res = solve_vrpd(inst, K, D, warm_truck_routes=ga.truck_routes, warm_drone_routes=ga.drone_routes)
    assert verify(inst, res.truck_routes, res.drone_routes, K, D) == []
    m = evaluate(inst, res.truck_routes, res.drone_routes, K, D)
    assert res.objective == pytest.approx(m.efficiency, abs=1e-6)
    assert res.objective <= ga.fitness + 1e-6                          # MILP never worse than its warm start
