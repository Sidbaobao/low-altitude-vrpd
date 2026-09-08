import pytest

from vrpd.config import DEFAULT
from vrpd.instance import build_instance
from vrpd.verify import InfeasibleSolution, verify

from .paper_tables import PAPER_UAVRP_DRONES, PAPER_VRPD_DRONES, PAPER_VRPD_TRUCKS


@pytest.fixture(scope="module")
def inst():
    return build_instance(DEFAULT)


def test_paper_vrpd_plan_is_feasible(inst):
    assert verify(inst, PAPER_VRPD_TRUCKS, PAPER_VRPD_DRONES, 4, 2) == []


def test_paper_uavrp_plan_violates_range_and_payload(inst):
    """The pure-drone baseline printed in the paper breaks the model's own limits."""
    errors = verify(inst, [], PAPER_UAVRP_DRONES, 0, 10, raise_on_error=False)
    assert any("exceeds range" in e for e in errors)
    assert any("exceeds capacity" in e for e in errors)
    with pytest.raises(InfeasibleSolution):
        verify(inst, [], PAPER_UAVRP_DRONES, 0, 10)


def test_detects_missing_and_duplicate_customers(inst):
    trucks = [r[:] for r in PAPER_VRPD_TRUCKS]
    trucks[0][1] = 2                       # now 2 is served twice and 38 never
    errors = verify(inst, trucks, PAPER_VRPD_DRONES, 4, 2, raise_on_error=False)
    assert any("never served: [38]" in e for e in errors)
    assert any("more than once: [2]" in e for e in errors)


def test_detects_fleet_size_and_shape(inst):
    errors = verify(inst, PAPER_VRPD_TRUCKS[:3], PAPER_VRPD_DRONES, 4, 2, raise_on_error=False)
    assert any("expected 4 truck routes" in e for e in errors)
    errors = verify(inst, [[0, 1, 2, 3]], [], 1, 0, raise_on_error=False)
    assert any("start and end at the depot" in e for e in errors)
    errors = verify(inst, [[0, 0]], [], 1, 0, raise_on_error=False)
    assert any("at least one customer" in e for e in errors)
