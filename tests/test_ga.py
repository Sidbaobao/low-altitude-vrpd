import random

import pytest

from vrpd.config import DEFAULT, QUICK
from vrpd.ga import decode, run_ga, seed_air_advantage, seed_nearest_neighbor, warm_start_seeds
from vrpd.instance import build_instance
from vrpd.metrics import evaluate
from vrpd.verify import verify


@pytest.fixture(scope="module")
def inst():
    return build_instance(DEFAULT)


def test_decode_always_feasible(inst):
    rng = random.Random(7)
    for _ in range(30):
        chrom = [rng.choice([0, 1]) for _ in range(inst.n)]
        t, d, repaired = decode(chrom, inst, 4, 2)
        assert verify(inst, t, d, 4, 2) == []
        assert len(t) == 4 and len(d) == 2
        # the repaired chromosome describes the decoded routes
        air = {i for r in d for i in r[1:-1]}
        assert [1 if i in air else 0 for i in inst.customers] == repaired


def test_decode_all_truck_chromosome_still_flies_every_drone(inst):
    t, d, _ = decode([0] * inst.n, inst, 4, 2)
    assert verify(inst, t, d, 4, 2) == []
    assert all(len(r) >= 3 for r in d)


def test_seeds_are_feasible_after_decoding(inst):
    for chrom in (seed_air_advantage(inst, 2), seed_nearest_neighbor(inst, 2)):
        t, d, _ = decode(chrom, inst, 4, 2)
        assert verify(inst, t, d, 4, 2) == []


def test_ga_small_instance_improves_on_seeds():
    inst = build_instance(QUICK.replace(ga_population=20, ga_generations=15, ga_patience=10))
    seeds = warm_start_seeds(inst, 1, 1)
    seed_best = min(evaluate(inst, *decode(s, inst, 1, 1)[:2], 1, 1).efficiency for s in seeds)
    res = run_ga(inst, 1, 1, seeds=seeds)
    assert verify(inst, res.truck_routes, res.drone_routes, 1, 1) == []
    assert res.fitness <= seed_best + 1e-9
    assert res.fitness == pytest.approx(evaluate(inst, res.truck_routes, res.drone_routes, 1, 1).efficiency)
    assert res.history == sorted(res.history, reverse=True)   # best-so-far never gets worse
