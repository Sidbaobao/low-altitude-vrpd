"""Genetic algorithm with warm start for the collaborative group (paper Sections 4.5–4.6).

Chromosome
----------
A bit per customer: ``0`` = served by a ground vehicle, ``1`` = served by a drone.  This is
the assignment decision of the VRPD; ``decode`` turns it into concrete routes:

* drone customers are cut into ``num_drones`` sorties by sweep + nearest neighbour + 2-opt
  and repaired until payload (Eq. 15) and range (Eq. 16) hold – customers dropped by the
  repair go back to the trucks and the chromosome is corrected accordingly (step 2 of 4.5);
* truck customers are cut into ``num_trucks`` routes the same way.

Fitness is the delivery-efficiency indicator Z of Eq. (1) computed by ``metrics.evaluate``
(lower is better), plus a large penalty for any constraint the verifier still reports.

Warm start (4.6)
----------------
``warm_start_seeds`` builds high-quality chromosomes from the pure-truck and pure-drone
MILP baselines and from nearest-neighbour constructions; ``run_ga`` injects them into the
initial population.
"""

from __future__ import annotations

import itertools
import random
from dataclasses import dataclass, field

from .heuristics import best_sweep_routes, nearest_neighbor, repair_drone_route, route_length, two_opt
from .instance import Instance
from .metrics import evaluate
from .verify import verify

PENALTY = 1.0e6


@dataclass
class GAResult:
    chromosome: list
    fitness: float
    truck_routes: list
    drone_routes: list
    generations: int
    history: list = field(default_factory=list)   # best fitness per generation


# ----------------------------------------------------------------------------- decoding

def _move(i, src, dst, chrom, bit):
    src.remove(i)
    dst.append(i)
    chrom[i - 1] = bit


def decode(chrom, inst: Instance, num_trucks: int, num_drones: int):
    """Chromosome -> (truck_routes, drone_routes, repaired_chromosome)."""
    p = inst.params
    chrom = list(chrom)
    drone_set = [i for i in inst.customers if chrom[i - 1] == 1]
    truck_set = [i for i in inst.customers if chrom[i - 1] == 0]

    if num_drones == 0:
        for i in list(drone_set):
            _move(i, drone_set, truck_set, chrom, 0)
    if num_trucks == 0:
        for i in list(truck_set):
            _move(i, truck_set, drone_set, chrom, 1)

    # Every drone and every truck leaves the depot exactly once (Eq. 8 / Eq. 10), so each
    # vehicle needs at least one customer.
    if num_drones and len(drone_set) < num_drones:
        for i in sorted(truck_set, key=lambda i: inst.euclid[0][i]):
            if len(drone_set) >= num_drones:
                break
            _move(i, truck_set, drone_set, chrom, 1)
    if num_trucks and len(truck_set) < num_trucks:
        for i in sorted(drone_set, key=lambda i: -inst.euclid[0][i]):
            if len(truck_set) >= num_trucks:
                break
            _move(i, drone_set, truck_set, chrom, 0)

    drone_routes = []
    if num_drones:
        for r in best_sweep_routes(drone_set, inst.points, inst.euclid, num_drones):
            fixed, removed = repair_drone_route(r, inst.euclid, inst.demand, p.drone_capacity, p.drone_range)
            for i in removed:
                _move(i, drone_set, truck_set, chrom, 0)
            drone_routes.append(fixed)
        # a sortie emptied by the repair still has to fly: hand it the nearest reachable customer
        for idx, r in enumerate(drone_routes):
            if len(r) >= 3:
                continue
            for i in sorted(truck_set, key=lambda i: inst.euclid[0][i]):
                if 2 * inst.euclid[0][i] <= p.drone_range and inst.demand[i] <= p.drone_capacity:
                    _move(i, truck_set, drone_set, chrom, 1)
                    drone_routes[idx] = [0, i, 0]
                    break

    truck_routes = best_sweep_routes(truck_set, inst.points, inst.manhattan, num_trucks) if num_trucks else []
    return truck_routes, drone_routes, chrom


class Evaluator:
    """Memoised decode + fitness (identical chromosomes are common after elitism)."""

    def __init__(self, inst: Instance, num_trucks: int, num_drones: int, cache_size: int = 50000):
        self.inst, self.num_trucks, self.num_drones = inst, num_trucks, num_drones
        self.cache = {}
        self.cache_size = cache_size

    def __call__(self, chrom):
        key = tuple(chrom)
        hit = self.cache.get(key)
        if hit is not None:
            return hit
        truck_routes, drone_routes, repaired = decode(chrom, self.inst, self.num_trucks, self.num_drones)
        errors = verify(self.inst, truck_routes, drone_routes, self.num_trucks, self.num_drones, raise_on_error=False)
        z = evaluate(self.inst, truck_routes, drone_routes, self.num_trucks, self.num_drones).efficiency
        result = (z + PENALTY * len(errors), truck_routes, drone_routes, repaired)
        if len(self.cache) >= self.cache_size:
            self.cache.clear()
        self.cache[key] = result
        self.cache[tuple(repaired)] = result
        return result


# ----------------------------------------------------------------------------- warm start

def chromosome_from_routes(inst: Instance, drone_routes) -> list:
    air = {i for r in drone_routes for i in r[1:-1]}
    return [1 if i in air else 0 for i in inst.customers]


def seed_from_baselines(inst: Instance, uavrp_routes, num_trucks: int, num_drones: int, keep: int = 3) -> list:
    """Combine the MILP baselines (Section 4.6 step 1): keep the ``num_drones`` best sorties of
    the pure-drone solution as the drone part, the remaining customers go to the trucks."""
    if not num_drones or len(uavrp_routes) < num_drones:
        return []
    ev = Evaluator(inst, num_trucks, num_drones)
    scored = []
    for combo in itertools.combinations(range(len(uavrp_routes)), num_drones):
        chrom = chromosome_from_routes(inst, [uavrp_routes[i] for i in combo])
        scored.append((ev(chrom)[0], chrom))
    scored.sort(key=lambda t: t[0])
    return [c for _, c in scored[:keep]]


def seed_air_advantage(inst: Instance, num_drones: int) -> list:
    """Give the drones the customers for which straight-line flight saves the most road
    distance (largest Manhattan-minus-Euclidean detour)."""
    p = inst.params
    max_air = num_drones * int(p.drone_capacity // p.demand_per_customer)
    ranked = sorted(inst.customers, key=lambda i: inst.manhattan[0][i] - inst.euclid[0][i], reverse=True)
    air = set(ranked[:max_air])
    return [1 if i in air else 0 for i in inst.customers]


def seed_nearest_neighbor(inst: Instance, num_drones: int) -> list:
    """Nearest-neighbour construction (Section 4.4): grow each drone sortie greedily from the
    depot while payload and range allow; everything else is served by trucks."""
    p = inst.params
    remaining = set(inst.customers)
    air = set()
    for _ in range(num_drones):
        route, load = [0], 0.0
        while True:
            cur = route[-1]
            cands = [j for j in remaining if load + inst.demand[j] <= p.drone_capacity]
            cands.sort(key=lambda j: inst.euclid[cur][j])
            placed = False
            for j in cands:
                trial = two_opt(route + [j, 0], inst.euclid)
                if route_length(trial, inst.euclid) <= p.drone_range:
                    route.append(j)
                    load += inst.demand[j]
                    remaining.remove(j)
                    placed = True
                    break
            if not placed:
                break
        air.update(route[1:])
    return [1 if i in air else 0 for i in inst.customers]


def warm_start_seeds(inst: Instance, num_trucks: int, num_drones: int, uavrp_routes=None) -> list:
    seeds = []
    if uavrp_routes:
        seeds += seed_from_baselines(inst, uavrp_routes, num_trucks, num_drones)
    seeds.append(seed_air_advantage(inst, num_drones))
    seeds.append(seed_nearest_neighbor(inst, num_drones))
    seeds.append([0] * inst.n)          # all-truck; decode moves the minimum to the drones
    return seeds


# ----------------------------------------------------------------------------- operators

def tournament(pop, fits, rng: random.Random, size: int):
    best = None
    for _ in range(size):
        i = rng.randrange(len(pop))
        if best is None or fits[i] < fits[best]:
            best = i
    return pop[best]


def one_point_crossover(a, b, rng: random.Random):
    n = len(a)
    if n < 2:
        return a[:], b[:]
    cut = rng.randint(1, n - 1)
    return a[:cut] + b[cut:], b[:cut] + a[cut:]


def mutate(chrom, rate: float, rng: random.Random):
    return [1 - g if rng.random() < rate else g for g in chrom]


def local_search(chrom, fit, ev: Evaluator, rng: random.Random, iters: int):
    """Step 7 of Section 4.5: flip one bit or swap a truck/drone pair, keep improvements."""
    best, best_fit = list(chrom), fit
    n = len(best)
    for _ in range(iters):
        cand = list(best)
        ones = [i for i, g in enumerate(cand) if g == 1]
        zeros = [i for i, g in enumerate(cand) if g == 0]
        if ones and zeros and rng.random() < 0.5:
            i, j = rng.choice(ones), rng.choice(zeros)
            cand[i], cand[j] = 0, 1
        else:
            i = rng.randrange(n)
            cand[i] = 1 - cand[i]
        f = ev(cand)[0]
        if f < best_fit - 1e-12:
            best, best_fit = ev(cand)[3], f
    return best, best_fit


# ----------------------------------------------------------------------------- main loop

def run_ga(inst: Instance, num_trucks: int, num_drones: int, seeds=(), log=None) -> GAResult:
    p = inst.params
    rng = random.Random(p.ga_seed)
    ev = Evaluator(inst, num_trucks, num_drones)
    n = inst.n
    max_air = num_drones * int(p.drone_capacity // p.demand_per_customer) if num_drones else 0

    # step 1: initial population = warm-start seeds + random individuals
    pop = [list(s) for s in seeds][:p.ga_population]
    while len(pop) < p.ga_population:
        k = rng.randint(min(num_drones, max_air), max_air) if num_drones else 0
        ones = set(rng.sample(range(n), k))
        pop.append([1 if i in ones else 0 for i in range(n)])

    # steps 2–3: repair + fitness
    evaluated = [ev(c) for c in pop]
    pop = [e[3] for e in evaluated]
    fits = [e[0] for e in evaluated]

    best_i = min(range(len(pop)), key=fits.__getitem__)
    best, best_fit = pop[best_i], fits[best_i]
    history, stale, gen = [best_fit], 0, 0

    for gen in range(1, p.ga_generations + 1):
        ranked = sorted(range(len(pop)), key=fits.__getitem__)
        elites = [pop[i] for i in ranked[:p.ga_elite]]
        elite_fits = [fits[i] for i in ranked[:p.ga_elite]]

        # steps 4–6: selection, crossover, mutation
        offspring = []
        while len(offspring) < p.ga_population - p.ga_elite:
            a = tournament(pop, fits, rng, p.ga_tournament_size)
            b = tournament(pop, fits, rng, p.ga_tournament_size)
            if rng.random() < p.ga_crossover_rate:
                c1, c2 = one_point_crossover(a, b, rng)
            else:
                c1, c2 = a[:], b[:]
            offspring.append(mutate(c1, p.ga_mutation_rate, rng))
            offspring.append(mutate(c2, p.ga_mutation_rate, rng))
        offspring = offspring[:p.ga_population - p.ga_elite]
        off_eval = [ev(c) for c in offspring]
        offspring = [e[3] for e in off_eval]
        off_fits = [e[0] for e in off_eval]

        # step 7: local search on the most promising offspring
        for idx in sorted(range(len(offspring)), key=off_fits.__getitem__)[:p.ga_elite]:
            offspring[idx], off_fits[idx] = local_search(offspring[idx], off_fits[idx], ev, rng,
                                                         p.ga_local_search_iters)

        # step 8: elitist replacement
        pop = elites + offspring
        fits = elite_fits + off_fits

        gen_best = min(range(len(pop)), key=fits.__getitem__)
        if fits[gen_best] < best_fit - 1e-9:
            best, best_fit, stale = pop[gen_best], fits[gen_best], 0
        else:
            stale += 1
        history.append(best_fit)
        if log and (gen % 10 == 0 or gen == 1):
            log(f"  GA gen {gen:4d}  best Z = {best_fit:.4f}")
        if stale >= p.ga_patience:
            break

    _, truck_routes, drone_routes, best = ev(best)
    return GAResult(chromosome=best, fitness=best_fit, truck_routes=truck_routes,
                    drone_routes=drone_routes, generations=gen, history=history)
