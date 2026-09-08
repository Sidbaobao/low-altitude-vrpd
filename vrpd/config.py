"""Single source of truth for every parameter used by the three delivery groups.

All values follow Section 5.1 of the paper.  Units throughout the package:

* distance  – kilometres
* time      – minutes
* money     – 10,000 CNY (so ``TRUCK_COST = 0.8`` means 8,000 CNY per km)

Having one ``Params`` object shared by CVRP, UAVRP and VRPD is what guarantees
the "same fixed costs, same demand, same capacities" fairness condition stated
in Section 3.1.  Tests build smaller instances through ``Params.replace``.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass


@dataclass(frozen=True)
class Params:
    # ---- instance (Section 5.1) ------------------------------------------------
    num_node: int = 50                       # customers, depot is node 0
    range_coordinate: tuple = (0.0, 1.54)    # 1.54 km x 1.54 km = 2.37 km^2 study area
    random_seed: int = 2025
    demand_per_customer: float = 4.0         # kg, fixed for every customer (Assumption 9)

    # ---- vehicle physics -------------------------------------------------------
    truck_capacity: float = 60.0             # kg
    drone_capacity: float = 20.0             # kg
    drone_range: float = 5.0                 # km, total flight distance per drone (Eq. 16)
    truck_speed: float = 20.0                # km/h
    drone_speed: float = 60.0                # km/h
    fixed_time_truck: float = 5.0            # min (Eq. 3)
    fixed_time_drone: float = 4.0            # min (Eq. 4)
    #: How the fixed service time enters T_truck / T_drone of Eq. (3)/(4):
    #: "vehicle" – every dispatched vehicle incurs its own loading / hand-over time, following
    #:            Assumption 6 ("each delivery tool consumes a fixed delivery time").  This is
    #:            the accounting used for the results in this repository.
    #: "mode"    – one additive term per mode regardless of fleet size; this is the arithmetic
    #:            behind Table 2 of the paper (33.48 = 29.48 + 4) and is used by the tests that
    #:            reproduce that table.
    #: Routes are identical under both readings because the fleet size of every group is fixed:
    #: the term is a constant in the objective and only the reported times and Z change.
    fixed_time_per: str = "vehicle"

    # ---- economics -------------------------------------------------------------
    truck_cost: float = 0.8                  # 10k CNY per km   (8,000 CNY/km)
    drone_cost: float = 1.0                  # 10k CNY per km   (10,000 CNY/km)
    fixed_cost_truck: float = 50.0           # 10k CNY per truck (500,000 CNY)
    fixed_cost_drone: float = 25.0           # 10k CNY per drone (250,000 CNY)
    time_weight: float = 0.8                 # Eq. (1)
    cost_weight: float = 0.2                 # Eq. (1)
    time_value: float = 0.1                  # 10k CNY per minute (1,000 CNY/min)

    # ---- solver (Section 4.3) --------------------------------------------------
    time_limit_cvrp: float = 900.0
    time_limit_uavrp: float = 900.0
    time_limit_vrpd: float = 1800.0
    mip_gap: float = 0.01
    solver_threads: int = 0                  # 0 = let Gurobi decide

    # ---- genetic algorithm (Sections 4.5, 4.6) ---------------------------------
    ga_population: int = 100
    ga_generations: int = 300
    ga_crossover_rate: float = 0.8
    ga_mutation_rate: float = 0.05
    ga_tournament_size: int = 3
    ga_elite: int = 5
    ga_local_search_iters: int = 20
    ga_patience: int = 60                    # stop after this many generations without improvement
    ga_seed: int = 2025

    def replace(self, **changes) -> "Params":
        return dataclasses.replace(self, **changes)


DEFAULT = Params()

#: Fleet sizes (trucks, drones) per delivery group.  Chosen in the paper so that every
#: group carries the same fixed cost: 5*50 = 10*25 = 4*50 + 2*25 = 250 (10k CNY).
FLEETS = {
    "CVRP": (5, 0),
    "UAVRP": (0, 10),
    "VRPD": (4, 2),
}

#: A small configuration used by ``run_all.py --quick`` and by the test-suite so the whole
#: pipeline (including Gurobi) can be exercised in well under a minute.
QUICK = DEFAULT.replace(
    num_node=12,
    time_limit_cvrp=60.0,
    time_limit_uavrp=60.0,
    time_limit_vrpd=90.0,
    ga_population=30,
    ga_generations=40,
    ga_patience=15,
)
FLEETS_QUICK = {
    "CVRP": (2, 0),
    "UAVRP": (0, 3),
    "VRPD": (1, 1),
}
