"""Evaluation indicators of a delivery plan – paper Eq. (1)–(6).

    Z        = TIME_WEIGHT * TIME_VALUE * T_overall + COST_WEIGHT * C_total       (1)
    T_overall = max{T_truck, T_drone}                                              (2)
    T_truck  = 60/TRUCK_SPEED * sum_k sum_ij x_kij M_ij + FIXED_TIME_TRUCK        (3)
    T_drone  = 60/DRONE_SPEED * sum_d sum_ij y_dij D_ij + FIXED_TIME_DRONE        (4)
    C_truck  = sum_k sum_ij x_kij M_ij TRUCK_COST + |V| FIXED_COST_TRUCK          (5)
    C_drone  = sum_d sum_ij y_dij D_ij DRONE_COST + |U| FIXED_COST_DRONE          (6)

Notes
-----
* With ``fixed_time_per="mode"`` (default) the fixed service time is a single additive term
  per mode, exactly as printed in Eq. (3)/(4); ``"vehicle"`` charges it once per vehicle.
* For the single-mode groups Eq. (2) reduces to the time of the mode that is present.
* ``time_value`` converts minutes to 10k CNY so that both terms of Eq. (1) share one unit;
  the smaller ``Z`` is, the higher the delivery efficiency.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict

from .instance import Instance


@dataclass(frozen=True)
class Metrics:
    num_trucks: int
    num_drones: int
    truck_distance: float
    drone_distance: float
    total_distance: float
    truck_time: float
    drone_time: float
    overall_time: float
    truck_travel_cost: float
    drone_travel_cost: float
    travel_cost: float
    fixed_cost: float
    total_cost: float
    efficiency: float

    def as_dict(self) -> dict:
        return asdict(self)


def fixed_time(p, mode: str, fleet_size: int) -> float:
    """Total fixed service time of one mode – see ``Params.fixed_time_per``."""
    per_unit = p.fixed_time_truck if mode == "truck" else p.fixed_time_drone
    if p.fixed_time_per == "vehicle":
        return per_unit * fleet_size
    if p.fixed_time_per == "mode":
        return per_unit
    raise ValueError(f"unknown fixed_time_per={p.fixed_time_per!r}")


def route_length(route, matrix) -> float:
    return float(sum(matrix[a][b] for a, b in zip(route, route[1:])))


def evaluate(inst: Instance, truck_routes, drone_routes, num_trucks: int, num_drones: int) -> Metrics:
    p = inst.params
    truck_distance = sum(route_length(r, inst.manhattan) for r in truck_routes)
    drone_distance = sum(route_length(r, inst.euclid) for r in drone_routes)

    truck_time = (60.0 / p.truck_speed) * truck_distance + fixed_time(p, "truck", num_trucks) if num_trucks else 0.0
    drone_time = (60.0 / p.drone_speed) * drone_distance + fixed_time(p, "drone", num_drones) if num_drones else 0.0
    overall_time = max(truck_time, drone_time)

    truck_travel_cost = p.truck_cost * truck_distance
    drone_travel_cost = p.drone_cost * drone_distance
    fixed_cost = num_trucks * p.fixed_cost_truck + num_drones * p.fixed_cost_drone
    travel_cost = truck_travel_cost + drone_travel_cost
    total_cost = travel_cost + fixed_cost

    efficiency = p.time_weight * p.time_value * overall_time + p.cost_weight * total_cost

    return Metrics(
        num_trucks=num_trucks,
        num_drones=num_drones,
        truck_distance=truck_distance,
        drone_distance=drone_distance,
        total_distance=truck_distance + drone_distance,
        truck_time=truck_time,
        drone_time=drone_time,
        overall_time=overall_time,
        truck_travel_cost=truck_travel_cost,
        drone_travel_cost=drone_travel_cost,
        travel_cost=travel_cost,
        fixed_cost=fixed_cost,
        total_cost=total_cost,
        efficiency=efficiency,
    )
