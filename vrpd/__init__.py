"""Truck–drone collaborative delivery: reproduction package for

    Gong, J. (2025). Research on Optimization of Urban Logistics Network Based on
    Low Altitude Economy: Path Planning and Traffic Impact Analysis of Collaborative
    Distribution Mode Between Drones and Ground Vehicles.

Three delivery groups share one instance, one parameter set and one evaluation
function so that their results are directly comparable (paper Section 3.1):

* CVRP  – pure ground-vehicle delivery
* UAVRP – pure drone delivery
* VRPD  – parallel truck + drone collaborative delivery
"""

from .config import Params, DEFAULT, FLEETS  # noqa: F401
from .instance import Instance, build_instance  # noqa: F401
from .metrics import Metrics, evaluate  # noqa: F401
from .verify import verify, InfeasibleSolution  # noqa: F401
