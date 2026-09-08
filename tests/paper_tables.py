"""Routes and figures published in Table 1 / Table 2 of the paper (Section 5.2).

They are used by the tests to prove that this package evaluates a plan exactly the way the
paper does, and to show that the verifier catches the published pure-drone baseline, whose
sorties exceed the 5 km range and 20 kg payload the model prescribes.
"""

# VRPD, collaborative group – 4 trucks + 2 drones
PAPER_VRPD_TRUCKS = [
    [0, 38, 46, 10, 16, 3, 50, 13, 19, 32, 39, 0],
    [0, 37, 29, 15, 41, 45, 33, 21, 27, 30, 42, 0],
    [0, 22, 35, 34, 23, 49, 12, 20, 25, 24, 1, 0],
    [0, 36, 14, 31, 47, 43, 9, 40, 28, 4, 17, 0],
]
PAPER_VRPD_DRONES = [
    [0, 2, 5, 7, 8, 44, 0],
    [0, 48, 18, 26, 11, 6, 0],
]
PAPER_VRPD_TABLE2 = dict(total_distance=16.80, travel_cost=14.68, total_cost=264.68,
                         overall_time=36.78, efficiency=55.88)

# UAVRP, pure-drone group – 10 drones
PAPER_UAVRP_DRONES = [
    [0, 34, 35, 12, 0],
    [0, 30, 32, 28, 0],
    [0, 14, 5, 0],
    [0, 21, 46, 45, 11, 0],
    [0, 17, 49, 20, 13, 16, 8, 33, 7, 48, 0],
    [0, 36, 47, 9, 29, 41, 3, 0],
    [0, 39, 1, 18, 50, 0],
    [0, 42, 24, 25, 23, 26, 6, 19, 38, 44, 15, 2, 4, 40, 31, 43, 22, 0],
    [0, 37, 0],
    [0, 27, 10, 0],
]
PAPER_UAVRP_TABLE2 = dict(total_distance=29.48, travel_cost=29.48, total_cost=279.48,
                          overall_time=33.48, efficiency=58.57)
