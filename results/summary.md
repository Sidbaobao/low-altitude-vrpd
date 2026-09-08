## Table 1 - Routes

| Group | Vehicle | Route | Distance (km) | Load (kg) |
|---|---|---|---:|---:|
| VRPD | Truck 1 | 0->36->14->34->23->49->25->24->1->39->0 | 1.551 | 36 |
| VRPD | Truck 2 | 0->37->29->33->15->41->45->7->5->4->28->2->0 | 3.262 | 44 |
| VRPD | Truck 3 | 0->30->27->32->19->13->50->3->16->44->8->0 | 3.350 | 40 |
| VRPD | Truck 4 | 0->17->31->40->47->43->9->48->18->22->35->0 | 2.959 | 40 |
| VRPD | Drone 1 | 0->20->12->6->11->26->0 | 2.250 | 20 |
| VRPD | Drone 2 | 0->42->10->46->38->21->0 | 1.349 | 20 |
| CVRP | Truck 1 | 0->14->34->35->22->18->48->9->43->47->40->31->17->0 | 3.221 | 48 |
| CVRP | Truck 2 | 0->36->0 | 0.226 | 4 |
| CVRP | Truck 3 | 0->37->29->33->15->41->45->7->5->4->28->2->0 | 3.262 | 44 |
| CVRP | Truck 4 | 0->39->24->25->20->12->6->11->26->23->49->1->0 | 2.998 | 44 |
| CVRP | Truck 5 | 0->42->30->32->46->10->19->13->50->3->16->44->8->38->27->21->0 | 4.134 | 60 |
| UAVRP | Drone 1 | 0->39->1->24->25->42->0 | 0.933 | 20 |
| UAVRP | Drone 2 | 0->17->40->28->4->2->0 | 1.571 | 20 |
| UAVRP | Drone 3 | 0->30->44->16->3->50->0 | 2.488 | 20 |
| UAVRP | Drone 4 | 0->37->41->45->7->5->0 | 2.118 | 20 |
| UAVRP | Drone 5 | 0->34->26->11->6->23->0 | 2.229 | 20 |
| UAVRP | Drone 6 | 0->21->8->33->15->29->0 | 1.848 | 20 |
| UAVRP | Drone 7 | 0->31->47->9->48->43->0 | 1.716 | 20 |
| UAVRP | Drone 8 | 0->19->13->12->20->49->0 | 1.970 | 20 |
| UAVRP | Drone 9 | 0->36->22->18->35->14->0 | 1.552 | 20 |
| UAVRP | Drone 10 | 0->32->10->46->38->27->0 | 1.224 | 20 |

## Table 2 - Delivery effect of the three groups

| Group | Fleet | Distance (km) | Fixed cost | Travel cost | Total cost | Total time (min) | Efficiency Z | Solver | Gap | Time (s) |
|---|---|---:|---:|---:|---:|---:|---:|---|---:|---:|
| UAVRP | 10 drones | 17.65 | 250.00 | 17.65 | 267.65 | 57.65 | **58.14** | TIME_LIMIT | 2.78% | 901.0 |
| CVRP | 5 trucks | 13.84 | 250.00 | 11.07 | 261.07 | 66.52 | **57.54** | OPTIMAL | 0.93% | 1.5 |
| VRPD | 4 trucks + 2 drones | 14.72 | 250.00 | 12.50 | 262.50 | 53.37 | **56.77** | TIME_LIMIT | 4.91% | 1801.0 |

## VRPD relative to the single-mode groups (positive = VRPD is better)

| Indicator | vs CVRP | vs UAVRP |
|---|---:|---:|
| Total distance | -6.4% | +16.6% |
| Total time | +19.8% | +7.4% |
| Travel cost | -12.9% | +29.2% |
| Total cost | -0.5% | +1.9% |
| Efficiency Z | +1.3% | +2.4% |

GA: 83 generations, best Z = 56.9460; MILP improved it to 56.7685.

## Supplement - fixed service time charged per vehicle (tables above) vs per mode (below), identical routes

| Group | Total time (min) | Efficiency Z |
|---|---:|---:|
| UAVRP | 21.65 | 55.26 |
| CVRP | 46.52 | 55.94 |
| VRPD | 38.37 | 55.57 |
