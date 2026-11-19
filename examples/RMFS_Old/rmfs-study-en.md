# RMFS route selection and snapshot lookahead

## Warehouse and objective

The model uses ten 2-by-6 rack clusters, 120 racks and ten workstations. Four-connected paths support right-angle turns. Makespan is simulated completion time. Wall-clock runtime measures the cost of computing the decision.

## DEVS crossing experiment

At simulated time 5 s, save a SOCS snapshot and restore it before each branch. Compare Wait on the distance-shortest A* route, north and south detours, Avoid B's Path, and three yield bays. Evaluate four diverse candidates first and expand when an unchecked candidate can still beat the incumbent under the free-flow bound. The branch clock advances to predicted rack clearance, motion completion, or scheduled entries. Continuous and animation sampling are disabled during lookahead. Verification can replay with footprint checks at 0.01 s.

## Crossing results

With ten robots and zero entry delay, Detour North completed in 58.30 s versus 59.76 s for Wait. With a 0.25 s entry delay, both the single Wait baseline and lookahead selected Wait at 59.76 s. Single-run runtime was 0.316 s and lookahead runtime was 2.082 s. Six of seven branches were run. Five completed, one reached a dead end, zero repeated states were detected and zero timed out. Branch checks took 2.070 s total, 345 ms on average and 404 ms at maximum. Dead ends were counted from the STALLED status. A rack parked at the destination blocked a crossing route in the stalled branch.

## 100-order reservation experiment

Ten robots process 100 orders with seed 42. Each mission goes from home to rack to workstation, returns the rack to its original slot, and returns to home. Traffic waiting is placed before dispatch at home. At each order decision, evaluate a distance-shortest A* plan and alternatives that replace one mission leg after excluding a candidate edge. Evaluate each against the same space-time reservations and select the earliest predicted home return. This experiment scores routes by reservation calculation. The selected schedule is then replayed in DEVS.

## 100-order results

The search evaluated 701 candidates, averaging 7.01 per order (range 5 to 10). The baseline used one A* plan per order. Of the selected plans, 15 kept the original A* route, 78 used an equal-distance alternative, and 7 used a longer route. The equal-distance alternatives occurred on the home-to-rack, outbound, rack-return and home-return legs in counts of 13, 27, 26 and 12. The longer alternatives occurred on the outbound (2) and rack-return (5) legs. Makespan fell from 3433.63 s to 3111.06 s (9.4%). Planning runtime rose from 0.803 s to 4.805 s. Candidate checks averaged 6.3 ms and peaked at 22.5 ms. DEVS replay completed all 100 orders with no collision detected at the checked event times.

## Limits and follow-up

Recorded runtimes are machine-dependent observations. The crossing search branches at one decision point. Repeated online decisions, tail latency under a decision deadline, and general deadlock freedom remain to be evaluated. Sampled collision checks do not guarantee separation between samples. To distinguish the effects of geometry and timing, record shared nodes and segments, time-overlapping reservations, home wait, turns, and detour distance per candidate.

## Reproduction commands

Run from the repository root.

```powershell
python -m examples.RMFS.warehouse_crossings --robots 10 --stats
python -m examples.RMFS.warehouse_crossings --robots 10 --crossing-delay 0.25 --stats
python -m examples.RMFS.fleet --robots 10 --orders 100 --seed 42 --home-route-stats
```
