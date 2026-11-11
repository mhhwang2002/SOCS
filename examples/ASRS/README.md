# ASRS warehouse concept model

This folder contains a conceptual 3D automated storage and retrieval system (AS/RS) based on the sketch in `examples/ASRS.pptx`. The original sketch file is preserved. `ASRS_detailed.pptx` contains the sketch plus two editable detail slides.

## Dimensions

Dimensions read from the sketch:

- 15 bays at 1.0 m each: rack length 15.0 m
- 6 storage levels at 0.75 m pitch: rack height 4.5 m
- 50 mm beam thickness

Dimensions assumed where the sketch did not specify them:

- Rack depth: 1.0 m
- Aisle width: 2.0 m
- Concept warehouse floor: 18 m × 6 m
- Rack post section: 70 mm
- Fork reach: 1.25 m
- Load clearance for lifting: 0.12 m

These assumptions are documented in the Blender scene as well. They are for visualization and kinematics demonstration, not structural or safety engineering.

## Crane and animation

The Blender scene has a keyframed motion demonstration. The DEVS example models simultaneous X rail travel and Z hoist travel, followed by Y-axis fork reach. At the cell, the crane sends a retrieve request; the selected cell sends a box output, clears its occupied state, and the crane attaches the box to its fork. The crane lifts, withdraws, and returns X/Z to its home pose, then approaches the unload station, sends the box out, and lowers the fork. The simulation pauses after Fork Down with the viewer open. Each axis uses acceleration, optional cruise, and deceleration; short moves can omit the cruise phase. The model contains 90 passive `ASRSCell` atomic models and one active `Crane`, joined by request and box-transfer couplings in the `ASRS` coupled model.

The crane home is at the center of cell coordinate `(-1, 0)`, outside the rack (`x=-0.5 m`); cell `(0, 0)` starts at `x=0`. Its rail and mast center is at `y=-1.25 m`; the fork retract position is `y=-0.55 m`, and its cell approach position is `y=0.50 m`, reducing the fork stroke from 1.25 m to 1.05 m. With the 0.76 m box depth and 0.18 m post depth, the retracted box has 0.23 m of clearance from the post. The left/right unload points remain at `y=-0.25 m` on a short platform at the lowest transfer height.

Open `ASRS_warehouse.blend` in Blender and play the timeline with Space. The scene is built by `build_asrs.py`.

To launch the coupled simulation and live Panda3D view from the project root:

```powershell
python -m pip install -e ".[visual3d]"
python -m examples.ASRS.run_asrs
```

In the viewer, **Space** starts or continues interactively, **A** runs continuously, **I** pauses at DEVS phase changes, **R** resets the camera, and **Esc** closes the window. Rack uprights, beams, shelf decks, floor rails, stored loads, and the crane are modeled as shaded 3D geometry.

## ACO + SOCS order scheduling experiment

Generate and save a readable JSON problem instance from the project root. The saved file contains the rack cells, product colors, order membership, requested quantity, and shortage quantity:

```powershell
python -m examples.ASRS.generate_aco_instance --seed 42 --output examples/ASRS/instances/demo.json
```

Both optimization and animation load the fixed problem file; `--seed` controls the ACO search and does not change the saved problem. Run the combined objective with:

```powershell
python -m examples.ASRS.aco_optimization --instance examples/ASRS/instances/demo.json --seed 42 --ants 12 --iterations 25
```

The generated example places 50 products (10 of each of five colors) in random cells of a 15 × 6 rack. Ten orders specify color-specific SKU demand. Each order receives five baseline product units; three to five orders request one or two additional units whose colors are also stored in the JSON. ACO can assign a shelf product only to an order that requires that color, so orders compete for shared SKU inventory. The prior `product_ids` are the generator’s baseline allocation; the optimizer may reassign those products among orders.

The single minimized score gives complete-order count strict priority. With the current scalar score, shortage and time are traded off according to lambda:

`J = M × (number of incomplete orders) + λ × (unshipped product units) + time`

`time` is makespan by default or mean order departure with `--time-objective mean`. The completion coefficient `M` is calculated above the maximum possible shortage and travel terms, so one additional complete order always outranks any change in λ-weighted shortage or time. `λ` is measured in simulation time units per missing color-specific product and defaults to 50,000. With the benchmark's λ=100, one fewer missing unit is worth 100 time units; this does not strictly prioritize shortage over makespan because the makespan bound is 875. A scalar score needs λ greater than the maximum makespan range (876 is sufficient here) to enforce that priority. The ACO records each product-to-order assignment and respects SKU color.

In this generated instance, demand totals exceed inventory by five units across blue and white SKUs. If every available product is shipped, total unshipped units are therefore fixed at five, and `λ × 5` is the same for every allocation. The complete-order term decides how those five shortages are concentrated among orders; makespan breaks ties. Changing λ affects comparisons when candidate schedules leave different quantities unshipped.

For example:

```powershell
python -m examples.ASRS.aco_optimization --instance examples/ASRS/instances/demo.json --penalty-weight 50 --seed 42
python -m examples.ASRS.run_aco_animation --instance examples/ASRS/instances/demo.json --objective makespan --penalty-weight 50 --seed 42
```

Order-specific demand includes color counts, so the optimizer can consolidate compatible products into complete orders. A pure per-unit shortage penalty cannot distinguish allocations that ship the same total number of products; maximizing complete orders provides that distinction. The model does not allow substitution between colors.

This first model intentionally uses a simple travel-time estimate: constant X/Z speeds, simultaneous axis motion, no acceleration/deceleration, no bin return, and fixed pick and station service times. It is an optimization prototype; the current interactive AS/RS model remains the separate one-box visualization demo.

To send an ACO schedule through the AS/RS Controller and animate the colored products, run:

```powershell
python -m examples.ASRS.run_aco_animation --instance examples/ASRS/instances/demo.json --objective makespan --seed 42
```

Use `--objective mean` to use mean order departure as the time tie-breaker while keeping the complete-order and shortage terms. Pass the same `--instance` file to reproduce the exact problem. The HUD shows the combined objective and λ-weighted shortage. Per-order departure messages show actual shipped quantity / requested quantity. After the simulation ends, press **S** to show makespan, shipped/requested totals, complete/partial order counts, and weighted shortage; press **S** again to return to live progress. A DEVS-measured cycle table is used by default for ACO evaluation; the animation executes the same crane cycle model. The table can be regenerated with `python -m examples.ASRS.crane_cycle_times` or overridden with `--cycle-times`. **Space** resumes, **A** runs automatically, **I** steps through DEVS events, and **Esc** closes.

### DEVS-measured crane cycle times

Generate the single-crane table from the actual Cell, Controller, and acceleration-profile Crane models:

```powershell
python -m examples.ASRS.crane_cycle_times
```

The command writes `examples/ASRS/data/crane_cycle_times.json`. It records 180 initial home-to-cell-to-unload cycles and 360 repeated cycles for each cell, starting unloading station (L/R), and ending unloading station (L/R). Each entry contains both the product `box_out` time and the time the crane is ready for its next command. The shared 10-unit controller startup delay is excluded from cycle values and charged once by the schedule evaluator. The generated table describes the current DEVS parameterization; it is not yet calibrated to a physical crane.

After unloading, products are displayed on per-order trays in the outbound staging area. Each order keeps the same left-to-right slot while products accumulate in the gray collection row. When the order leaves the system, its products move to the green complete row or orange partial-penalty row. The viewer legend maps slots to order IDs.

## Allocation strategy comparison

Compare random allocation, a deterministic feasible-first heuristic, and ACO on the same generated problem instances:

```powershell
python -m examples.ASRS.compare_allocation_methods --seed 42 --repeats 3 --ants 12 --iterations 25
```

The experiment generates 16 reproducible instances by combining 3 or 5 active SKU families, balanced or variable order sizes, SKU demand spread across many orders or clustered into fewer orders, and shortages spread across SKUs or concentrated in one SKU. Each instance has 50 stored units and 55 requested units. Every method receives the same stock, order demands, and rack positions. The random method and ACO are repeated with multiple seeds; the deterministic policy runs once per instance.

The deterministic policy prioritizes currently feasible complete orders, uses SKU scarcity as a tie-breaker, then allocates remaining products to partial orders. It routes assigned products using a nearest-cell heuristic and chooses the cheaper unloading station. ACO jointly searches order priority, SKU-compatible product assignments, cells, and stations. Results are compared by complete orders, shortage units, then makespan; the CSV reports complete, partial, and unprocessed orders separately.

## Deadline and backup-center pilot

The first deadline study evaluates existing schedules under order-specific deadlines without changing the ACO objective or stopping the simulated crane. All orders are available at time zero, so the `FCFS` reference is a fixed order-ID input sequence, not an online first-come policy. A fixed urgency ranking is generated from the experiment seed, and each order receives a due time based on that reference makespan and a deadline factor. This isolates schedule effects before adding online arrivals or work interruption.

Policies are FCFS, earliest due date (EDD), minimum estimated slack, nearest eligible pick, and the existing makespan ACO. The rescue workload is the quantity of each order's requested SKUs not delivered locally by that order's due time. The report distinguishes orders with some local delivery from orders with no local delivery by the deadline. The backup center is currently an accounting measure: it has no capacity, travel-time, or queue model yet.

Run the reproducible 16-instance study from the project root:

```powershell
python -m examples.ASRS.deadline_experiment --instance examples/ASRS/results/allocation_study/instances --cycle-times examples/ASRS/data/crane_cycle_times.json --seed 42 --repeats 5 --ants 12 --iterations 25
```

Use `--deadline-factors` to provide alternative multipliers relative to each instance's FCFS reference makespan. A single JSON file can also be passed to `--instance` for a quick pilot. The 16-instance run writes its design, raw policy runs, and instance-level summary under `examples/ASRS/results/deadline_study_16instances/`. The summary reports 95% t intervals across instances and paired changes versus FCFS; ACO search seeds are averaged within each instance before computing those intervals. Raw rows retain due times, urgency rank, and each ACO seed. Current instances have 55 requested units and 50 stored units, so five rescue units are structurally unavoidable; the experiment measures additional rescue workload caused by missing the local cutoff.

The single-instance pilot and 16-instance results are interpreted in [DEADLINE_EXPERIMENT.md](DEADLINE_EXPERIMENT.md). Crane dynamics are currently uncalibrated; this run identifies a large difference between the simplified ACO time estimate and acceleration-profile DEVS simulation, so it is not yet a hardware-performance claim.

## Reproducible online-order workload pilot

The next experiment stage releases orders over simulation time and compares online dispatch rules. Generate a fixed JSON arrival trace from a saved instance:

```powershell
python -m examples.ASRS.online_workload --instance examples/ASRS/instances/demo_seed_42.json --output examples/ASRS/instances/online/demo_seed_42_arrival_seed_42.json --cycle-times examples/ASRS/data/crane_cycle_times.json --seed 42 --mean-interarrival 30 --due-slack-factor 2 --urgency-spread 0.4
```

The trace records its generation seed, the source instance SHA-256, cycle-table SHA-256, every order's release and due times, urgency rank, and SKU demand. Arrival gaps follow an exponential distribution (a Poisson arrival process); the first order arrives at time zero. Synthetic due dates are `arrival + due_slack_factor × urgency_fraction × requested units × median DEVS cycle-ready time`; urgency fractions are assigned by a seeded shuffle across the configured `1 ± urgency_spread/2` range. This is an experimental control, not an estimate of real customer behavior.

Compare policies on one or more traces generated from the same fixed instance:

```powershell
python -m examples.ASRS.online_dispatch_experiment --instance examples/ASRS/instances/demo_seed_42.json --trace-dir examples/ASRS/instances/online --cycle-times examples/ASRS/data/crane_cycle_times.json --output-dir examples/ASRS/results/online_pilot --seed 42 --repeats 3 --ants 8 --iterations 12
```

For a broader paired design, generate a matrix over the 16 fixed allocation-study instances. The defaults use mean inter-arrival times 15, 60, and 120; due-date slack factors 1, 2, and 4; and three arrival/urgency replicates per cell:

```powershell
python -m examples.ASRS.online_workload_matrix --instances examples/ASRS/results/allocation_study/instances --output-dir examples/ASRS/instances/online_factorial --cycle-times examples/ASRS/data/crane_cycle_times.json --seed 1000 --replicates 3 --mean-interarrivals 15 60 120 --due-slack-factors 1 2 4 --urgency-spread 0.4
python -m examples.ASRS.online_dispatch_experiment --instance examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_factorial --cycle-times examples/ASRS/data/crane_cycle_times.json --output-dir examples/ASRS/results/online_factorial --seed 42 --repeats 2 --ants 6 --iterations 8
```

The summary stratifies by arrival intensity and deadline slack, averages the three arrival/urgency replicates within each source instance, then calculates 95% intervals across the 16 instances. It also reports paired changes from FCFS for complete orders, rescue units, mean response time, and P95 response time. This design avoids treating multiple traces from one instance as independent inventory scenarios. The current 1.05 m fork-stroke 432-trace factorial results are in `examples/ASRS/results/online_factorial_geometry_105/`, with traces in `examples/ASRS/instances/online_factorial_geometry_105/`. The earlier `online_factorial/` artifacts use the previous geometry. Current results and interpretation are in [RESEARCH_PLAN.md](RESEARCH_PLAN.md).

For a paired study that expresses arrival intensity as reference Crane load and varies due-date allowance separately, use `--common-random-numbers`. It reuses the same arrival and urgency draws across factor levels (arrival gaps are scaled by mean inter-arrival time). The current 16-instance design uses reference load levels 0.6, 0.9, and 1.2 and due-allowance factors 0.75, 1, and 2:

```powershell
python -m examples.ASRS.online_workload_matrix --instances examples/ASRS/results/allocation_study/instances --output-dir examples/ASRS/instances/online_load_slack_paired_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --seed 12000 --replicates 3 --mean-interarrivals 167.406494968 111.604329979 83.703247484 --due-slack-factors 0.75 1 2 --urgency-spread 0.4 --common-random-numbers
python -m examples.ASRS.online_dispatch_experiment --instance examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_load_slack_paired_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --output-dir examples/ASRS/results/online_load_slack_paired_geometry_105 --seed 12000 --repeats 2 --ants 6 --iterations 8 --partial-penalty-kappa 60 --partial-penalty-epsilon 1 --partial-penalty-cap 5
```

The target load is a reference estimate based on mean requested units × median DEVS cycle time; it is not measured utilization. Detailed assumptions, paired results, and the response-time definition are in [RESEARCH_PLAN.md](RESEARCH_PLAN.md).

Policies are Random, FCFS, EDD, minimum slack, preemptive minimum slack, nearest eligible pick, and rolling-horizon ACO. They see only released unresolved orders. All except `preemptive_minimum_slack` serve the selected order through the available-SKU limit before the next dispatch choice. The preemptive rule reselects after every completed pick-and-delivery cycle; an interrupted order keeps its assigned L/R station and waits in unlimited staging. It does not interrupt Crane motion or add a fixed switching overhead. ACO replans over the current released queue at each order boundary. The run writes raw observations, a trace-level summary, and experiment-design metadata. Response time is measured from arrival to the final local box-out for the order's pickable stock; an order with no locally pickable units is resolved at its dispatch decision. Deadline completion, rescue SKU units, mean/P90/P95 response time, full-run makespan, queue size, ACO compute time, and preemption count are retained.

The first preemption comparison on 432 paired traces found that this every-cycle reselection rule reduced on-time complete orders and increased mean and P95 response time relative to non-preemptive `minimum_slack` across the tested reference loads. It is a synthetic pilot, not evidence against all preemption strategies; details, confidence intervals, caveats, and the reproduction command are in [RESEARCH_PLAN.md](RESEARCH_PLAN.md). The result points to testing a selective urgent-arrival trigger instead of switching on every cycle.

This online study uses synthetic Poisson arrivals, synthetic due dates, fixed initial inventory, no replenishment, no interruption within an order, and no finite backup-center queue. The policies can run both in cycle-table replay and in a live coupled DEVS model that receives order-arrival messages. The `online_devs_validation` harness compares dispatch, box-out, and makespan timestamps on the same fixed trace. The current-geometry validation matched 16 traces × five policies within `1.71e-12` time units. This establishes implementation consistency for those tested paths, not physical Crane accuracy. Use the same trace files and stock instances for every policy. Confidence intervals in the factorial summary use source inventory/order instances as their replication unit.

### Partial-order penalty pilot

The online dispatch experiment also reports a post-hoc penalty for stock-related partial orders. At local departure, it multiplies missing product units by a capped urgency factor based on due-date slack:

```text
penalty = missing_units × min(cap, 1 + kappa / (max(due_time - departure_time, 0) + epsilon))
```

This first pilot does not change dispatch decisions or the ACO objective. It reports missing SKU-type count separately, without penalizing it. The default pilot parameters are `kappa=60`, `epsilon=1`, and `cap=5` in simulation-time units. The 16-trace result and parameter sensitivity are documented in [RESEARCH_PLAN.md](RESEARCH_PLAN.md). Order-level outcomes, including departure slack and calculated penalty, are written to `partial_order_penalty_raw.csv`.

```powershell
python -m examples.ASRS.online_dispatch_experiment --instance examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_convergence_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --output-dir examples/ASRS/results/partial_penalty_pilot_geometry_105 --seed 9000 --repeats 5 --ants 6 --iterations 8 --partial-penalty-kappa 60 --partial-penalty-epsilon 1 --partial-penalty-cap 5
```

The online DEVS/replay validation can be reproduced with:

```powershell
python -m examples.ASRS.online_devs_validation --instance examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_convergence_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --output-dir examples/ASRS/results/online_devs_validation_geometry_105 --seed 42
```

### Crane hardware parameter sensitivity

The motion-profile input is `data/crane_hardware_scenarios.json`. It includes the current illustrative DEVS defaults plus preliminary Dematic and Tianjin Master survey profiles. Some values or conditions remain unverified; the Dematic fork acceleration is imputed from the SOCS default. The model assumes symmetric acceleration/deceleration and triangular or trapezoidal motion. It maps physical X travel to DEVS X, physical Y hoist to DEVS Z and the short vertical fork-lift motion, and physical Z fork reach to DEVS Y. These profiles support sensitivity analysis, not manufacturer-specific performance claims.

The pilot at mean inter-arrival 60 and due-slack factor 2 used the same 16 traces and fixed deadlines under all profiles. For example, FCFS full-run makespan was 794.05 under the current profile, 820.48 under the Dematic-derived profile, and 1090.27 under the slower loaded-fork profile. The paired makespan changes versus current were +26.43 [95% CI 22.02, 30.83] and +296.22 [279.51, 312.93]. Deadline-complete orders were 3.88/10, 3.75, and 2.19; rescue units were 24.00, 26.19, and 38.38. These results quantify sensitivity to these assumed timing profiles only. They do not calibrate the model to actual hardware. The fixed deadlines in this run were generated from traces using the prior cycle table, so rerun the workload generator before treating the results as a new baseline study.

Run the sensitivity study and generate DEVS-measured cycle tables with:

```powershell
python -m examples.ASRS.hardware_sensitivity --instances examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_convergence --output-dir examples/ASRS/results/hardware_sensitivity_iat60_slack2 --scenario-file examples/ASRS/data/crane_hardware_scenarios.json --seed 42 --repeats 2 --ants 6 --iterations 8
```

Detailed policy results, paired confidence intervals, and the DEVS/replay checks for all motion profiles are in [RESEARCH_PLAN.md](RESEARCH_PLAN.md) and `examples/ASRS/results/`.

The research scope, testable claims, staged experiment design, and current limitations are tracked in [RESEARCH_PLAN.md](RESEARCH_PLAN.md).

The command saves every JSON instance plus raw runs and per-scenario method means under `examples/ASRS/results/allocation_study/`. It uses the DEVS-measured cycle table when available. The summary numbers shown below are the historical run from before that table was introduced; rerun the command to produce comparable updated makespans. This first experiment varies SKU structure within the five SKU families supported by the current single-crane model; expanding the model to additional SKU identifiers is a separate follow-up.

Initial run (`--seed 42 --repeats 3 --ants 12 --iterations 25`), averaged equally across the 16 scenarios. Here `M=6376` and lambda is 100. The incomplete-order term is `M × (partial + unprocessed)`, and the full score is `J = M × (partial + unprocessed) + lambda × shortage units + makespan`:

| Method | Complete orders / 10 | Partial orders | Unprocessed orders | `M × (partial + unprocessed)` | Shortage units | `lambda × shortage` | Mean makespan | Mean objective `J` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Random | 7.69 | 2.08 | 0.23 | 14,744.50 | 5.00 | 500.00 | 482.70 | 15,727.20 |
| Deterministic heuristic | 7.88 | 1.69 | 0.44 | 13,549.00 | 5.00 | 500.00 | 349.96 | 14,398.96 |
| ACO | 8.25 | 1.63 | 0.12 | 11,158.00 | 5.00 | 500.00 | 312.53 | 11,970.53 |

In this benchmark, each SKU's total demand is its stock plus a total shortage of five units. The policies ship all 50 available units, so total shortage and its weighted penalty are fixed; the meaningful allocation differences are how those five missing units are distributed, how many orders become complete, and the resulting route time. The table and decision rules are documented in [ALLOCATION_EXPERIMENT.md](ALLOCATION_EXPERIMENT.md). A finite operating horizon or pickup budget would be needed to make total shortage itself vary by policy.

## Files

- `build_asrs.py`: Blender scene and animation builder
- `asrs_devs.py`: passive cell, crane, and coupled ASRS models
- `run_asrs.py`: live Panda3D simulation entry point
- `compare_allocation_methods.py`: reproducible random, heuristic, and ACO benchmark
- `ASRS_warehouse.blend`: editable Blender scene
- `ASRS_warehouse_preview.png`: rendered preview
- `ASRS_detailed.pptx`: original sketch with rack and crane detail slides
