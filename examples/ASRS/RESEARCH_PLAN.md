# AS/RS Research Plan

## Working research question

For a single-aisle, single-crane AS/RS with finite SKU inventory, how do inventory allocation and crane dispatch policies affect urgent orders' local fulfillment and the amount of residual work that must be rescued by another fulfillment center?

The intended contribution is an experimentally reproducible comparison of policies under a transparent, calibrated-to-the-DEVS timing model. It is not yet a claim about a particular commercial warehouse or a universally superior optimizer.

## Current scope and evidence

- One rack, one crane, two unloading stations, five SKU families, and a finite initial inventory.
- The current fixed-batch study compares random allocation, a deterministic heuristic, and ACO on saved inventory/order instances. It reports complete and partial orders, shortage units, and makespan.
- A separate deadline pilot compares FCFS, EDD, minimum-slack, nearest-pick, and makespan ACO on 16 fixed instances. All orders are released at time zero; due dates are evaluation cutoffs.
- A DEVS-generated cycle table is measured from the same `Crane` atomic model used by the live coupled simulation. The cycle-table replay and live DEVS implementation have been checked against one another for fixed online traces and multiple motion profiles.
- Hardware parameter values are still exploratory. The alternate profiles use reported values from preliminary source surveys, with explicit imputations where data are missing; they are not verified specifications or calibrated models of a particular installation.
- The pilot's Poisson online-arrival trace generator is an initial workload-artifact tool. It does not yet simulate online dispatch or establish empirical arrival/deadline distributions.

## Claims to test, not assume

1. Allocation and dispatch choices can change local complete-order service and residual rescue work when SKU inventory is scarce.
2. ACO's benefit, if any, depends on SKU scarcity, spatial distribution, arrival intensity, and deadline tightness; it may trade average service time against tail delays or rescue workload.
3. Evaluating ACO only by full-horizon makespan can rank policies differently from deadline service and rescue workload.

Each claim requires paired experiments on identical inventory, demand, arrival, due-date, and timing inputs.

## Experimental stages

### Stage A — Fixed-batch allocation and route study

Use saved instances and compare random feasible allocation, a fully specified deterministic heuristic, and ACO. Vary product scarcity and spatial distribution while keeping instance semantics explicit. Preserve per-instance outcomes and seeds; report paired differences, variability, and confidence intervals. The existing 16-instance study is a pilot and should not be presented as broad evidence without additional scenario coverage.

### Stage B — Static urgent-order study

Keep all orders available at time zero and use due dates only to evaluate local shipments. This isolates scheduling under urgency without adding online information effects. Compare FCFS/input-order, EDD, minimum slack, nearest eligible pick, and makespan ACO. Report complete orders by deadline, missing SKU units, rescue orders, mean and tail completion times, and full-plan makespan.

### Stage C — Online arrivals and rolling dispatch (initial cycle-table pilot implemented)

Use fixed, shared arrival-trace files. At each crane decision point, expose only released orders to the dispatch policy; reserve inventory when a product is assigned or picked according to one explicitly chosen rule. Include FCFS, EDD, minimum slack, nearest pick, and a rolling-horizon ACO that replans only at defined non-preemptive decision points. Compare policies on exactly the same traces and inventory. Measure mean and P90/P95 response time from arrival, on-time complete-order rate, local shortage/rescue units, throughput, backlog, and runtime. Keep batch makespan as a secondary metric only when the workload has a defined end. The same policies and fixed inputs are now supported by both cycle-table replay and a live coupled DEVS model; equivalence has been checked at every dispatch and box-out event, rather than inferred only from aggregate metrics.

The first arrival-trace generator uses a Poisson process and a synthetic due-date rule based on a DEVS cycle-time statistic. These are controllable experimental factors, not estimates of real customer behavior. Later work should replace or supplement them with anonymized operational arrival and due-date data if available.

### Stage D — Timing calibration and external validity

Choose one identifiable manufacturer/model scenario rather than combining values from different products. Verify units, axis mapping, payload condition, acceleration/deceleration assumptions, and handling delays against original technical documents. Where possible, compare simulated travel/cycle durations against measurements. Until that evidence is available, report parameter sensitivity analysis only, with assumptions and imputations visible.

## Fair comparison and statistical reporting

- Share the exact instance, arrival trace, due dates, timing table, and stock rules across methods within each paired comparison.
- Separate workload randomness (inventory/orders/arrivals) from algorithm randomness (ACO seed); record both.
- Store every replicate row. Summaries must not replace raw results.
- Treat independent workload traces/instances as the principal replication unit. Average repeated ACO seeds within each workload before computing across-workload intervals, and also report optimizer variability separately.
- Report effect sizes and paired confidence intervals, not only means or best-seed values. Avoid claiming significance from the ACO seeds alone when they share one workload.
- Publish generated instances/traces, parameter files, exact commands, software revision, and the cycle-time table or its generation inputs.

## Immediate next work

1. Validate the online-trace schema and retain fixed traces with source-instance hashes.
2. Use the live DEVS equivalence harness whenever dispatch logic, cycle-time generation, or motion parameters change.
3. Expand paired policy comparisons across independently generated inventories, arrivals, and demand/deadline regimes; choose sample size based on pilot variance and a prespecified effect size.
4. Obtain and verify one coherent hardware specification or measured cycle dataset, including handling delays and loaded/unloaded motion, before making physical-performance claims.

## Current limitations

The current demand and arrival setups are synthetic, the Crane timing parameters are not hardware-calibrated, and the backup center is represented only as rescue workload. The live online DEVS model and cycle-table replay agree to numerical precision on the checked traces, but the DEVS comparison does not validate either model against physical equipment. These boundaries must remain explicit in any manuscript based on the current results.

## Online factorial pilot — current 1.05 m fork geometry

After changing the Crane geometry to a 1.05 m fork stroke and rail center y=-1.25 m, the online workload matrix and all due dates were regenerated from the current DEVS cycle-time table. The matrix uses the 16 fixed allocation-study instances, mean inter-arrival times of 15, 60, and 120 time units, due-slack factors of 1, 2, and 4, urgency spread 0.4, and three arrival/urgency traces per instance-condition. It produced 432 current-geometry traces and 3,456 policy observations. Random and rolling ACO used two algorithm seeds per trace; ACO used 6 ants × 8 iterations. Intervals use paired differences across 16 source instances after averaging traces and algorithm seeds within each instance.

For due-slack factor 2, current-geometry results are:

| Mean inter-arrival | Policy | Complete orders by deadline / 10 | Change vs FCFS (95% CI) | Rescue units | Change vs FCFS (95% CI) | Mean response |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 15 | FCFS | 2.19 | baseline | 38.73 | baseline | 361.6 |
| 15 | EDD | 2.63 | +0.44 [0.08, 0.80] | 39.79 | +1.06 [-0.42, 2.55] | 333.0 |
| 15 | Nearest pick | 2.88 | +0.69 [0.44, 0.94] | 36.19 | -2.54 [-4.40, -0.68] | 352.7 |
| 15 | Rolling ACO | 3.01 | +0.82 [0.59, 1.06] | 37.47 | -1.26 [-3.07, 0.55] | 350.9 |
| 60 | FCFS | 4.12 | baseline | 23.46 | baseline | 221.6 |
| 60 | EDD | 4.44 | +0.31 [0.02, 0.60] | 23.19 | -0.27 [-1.10, 0.56] | 209.2 |
| 60 | Nearest pick | 4.73 | +0.60 [0.28, 0.93] | 21.48 | -1.98 [-3.62, -0.34] | 216.8 |
| 60 | Rolling ACO | 5.22 | +1.09 [0.63, 1.56] | 20.83 | -2.63 [-4.26, -0.99] | 211.3 |
| 120 | FCFS | 6.60 | baseline | 8.42 | baseline | 119.5 |
| 120 | EDD | 6.79 | +0.19 [0.04, 0.33] | 7.73 | -0.69 [-1.15, -0.23] | 117.8 |
| 120 | Nearest pick | 6.67 | +0.06 [-0.09, 0.21] | 8.27 | -0.15 [-0.60, 0.31] | 118.9 |
| 120 | Rolling ACO | 7.00 | +0.40 [0.22, 0.57] | 7.98 | -0.44 [-0.98, 0.10] | 116.2 |

Rolling ACO improves complete-by-deadline counts on these conditions, but P95 response remains above FCFS at all three arrival intervals (703.9 vs 636.1, 479.9 vs 372.5, and 242.6 vs 216.9). This remains a synthetic pilot rather than a publication result; its due dates are not fitted to customer data and its rescue units are not forecasts of a physical backup center.

All current-geometry conditions, confidence intervals, and raw runs are in `results/online_factorial_geometry_105/online_policy_summary.csv`, `online_policy_raw.csv`, and `online_experiment_design.json`. The 432 input traces are in `instances/online_factorial_geometry_105/`; each stores source-instance and current cycle-table hashes. Earlier `online_factorial/` artifacts used the previous 1.25 m fork stroke and are historical only.

Recreate the current-geometry factorial traces and policy runs from the repository root with:

```powershell
python -m examples.ASRS.online_workload_matrix --instances examples/ASRS/results/allocation_study/instances --output-dir examples/ASRS/instances/online_factorial_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --seed 1000 --replicates 3 --mean-interarrivals 15 60 120 --due-slack-factors 1 2 4 --urgency-spread 0.4
python -m examples.ASRS.online_dispatch_experiment --instance examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_factorial_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --output-dir examples/ASRS/results/online_factorial_geometry_105 --seed 42 --repeats 2 --ants 6 --iterations 8
```

### Paired reference-load × due-allowance study

This study separates system load from order deadline tightness. The 16 fixed instances all request an average of 5.5 units per order. The current cycle table has a median `ready` duration of 18.262526724 time units, giving a reference order workload \(\bar P_{ref}=5.5\times18.262526724=100.443896981\). This is a nominal workload estimate, not observed utilization. Set \(\rho_{ref}=\bar P_{ref}/\text{mean inter-arrival time}\) to 0.6, 0.9, and 1.2, corresponding to mean inter-arrival times of 167.406494968, 111.604329979, and 83.703247484. Actual Crane load depends on locations, inventory depletion, and the selected route, so these are reference load levels.

Due-date allowance is independently varied using factors \(\theta\in\{0.75,1.0,2.0\}\):

\[
d_i-a_i=\theta\,u_i\,q_i\,\tilde c,
\]

where \(q_i\) is requested quantity, \(\tilde c\) is the median DEVS `ready` cycle, and urgency multipliers \(u_i\) range from 0.8 to 1.2. Thus the normalized allowance \((d_i-a_i)/(q_i\tilde c)\) spans 0.6–0.9, 0.8–1.2, or 1.6–2.4 across the three conditions. The `minimum_slack` dispatch rule uses dynamic remaining slack, \(d_i-t-\hat P_i^{remaining}\), when selecting its next order. It is still non-preemptive: the active order finishes before another is selected.

The design uses 16 instances × 3 reference loads × 3 due-allowance factors × 3 replicates = 432 traces. Common random numbers are used: at a given load, arrival times are identical across due-allowance levels; across loads, the same exponential draws are scaled by the inter-arrival mean. Urgency ranks also match across the paired conditions. Each trace is evaluated with the non-preemptive policies and the preemptive minimum-slack policy; Random and rolling ACO use two seeds, while deterministic policies run once, giving 3,888 policy observations. Rolling ACO uses 6 ants × 8 iterations. This remains a cycle-table dispatch-policy comparison, not a test of the proposed urgency-weighted lexicographic objective.

For FCFS, mean local-resolution response was 109.96, 139.12, and 172.78 time units at \(\rho_{ref}=0.6,0.9,1.2\), respectively, and was identical across due-allowance levels because FCFS does not use due dates. This response metric ends at the last local box-out for the stock-pickable portion of the order (or its dispatch decision if no unit is pickable); it is not necessarily full customer-order completion time. Across all policies, average on-time completed orders decreased sharply under tighter due allowances. Rolling ACO improved mean completion counts and rescue-by-due units versus FCFS in most conditions, but its P95 response was consistently higher; its paired P95 increase was statistically distinguishable from zero at \(\rho_{ref}=0.9\) and 1.2, while the intervals included zero at 0.6. The complete paired deltas and 95% intervals are in `results/online_load_slack_paired_geometry_105/online_policy_summary.csv`.

Recreate the paired traces and policy run with:

```powershell
python -m examples.ASRS.online_workload_matrix --instances examples/ASRS/results/allocation_study/instances --output-dir examples/ASRS/instances/online_load_slack_paired_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --seed 12000 --replicates 3 --mean-interarrivals 167.406494968 111.604329979 83.703247484 --due-slack-factors 0.75 1 2 --urgency-spread 0.4 --common-random-numbers
python -m examples.ASRS.online_dispatch_experiment --instance examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_load_slack_paired_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --output-dir examples/ASRS/results/online_load_slack_paired_geometry_105 --seed 12000 --repeats 2 --ants 6 --iterations 8 --partial-penalty-kappa 60 --partial-penalty-epsilon 1 --partial-penalty-cap 5
```

The cycle-boundary preemption pilot is saved in `results/online_preemption_paired_geometry_105/`. It adds `preemptive_minimum_slack`, which re-evaluates every released order after each completed product pick-and-delivery cycle. An order keeps its first L/R station and may wait with partial picks in unlimited staging. There is no interruption within motion and no added fixed switching cost; all Crane travel is charged using the DEVS-derived cycle table. The switching count records leaving an order that still has pickable demand. This is intentionally a simple first policy, and it may switch even when no new order has arrived.

Across 432 identical traces, the new rule did not improve on-time complete orders or response time over `minimum_slack`. At the middle due allowance (factor 1.0), paired mean changes (preemptive minus non-preemptive, CI across the 16 source instances) were:

| Reference load \(\rho_{ref}\) | Complete orders by deadline, change [95% CI] | Rescue units, change [95% CI] | Mean response, change [95% CI] | P95 response, change [95% CI] | Mean switches |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0.6 | -1.29 [-1.73, -0.85] | +1.08 [+0.25, +1.92] | +14.52 [+10.84, +18.20] | +19.26 [+12.04, +26.47] | 11.90 |
| 0.9 | -1.27 [-1.53, -1.01] | +0.94 [+0.26, +1.61] | +25.25 [+20.16, +30.34] | +26.31 [+16.95, +35.66] | 17.63 |
| 1.2 | -1.25 [-1.49, -1.01] | +1.04 [+0.03, +2.06] | +32.76 [+27.55, +37.97] | +26.61 [+15.59, +37.64] | 22.02 |

At the other due-allowance factors, on-time complete-order counts were also lower under preemption in all nine load/slack conditions; eight paired 95% intervals excluded zero. Rescue-unit and weighted partial-penalty changes were less consistent and their intervals generally included zero. Thus this pilot does not support “preemption helps when load is high” for the implemented rule. Instead, the simple rule incurs repeated order switching and worsens complete-order service and response time. The finding is limited to these synthetic traces, fixed initial inventory, this slack estimate, and this switching boundary. It motivates a more selective trigger (for example, switch only when a newly released order's projected on-time completion would otherwise be lost), plus a policy-independent objective evaluator and a separate full-order response metric before further claims.

Reproduce the preemption run with:

```powershell
python -m examples.ASRS.online_dispatch_experiment --instance examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_load_slack_paired_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --output-dir examples/ASRS/results/online_preemption_paired_geometry_105 --seed 12000 --repeats 2 --ants 6 --iterations 8 --partial-penalty-kappa 60 --partial-penalty-epsilon 1 --partial-penalty-cap 5
```

### ACO search-effort check

The middle workload condition (mean inter-arrival 60, due-slack factor 2) was rerun using newly generated, current-geometry traces for each of the same 16 instances. Five ACO seeds per trace were compared at 6 ants × 8 iterations and 12 ants × 25 iterations. This is a search-effort sensitivity check, not a proof of convergence.

| ACO setting | Complete by deadline / 10 | Paired change vs FCFS (95% CI) | Rescue units | Paired change vs FCFS (95% CI) | Mean response | P95 response | ACO compute time per run |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 6 ants × 8 iterations | 5.51 | +1.08 [0.44, 1.71] | 17.71 | -2.66 [-4.89, -0.44] | 191.4 | 413.4 | 0.060 s |
| 12 ants × 25 iterations | 5.78 | +1.34 [0.60, 2.08] | 17.05 | -3.32 [-5.59, -1.06] | 184.6 | 414.1 | 0.353 s |

Across instances, the high-effort run improved the mean number of complete orders by 0.26 [95% CI -0.03, 0.55], reduced rescue units by 0.66 [-1.23, -0.10], and reduced mean response by 6.82 [-10.26, -3.38]. P95 changed by 0.66 [-9.65, 10.96]. The additional ACO computation was 0.293 s per run [0.235, 0.350] on this machine. The complete-order difference is not conclusive at this pilot size, while the rescue and mean-response changes favor additional search. ACO still has a higher P95 than FCFS; average improvements do not remove tail-delay concerns.

The current-geometry matched runs are in `results/online_convergence_low_geometry_105/` and `results/online_convergence_high_geometry_105/`; their 16 traces are in `instances/online_convergence_geometry_105/`.

Reproduce the matched search-effort runs with:

```powershell
python -m examples.ASRS.online_workload_matrix --instances examples/ASRS/results/allocation_study/instances --output-dir examples/ASRS/instances/online_convergence_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --seed 9000 --replicates 1 --mean-interarrivals 60 --due-slack-factors 2 --urgency-spread 0.4
python -m examples.ASRS.online_dispatch_experiment --instance examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_convergence_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --output-dir examples/ASRS/results/online_convergence_low_geometry_105 --seed 9000 --repeats 5 --ants 6 --iterations 8
python -m examples.ASRS.online_dispatch_experiment --instance examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_convergence_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --output-dir examples/ASRS/results/online_convergence_high_geometry_105 --seed 9000 --repeats 5 --ants 12 --iterations 25
```

### Partial-order penalty pilot — post-hoc scoring

The first simple partial-shipment study leaves dispatch decisions and the rolling ACO objective unchanged. It scores each order when its local service ends, using missing product units and the due-date slack at that time:

\[
P_i = R_i\min\left(g_{\max}, 1+\frac{\kappa}{\max(d_i-t_i^p,0)+\epsilon}\right),
\]

where \(R_i\) is the number of requested units that could not be picked from current inventory, \(t_i^p\) is the order's local departure time (the dispatch decision time if no unit is locally available), and \(g_{\max}\) caps the urgency multiplier. The pilot uses \(g_{\max}=5\), \(\epsilon=1\) time unit, and \(\kappa=60\) time units. It also reports the number of distinct missing SKU types, but does not penalize that count yet. The score is an evaluation metric only; it does not influence the policies or ACO's choices.

The same 16 current-geometry traces (mean inter-arrival 60, due-slack factor 2) were replayed with five Random and rolling-ACO seeds per trace and one run per deterministic policy, for 224 policy runs. Mean weighted partial penalties across the 16 source instances were: FCFS 19.42, Random 18.18, EDD 18.85, minimum slack 19.04, nearest pick 18.26, and rolling ACO 18.84. Each policy's paired 95% interval for the penalty change versus FCFS included zero. Rolling ACO had the highest mean deadline-complete count (5.51/10), while Random had the lowest mean partial penalty. This is a tradeoff signal, not evidence that any policy is superior.

The inventory structure fixes the total stock-related shortage at five units in every instance; the penalty distinguishes how those units are distributed across orders and the slack at their departure. It is not the same as `rescue_units`, which also counts available products that missed their due dates. A small post-hoc sensitivity calculation from the saved order-level outcomes gave these mean scores:

| \(\kappa\) (time units) | FCFS | Random | EDD | Minimum slack | Nearest pick | Rolling ACO |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 5.00 | 5.00 | 5.00 | 5.00 | 5.00 | 5.00 |
| 30 | 18.59 | 17.18 | 17.92 | 18.15 | 17.15 | 17.81 |
| 60 | 19.42 | 18.18 | 18.85 | 19.04 | 18.26 | 18.84 |
| 120 | 20.91 | 19.66 | 20.43 | 20.59 | 20.07 | 20.39 |

For every tested nonzero \(\kappa\), paired 95% intervals versus FCFS included zero. This pilot therefore supports testing the metric further, but not selecting its parameters from the observed policy ranking. The raw policy runs and order-level outcomes are in `results/partial_penalty_pilot_geometry_105/`; the order-level file retains each order's departure slack, missing units, missing SKU types, urgency multiplier, and penalty.

Reproduce the pilot with:

```powershell
python -m examples.ASRS.online_dispatch_experiment --instance examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_convergence_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --output-dir examples/ASRS/results/partial_penalty_pilot_geometry_105 --seed 9000 --repeats 5 --ants 6 --iterations 8 --partial-penalty-kappa 60 --partial-penalty-epsilon 1 --partial-penalty-cap 5
```

### Live DEVS and cycle-table replay equivalence

The event-driven replay is checked against a live coupled DEVS model using the same fixed trace, inventory, cycle-time table, and policy. The harness compares each dispatch start, each product box-out, and the final makespan, not just summary statistics. For all five deterministic/stochastic greedy policies (FCFS, Random, EDD, minimum slack, nearest pick), all 16 regenerated traces in `instances/online_convergence_geometry_105/` matched under the current geometry: 80 policy/trace pairs, with maximum absolute timing error below `1.71e-12` time units.

The same 80-pair check was repeated under each alternate motion profile in `data/crane_hardware_scenarios.json`. Maximum errors were `3.75e-12` for the Dematic-derived profile and `1.82e-12` for the Tianjin-derived profile. The comparison keeps the trace's arrivals and deadlines fixed while changing only Crane motion parameters. The trace deadlines were generated using the prior DEVS cycle table; the validator is therefore run with `--allow-reference-table-mismatch` to retain them as fixed workload inputs. This establishes implementation consistency for the checked deterministic state transitions; it is not hardware validation, policy validation for rolling ACO, or evidence that the surveyed values describe an installed crane.

### Next stage — objective evaluator and selective preemption

Use the 432 paired traces in `instances/online_load_slack_paired_geometry_105/` as the shared workload for the next stage. First implement the policy-independent lexicographic evaluator: urgency-weighted on-time complete orders, weighted unfulfilled-by-deadline units, weighted unit tardiness, then local makespan. The urgency-weight parameters must be fixed in the experiment protocol and included in sensitivity analysis before interpreting policy rankings. Keep raw complete-order count alongside the weighted score.

The first cycle-boundary preemption pilot is complete and is reported above. Next, implement the policy-independent evaluator for urgency-weighted on-time complete orders, weighted unfulfilled-by-deadline units, weighted unit tardiness, and local makespan. Then compare a selective preemption trigger with both the current non-preemptive baseline and the tested every-cycle reselection rule. Keep each order's L/R station assignment fixed, let partially fulfilled orders wait in unlimited staging, and use only released orders and current inventory. The model still has no actual backup-center queue; units not shipped locally by the due date remain a rescue-workload proxy. Report full-order completion response separately from partial local-departure response; the current `mean_response_time` metric is local-resolution time for pickable stock, not necessarily full customer-order completion.

Commands (run from the repository root):

```powershell
python -m examples.ASRS.online_devs_validation --instance examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_convergence_geometry_105 --cycle-times examples/ASRS/data/crane_cycle_times.json --output-dir examples/ASRS/results/online_devs_validation_geometry_105 --seed 42
```

### Crane parameter sensitivity pilot

The motion profiles are stored in `data/crane_hardware_scenarios.json`. The current DEVS baseline uses 2 m/s and 1 m/s² on each modeled axis. The Dematic-derived scenario uses reported travel/hoist/fork speeds and accelerations where available; fork acceleration is imputed from the SOCS default. The Tianjin-derived scenario uses the preliminary reported travel, hoist, and loaded-fork values. Source version, load condition, and full acceleration/deceleration curves have not been independently verified. The model assumes symmetric trapezoidal/triangular profiles. Physical X travel maps to DEVS X; physical Y hoist maps to DEVS Z and the short fork-lift motion; physical Z fork reach maps to DEVS Y. The DEVS Y and lift motions share the respective fork-reach and hoist parameter sets.

For a first sensitivity pass, the 16 online-convergence workloads (mean inter-arrival 60, due-slack factor 2) were replayed with fixed arrivals/deadlines under each profile. FCFS and rolling ACO illustrate the response to changed cycle durations:

| Motion profile | Policy | Complete by deadline / 10 | Rescue units | Mean response | P95 response | Full-run makespan | Paired makespan change vs current (95% CI) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Current DEVS baseline | FCFS | 3.88 | 24.00 | 232.64 | 391.58 | 794.05 | baseline |
| Dematic-derived | FCFS | 3.75 | 26.19 | 245.29 | 415.85 | 820.48 | +26.43 [22.02, 30.83] |
| Tianjin-derived | FCFS | 2.19 | 38.38 | 396.66 | 676.03 | 1090.27 | +296.22 [279.51, 312.93] |
| Current DEVS baseline | Rolling ACO | 5.53 | 20.50 | 221.21 | 509.44 | 785.74 | baseline |
| Dematic-derived | Rolling ACO | 5.28 | 22.00 | 236.01 | 527.14 | 814.03 | +28.29 [23.11, 33.48] |
| Tianjin-derived | Rolling ACO | 3.09 | 34.59 | 383.15 | 833.83 | 1088.18 | +302.44 [284.55, 320.34] |

The alternate profiles lengthen the model's cycles, and the more restrictive loaded-fork profile markedly lowers deadline completion and increases residual rescue units. These changes are conditional on the selected synthetic trace/deadline set and motion assumptions; they are not predictions of commercial AS/RS performance. Rolling ACO's lower response mean than FCFS in the baseline does not remove its larger P95, and altered timings change its schedule as well. The current geometry uses a 1.05 m fork stroke and a rail center at `y=-1.25 m`; earlier online-pilot tables in this plan were generated for the prior 1.25 m stroke and should be regenerated before reuse as current-model evidence.

Reproduce the cycle-table generation, paired sensitivity study, and DEVS cross-checks with:

```powershell
python -m examples.ASRS.hardware_sensitivity --instances examples/ASRS/results/allocation_study/instances --trace-dir examples/ASRS/instances/online_convergence --output-dir examples/ASRS/results/hardware_sensitivity_iat60_slack2 --scenario-file examples/ASRS/data/crane_hardware_scenarios.json --seed 42 --repeats 2 --ants 6 --iterations 8
```

The generated per-profile cycle tables, raw runs, and paired summary are saved under `results/hardware_sensitivity_iat60_slack2/`. Treat these results as an initial parameter sensitivity analysis. Calibration requires an identified machine's verified specifications and preferably measured cycle times, including payload condition and handling delays.
