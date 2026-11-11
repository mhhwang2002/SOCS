# AS/RS with a Two-Fork Crane

This is an independent AS/RS example. It does not change `examples/ASRS`.

## Model

- Each rack cell is a passive DEVS atomic model that reports its initial occupancy and transfers its box on request.
- `TwoForkCrane` has one shared X rail and Z hoist, plus independently controlled Fork1 and Fork2 reach/lift/load states.
- Fork spacing is one cell pitch. For adjacent bays at the same level, the rail stops at their midpoint: Fork2 retrieves the left cell and Fork1 the right cell. A single-cell retrieval uses Fork1.
- `ASRSController` waits for cell and crane initial status, then dispatches retrieval tasks and waits for the crane to return idle.
- `make_asrs_model` couples cell status, retrieval requests, box transfers, crane commands, idle notices, and station outputs.

## Optimize the saved demo instance

Regenerate the kinematic timing table when crane speeds, accelerations, rack geometry, or fork dimensions change:

```powershell
python -m examples.ASRS_2ForkCrane.preprocess_timing
```

The preprocessor advances a SOCS DEVS timing probe through movement and service intervals derived from the same `TwoForkCrane` acceleration profiles used by the coupled model. It writes a versioned JSON table under `examples/ASRS_2ForkCrane/data/`. The table records travel between L/R and cell poses, cell-to-cell movement, fork approach, and the Fork Lift-to-Fork Down cycle to either station. The optimizer rejects a profile if its saved kinematic or geometry parameters differ from the current crane model.

The default timing profile can be overridden with `--timing-profile PATH`. The default station rule, `--station-policy first-pickup`, chooses L/R at the first pickup of an order. `--station-policy whole-order` builds the order's pickup groups first, then chooses a station from the full route cost.

## 1. Single-unit and double-capable policies on TwoForkCrane

Single-unit picking means carrying one product per trip; it does not mean the crane has only one fork installed. Both policies in this section use the same TwoForkCrane. The `single` policy uses Fork1 for one product per trip. The `double-capable` policy requires a simultaneous two-product pickup whenever a feasible adjacent pair exists; it falls back to one product when no pair is feasible.

Both policies use `demo.json`, 12 ants, 100 iterations, lambda 100, makespan objective, first-pickup station selection, and seeds 11, 42, and 73. All runs shipped 50/55 products and completed 8/10 orders. A paired pickup counts one trip that retrieves two products at once.

| Seed | Single-unit actual makespan | Double-capable actual makespan | Paired pickups |
| ---: | ---: | ---: | ---: |
| 11 | 845.346 | 579.527 | 18 |
| 42 | 831.346 | 571.120 | 18 |
| 73 | 843.346 | 544.433 | 18 |
| Mean | 840.013 | 565.027 | 18 |

For this demo and these seeds, requiring simultaneous pickups when possible reduced mean actual makespan by 32.7%. This is an end-to-end policy comparison: ACO independently selects the order sequence, products, and stations under each mode. It does not isolate the pickup mechanism while holding those decisions fixed.

## 2. Single, Double, Sequential, All

The four modes define which pickup action is allowed at each decision point. Feasibility uses the current order's remaining SKU demand and the inventory that has not yet been assigned.

| Mode | Operation |
| --- | --- |
| **Single** | Fork1 retrieves one product, then the crane unloads it. Each product uses its own trip. |
| **Double** | If two same-order products are available in adjacent bays at the same level, Fork1 and Fork2 retrieve them simultaneously. If no such pair is feasible, the crane uses Single. ACO chooses among feasible simultaneous pairs; it does not choose a single pickup while a pair is available. |
| **Sequential** | If two products for the current order are available, Fork1 retrieves the first, then Fork2 retrieves the second from another cell; both are unloaded together. Single is used only when no second product is feasible. The second cell can be at another bay or level. The crane may remain at the same X/Z pose if the fork offsets align with both cells; the two cell requests still occur separately. |
| **All** | Strict priority: Double if a simultaneous pair is feasible; otherwise Sequential if a two-product route is feasible; otherwise Single. ACO samples only among actions at the highest available tier. |

Double and All both force simultaneous pickup whenever an eligible pair is available. Sequential and All force a two-product sequential trip when no simultaneous pair is available but a second product for the current order is feasible. These modes do not combine products for different orders or visit more than two cells in one trip.

Run a mode with a saved instance and fixed ACO settings by changing `--picking-mode`:

```powershell
python -m examples.ASRS_2ForkCrane.aco_optimization --instance examples/ASRS/instances/demo.json --time-objective makespan --penalty-weight 100 --seed 42 --iterations 100 --picking-mode single
python -m examples.ASRS_2ForkCrane.aco_optimization --instance examples/ASRS/instances/demo.json --time-objective makespan --penalty-weight 100 --seed 42 --iterations 100 --picking-mode double
python -m examples.ASRS_2ForkCrane.aco_optimization --instance examples/ASRS/instances/demo.json --time-objective makespan --penalty-weight 100 --seed 42 --iterations 100 --picking-mode sequential
python -m examples.ASRS_2ForkCrane.aco_optimization --instance examples/ASRS/instances/demo.json --time-objective makespan --penalty-weight 100 --seed 42 --iterations 100 --picking-mode all
```

For the demo instance with seed 42, 12 ants, and 100 iterations, each policy independently generated its schedule:

| Mode | Actual makespan | Complete orders | Shortage units | Trips / pickup mix |
| --- | ---: | ---: | ---: | --- |
| Single | 831.346 | 8/10 | 5 | 50 single |
| Double | 571.120 | 8/10 | 5 | 32 trips: 18 simultaneous, 14 single |
| Sequential | Recompute required | 8/10 | 5 | Recompute required |
| All | Recompute required | Recompute required | Recompute required | Recompute required |

`Shortage units` is the total number of requested product units that were not shipped, summed across orders and SKUs. For each SKU, it is `max(requested quantity - shipped quantity, 0)`. For example, an order requesting 7 products and shipping 5 has a shortage of 2, so it incurs a shortage penalty of `2 × lambda`. Shortage units are not the number of partial orders or the number of distinct SKU types. The objective also has a separate priority term for the number of incomplete orders.

The Sequential SKU-demand constraint was corrected so a pickup group cannot ship more units of an SKU than the current order still requires. With the demo instance, seed 42, 12 ants, 100 iterations, makespan objective, and lambda 100, the corrected Sequential result completes 8/10 orders with 5 shortage units. The two partial orders are:

| Order | Requested | Shipped | Shortage | SKU detail |
| --- | ---: | ---: | ---: | --- |
| 2 | 6 | 4 | 2 | 5 SKU types requested; short 1 blue and 1 white |
| 9 | 7 | 4 | 3 | 4 SKU types requested; short 1 blue and 2 white |
| **Total** | **13** | **8** | **5** | **2 partial orders** |

The previous Sequential shortage value of 9 was produced by the SKU-demand bug and is invalid. The previous Sequential and All makespan/trip figures, and the pairability comparison results below, were generated before this correction and must be recomputed before being used. All modes ship the same total inventory only when their order allocations permit it; shortage can differ because ACO allocates shared inventory among orders differently.

### Controlled inventory pairability

Generate four fixed layouts that preserve the same product SKUs and order demands but vary adjacent occupied cell pairs:

```powershell
python -m examples.ASRS_2ForkCrane.generate_pairability_instances
```

The generator writes `pairability_almost-none.json` (4 adjacent pairs, the practical minimum with 50 products in this 15-by-6 rack), `pairability_low.json` (8), `pairability_medium.json` (20), and `pairability_high.json` (40). Each file records its layout seed and raw/order-compatible pair counts in `layout_scenario` metadata.

Compare the four modes with:

```powershell
python -m examples.ASRS_2ForkCrane.compare_pairability_instances --seeds 42 43 44 --ants 6 --iterations 20 --penalty-weight 100
```

The script saves all runs to `examples/ASRS_2ForkCrane/instances/pairability/comparison.csv`. This initial screen reports mean estimated makespan across three ACO seeds; the last column also shows mean All shortage units.

| Layout (adjacent pairs) | Single | Double | Sequential | All | All shortage |
| --- | ---: | ---: | ---: | ---: | ---: |
| Almost none (4) | 862.559 | 831.517 | Stale; rerun | Stale; rerun | Stale; rerun |
| Low (8) | 868.367 | 754.336 | Stale; rerun | Stale; rerun | Stale; rerun |
| Medium (20) | 898.992 | 681.674 | Stale; rerun | Stale; rerun | Stale; rerun |
| High (40) | 911.811 | 594.292 | Stale; rerun | Stale; rerun | Stale; rerun |

The Single and Double columns are unchanged by the Sequential SKU-demand correction. The Sequential and All columns are retained here only as explicitly stale placeholders until the comparison is rerun. The small search budget and three seeds make these preliminary results. Compare completion, shortage, and makespan together; at lambda 100, lower makespan can come with more shortage units.

**Research note:** To isolate pickup-mode effects, hold order priority, product/SKU assignments, pickup sequence, and unloading stations fixed, then replay that same schedule under Single, Double, and Sequential. The ACO mode comparisons above optimize those decisions independently, so they compare complete operating policies rather than only the pickup method.

The animation runner accepts the same `--picking-mode` option. Add `--headless` to execute the coupled DEVS model without opening Panda3D. During a Sequential trip, the animation shows the first load remaining on Fork1 as the crane travels to retrieve the second product with Fork2.

## Run the coupled model

```powershell
python -m examples.ASRS_2ForkCrane.run_aco_animation --instance examples/ASRS/instances/demo.json --objective makespan --penalty-weight 100 --seed 42
```

In the viewer, `A` runs continuously, `I` steps interactively, `Space` resumes, `V` toggles continuous animation, `S` shows statistics, `R` resets the camera, and `Esc`/`Q` closes the viewer.

The ACO schedule evaluation uses the preprocessed DEVS kinematic travel and fork-cycle times, plus the controller's 10-second startup delay. Regenerate the timing profile after changing crane speed, acceleration, geometry, or fork dimensions.
