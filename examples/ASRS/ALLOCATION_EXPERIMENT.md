# AS/RS Order Allocation Experiment

The summary values below are the original allocation-study run using the legacy constant-speed schedule evaluator. The current comparison command uses `data/crane_cycle_times.json` when present; rerun it before comparing new makespans with these historical values.

This document describes the single-fork AS/RS comparison of random allocation, a deterministic heuristic, and ACO. The experiment asks whether a policy can concentrate unavoidable SKU shortages in fewer orders, complete more orders, and reduce the route makespan.

## How the 16 instances differ

The benchmark crosses four binary settings, producing 16 saved instances:

| Setting | Values | How it changes an instance |
| --- | --- | --- |
| Active SKU families | 3 or 5 | Uses the first 3 (`red`, `blue`, `green`) or all 5 current families. Total stock is 50 units: 17/17/16 across three families, or 10 per family across five. |
| Order-size profile | Balanced or variable | Balanced has five orders requesting 6 units and five requesting 5. Variable uses 2, 3, 4, 5, 5, 6, 6, 7, 8, and 9 units. Both profiles total 55 units. |
| SKU overlap across orders | Spread or clustered | Spread distributes each SKU's demand across as many orders as capacity allows. Clustered fills shuffled orders with one SKU before moving to the next SKU, creating more concentrated SKU mixes. |
| Shortage pattern | Spread or concentrated | The five units of total shortage are spread across the active SKUs (2/2/1 for three families, or 1 each for five), or all five are added to one randomly selected active SKU. |

All instances have 10 orders, 50 stored products, and total requested demand of 55. Product cells are randomly selected from the 15-by-6 rack using a scenario-specific generation seed, so the shelf locations differ between instances. The order-size profile and SKU-allocation settings also produce different order demands. Exact orders, SKUs, and cell positions are preserved in each JSON file under `results/allocation_study/instances/`.

The 3-SKU/5-SKU axis only changes the active families within the five SKU families supported by the current model. This experiment does not yet test more than five families.

Within each instance, all three methods receive exactly the same orders, product stock, SKU labels, and cell positions. Only their scheduling/allocation decisions differ.

## Decisions made by each policy

### Random allocation

The random baseline makes these decisions randomly while respecting SKU compatibility:

1. Shuffles the order processing priority.
2. For each order, randomly chooses among the remaining shelf products whose SKU the order still requires. It assigns products until the demand is met or no matching inventory remains.
3. Randomly chooses L or R for each order.
4. Randomly orders the assigned products within each order.

This is a randomized greedy policy: after it processes an order, its assigned inventory is no longer available to later orders. It does not optimize a score or learn from earlier runs.

### Deterministic heuristic

The deterministic policy uses fixed rules, with no random choices or pheromone learning:

1. If any remaining order can be completed from current SKU stock, selects the feasible order with the highest scarcity score `sum(order SKU demand / remaining SKU stock)`. Ties prefer the smaller order, then the lower order ID.
2. If no order is fully feasible, selects the order with the largest currently fulfillable fraction. Ties prefer the lower order ID.
3. For the selected order, takes the required SKU units that are available. For each SKU, it chooses the nearest matching cells from the crane's current position.
4. Routes those products using nearest-cell selection and chooses the L/R station with the shorter estimated route.

The feasible-first and scarcity rules are hand-coded. They are a simple baseline, not a guarantee of a globally maximum number of complete orders.

### Ant Colony Optimization (ACO)

ACO jointly searches the order sequence, the product-to-order assignment, product cells, and L/R stations:

1. **Order sequence:** at each step, samples a remaining order using pheromone for the previous-order-to-next-order transition. Its heuristic favors an order that is currently fully feasible and favors smaller total demand.
2. **SKU allocation and cell selection:** for the selected order, considers remaining products with a required SKU. Sampling weights combine product-transition pheromone and inverse estimated trip time. A selected shelf product is assigned to that order and removed from shared inventory.
3. **Station selection:** on an order's first pickup, both L and R are candidate actions. The first-pick station is recorded and kept for that order's remaining picks. Station pheromone and estimated travel time affect the sampling weight.
4. **Pickup order:** each selected product becomes the next pickup. The crane returns to the order's selected station after each product in this single-fork model.
5. **Learning:** after each iteration, pheromones evaporate. The top quarter of that iteration's ant schedules deposit rank-weighted pheromone on order transitions, product transitions, and order stations.

The ACO score guarantees **maximizing complete orders as the first priority**. After the complete-order count is fixed, it minimizes the weighted sum `lambda × shortage units + makespan`; those two terms can trade off. In this benchmark, lambda is 100 simulation-time units per missing product, so one fewer missing product is worth 100 simulation-time units. Since the maximum makespan bound is 875, lambda 100 does not make shortage strictly higher priority than makespan: a schedule may accept one more missing unit if it saves more than 100 time units. To guarantee shortage priority over any makespan change with a scalar score, lambda must exceed the maximum makespan range (876 is sufficient for this model); alternatively, compare the tuple `(incomplete orders, shortage units, makespan)` lexicographically. ACO runs with 12 ants and 25 iterations per seed.

## Metrics and result interpretation

The table below averages equally across the 16 instances. Random allocation and ACO each have three seeded runs per instance (48 runs each). The deterministic policy runs once per instance (16 runs). In this experiment, `M=6376` and `lambda=100` for every instance. The incomplete-order term is `M × (partial orders + unprocessed orders)`. Here `M = lambda × total requested units + maximum makespan bound + 1 = 100 × 55 + 875 + 1`. This makes one fewer incomplete order outweigh any possible shortage and time changes. The weighted shortage penalty is `lambda × shortage units`. The final objective is:

`J = M × (partial orders + unprocessed orders) + lambda × shortage units + makespan`

The component and total objective columns show means across runs and instances. The objective is reported for all three policies using the same formula; only ACO uses it to guide its search.

| Method | Complete orders / 10 | Partial orders | Unprocessed orders | `M × incomplete orders` | Shortage units | `lambda × shortage` | Mean makespan | Mean objective `J` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Random | 7.69 | 2.08 | 0.23 | 14,744.50 | 5.00 | 500.00 | 482.70 | 15,727.20 |
| Deterministic heuristic | 7.88 | 1.69 | 0.44 | 13,549.00 | 5.00 | 500.00 | 349.96 | 14,398.96 |
| ACO | 8.25 | 1.63 | 0.12 | 11,158.00 | 5.00 | 500.00 | 312.53 | 11,970.53 |

Every policy ships all 50 units in these runs. Because all demand totals 55 and all SKU inventory can be assigned to a matching order, the total shortage is fixed at 5; the penalty column therefore does not distinguish the methods. The allocation effect appears in how those five missing units are distributed among orders and how many orders are fully completed. A finite operating horizon or pickup budget would be needed to make total shortage vary by policy in this model.

## Reproduce the experiment

From the project root:

```powershell
python -m examples.ASRS.compare_allocation_methods --seed 42 --repeats 3 --ants 12 --iterations 25
```

The command writes 16 readable JSON instances, per-run results, and scenario/method averages under `examples/ASRS/results/allocation_study/`:

- `allocation_methods_raw.csv`: each random and ACO seed run, plus each deterministic run.
- `allocation_methods_summary.csv`: mean metrics for each method in each instance.
- `instances/*.json`: fixed orders, SKU demands, product stock, and cell locations.

The schedule evaluator stops after the last pickup event, so repeated candidate evaluation does not advance the simulator to its distant time bound.
