# Theory research tasks

As of: 2026-11-19

Schedule: TargetDate values are provisional targets based on priority and may change as research progresses.

The current scope is moving objects that advance in order along given fixed paths. Time is abstracted away while movement order is preserved. Safety, eventual completion, and efficiency are distinct concerns.

## Priority 1 — Formalize residual sweeps and dynamic groups

**TargetDate: 2026-11-20**

- Define residual subpaths, residual sweeps, and the residual interference graph precisely. **TargetDate: 2026-11-20**
- Prove that residual sweeps monotonically shrink under forward progress along fixed paths. **TargetDate: 2026-11-20**
- Establish that interference edges can only disappear and that connected components can only remain unchanged or split. **TargetDate: 2026-11-20**
- Include the point sweep occupied by an object after it reaches its destination. **TargetDate: 2026-11-20**
- Explicitly exclude new path assignments, reverse movement, path changes, and changes in object dimensions from these assumptions. **TargetDate: 2026-11-20**

## Priority 2 — Preserve safety and eventual completion in asynchronous execution

**TargetDate: 2026-11-23**

- Establish conditions for replacing stage barriers in a validated tuple path with execution driven by clearance reports. **TargetDate: 2026-11-23**
- Specify which ordering and interference relations among the original control subpath sweeps must be preserved. **TargetDate: 2026-11-23**
- Find conditions proving that an approved move not only is currently safe but also preserves the ability to finish without later deadlock. **TargetDate: 2026-11-23**
- Include occupancy by moving objects, waiting objects, and objects already at their destinations. **TargetDate: 2026-11-23**
- Distinguish connected components of the interference graph from cycles in actual waiting relations. Membership in the same component does not imply deadlock. **TargetDate: 2026-11-23**

## Priority 3 — Relate sweep merging to traffic control

**TargetDate: 2026-11-24**

- Retain the connectivity condition and pairwise disjointness of merged sweeps for n-object tuples. **TargetDate: 2026-11-24**
- State clearly that sweep merging groups movement commands without changing the original control subpath sweeps. **TargetDate: 2026-11-24**
- Characterize constructions in which a blocked later subpath cannot delay departure along an already permitted first subpath. **TargetDate: 2026-11-24**
- Prove conditions preserving safety and eventual completion when only permitted prefixes are combined into commands. **TargetDate: 2026-11-24**
- Treat the existence of a safe merge, completeness of merge search, and makespan optimality separately. Do not assume merging always reduces makespan. **TargetDate: 2026-11-24**

## Priority 4 — Generalize feasibility conditions

**TargetDate: 2026-11-27**

- Extend the unit path-pair classification from one common passage to multiple interference intervals. **TargetDate: 2026-11-27**
- Distinguish centerline-path intersections from footprint sweep intersections and state the applicable assumptions. **TargetDate: 2026-11-27**
- Identify additional conditions needed to move from pairwise feasibility to feasibility for n objects. **TargetDate: 2026-11-27**
- Go beyond the sufficient acyclic static precedence condition to distinguish solvable and unsolvable cyclic cases. **TargetDate: 2026-11-27**
- Distinguish necessary conditions, sufficient conditions, equivalences, and empirical observations. **TargetDate: 2026-11-27**

## Priority 5 — Makespan-optimal control for given paths or a given tuple path

**TargetDate: 2026-12-01 — initial formulation and algorithm review; provisional**

- Terminology: use **tuple path** for 묶음경로 and **paired path** for its two-object case. Distinguish a given set of geometric paths from a given tuple path: the latter already specifies some waiting and execution order. State which choices the optimizer may change. **TargetDate: 2026-12-01**
- Formulate minimum makespan as the time until every object finishes its assigned path, subject to collision avoidance and eventual completion. Identify allowed decisions: waiting, precedence, permitted motion prefixes, merging, and any speed control. Do not silently allow geometric rerouting in the fixed-path problem. **TargetDate: 2026-12-01**
- Investigate whether the problem can be solved by exact graph search such as A*, Dijkstra's algorithm, or branch-and-bound, rather than only simulation-guided heuristics. Identify the assumptions and finite abstraction needed for completeness and optimality; do not assume A* makes the joint problem computationally easy. **TargetDate: 2026-12-01**
- Specify a sufficient joint search state: progress, heading, velocity/motion phase, outstanding commands, occupancy/reservations, and relevant scheduled events. Define safe successors and elapsed-time costs. Avoid treating the number of branches or geometric distance as makespan. **TargetDate: 2026-12-01**
- Develop admissible completion-time lower bounds, initially using the maximum of individual minimum remaining travel times with interference ignored. Establish when the state and heuristic support correct A* search. **TargetDate: 2026-12-01**
- Compare exact finite search with bounded snapshot lookahead and heuristic control. Distinguish an optimum within a chosen finite model, an approximation, and a merely feasible result. Identify where motion dynamics or concurrency invalidate simple additive costs. **TargetDate: 2026-12-01**

## Priority 6 — Time-optimal new-path generation: RobotPathPlanner

**TargetDate: 2026-12-03 — initial formulation and planning model; provisional**

- Define the next-task transition: after completing a path, the object retains its occupied destination point sweep; when a new task assigns a destination, RobotPathPlanner generates a new path from the current physical state. Specify assignment and planning times. **TargetDate: 2026-12-03**
- Distinguish shortest geometric distance, minimum travel time without interference, earliest arrival under current traffic, and minimum fleet makespan. State which objective a new-path decision optimizes. **TargetDate: 2026-12-03**
- Formulate motion-time costs using acceleration, cruising, deceleration, heading, and 90/180-degree rotation. Account for continuous straight travel through intermediate vertices; summing stop-to-stop edge times must not impose unintended stops. **TargetDate: 2026-12-03**
- Study heading/velocity-aware states and direct straight-run edges, retaining original intermediate control points for sensing and traffic control. Establish when the resulting costs are valid for A* and how to construct optimistic time heuristics. **TargetDate: 2026-12-03**
- Incorporate RoadMap occupancy and other objects' residual paths. Compare waiting with detours using time-dependent search and/or snapshot lookahead; spatial overlap alone must not be equated with actual collision or unavoidable delay. **TargetDate: 2026-12-03**
- Study feasibility admission for a new path, including pairwise checks and multi-object precedence cycles. A detected static cycle is not automatically proof of infeasibility. New paths can create interference edges and merge groups; the fixed-path shrinking result no longer applies across reassignment. **TargetDate: 2026-12-03**
- Keep path generation and fixed-path execution as separate responsibilities of RobotPathPlanner and TrafficManager, sharing RoadMap. Specify what must be revalidated before a newly planned path can be commanded. **TargetDate: 2026-12-03**

## Research workflow

1. Search for counterexamples in small two- and three-object cases. **TargetDate: Ongoing (from 2026-11-20)**
2. Develop definitions and preservation properties that explain those counterexamples. **TargetDate: Ongoing (from 2026-11-20)**
3. Connect proved conditions to implementation checks and regression tests. **TargetDate: Ongoing (from 2026-11-20)**
4. Study efficiency and optimality after establishing correctness and eventual completion. **TargetDate: Ongoing (from 2026-11-20)**

See [DoneListTheory_en.md](DoneListTheory_en.md) for completed work and its scope.
