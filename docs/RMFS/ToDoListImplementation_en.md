# Implementation tasks

As of: 2026-11-19

Schedule: TargetDate values are provisional targets based on priority and may change as research progresses.

Preserve existing runnable examples and experimental results. Discuss substantial architectural changes or objects with new responsibilities with the user first. Validating traffic control on fixed paths takes priority over adding path-generation strategies.

## Priority 1 — Strengthen independent validation of dynamic execution

**TargetDate: 2026-11-20**

- Develop a validator that rechecks safety at approval time independently of the departure-decision code. **TargetDate: 2026-11-20**
- Check original control-subpath order, interference precedence, command endpoints, and final arrivals. **TargetDate: 2026-11-20**
- Check that RoadMap residual sweeps conservatively contain actual occupancy and approved remaining movement. **TargetDate: 2026-11-20**
- Recheck every distinct object pair in each constructed tuple and merged result. **TargetDate: 2026-11-20**
- Run collision validation with collision checking ON and collision stopping ON for both MRobot and PSensor. **TargetDate: 2026-11-20**

## Priority 2 — Systematically test small cases

**TargetDate: 2026-11-23**

- Vary paths, starting positions, initial headings, and speeds in two- and three-object cases. **TargetDate: 2026-11-23**
- Include shared passages in the same and opposite directions, crossings, endpoint interference, and cyclic requirements. **TargetDate: 2026-11-23**
- Record failure to complete in finite time and deadlock as failures, in addition to collisions. **TargetDate: 2026-11-23**
- Preserve discovered counterexamples as regression tests with fixed seeds, inputs, and snapshots. **TargetDate: 2026-11-23**
- Compare DEVS-only runs with collision-validation runs. Do not introduce periodic time loops for control. **TargetDate: 2026-11-23**

## Priority 3 — Traceability and reproduction

**TargetDate: 2026-11-24**

- Record the specific reason for each wait and which resource release resolves it. **TargetDate: 2026-11-24**
- Trace original control subpath sweeps, merged movement commands, and RoadMap reported state separately. **TargetDate: 2026-11-24**
- Inspect group splitting, objects becoming independent, and the link between departure approvals and clearance reports. **TargetDate: 2026-11-24**
- Document debugging by replay from a snapshot immediately preceding a problem. **TargetDate: 2026-11-24**
- Extend snapshot-restore and reset-reproduction tests to more cases. **TargetDate: 2026-11-24**

## Priority 4 — Use dynamic groups in actual control

**TargetDate: 2026-11-25**

- RoadMap currently computes the residual interference graph and connected components and displays them in the Viewer. **TargetDate: 2026-11-25**
- Group information is not yet used to change departure conditions or the execution schedule. **TargetDate: 2026-11-25**
- Apply independent control per component after establishing the theoretical separation conditions. **TargetDate: 2026-11-25**
- Check safety, full completion, and preservation of the original control subpath sweeps before and after this change. **TargetDate: 2026-11-25**

## Priority 5 — Performance evaluation and optimization

**TargetDate: 2026-11-27**

- Expand the 2x2 comparison of sweep merging and clearance-driven early departure across scenarios and seeds. **TargetDate: 2026-11-27**
- Collect makespan, simultaneous movement counts, waiting times and reasons, command counts, and wall-clock runtime. **TargetDate: 2026-11-27**
- Verify identical input paths, original tuple paths, and options; keep raw results from before and after changes separate. **TargetDate: 2026-11-27**
- Establish correctness of the simple all-pairs dynamic-group implementation before considering incremental updates and caching. **TargetDate: 2026-11-27**
- Check runtime and memory for regression runs with many objects. Do not describe small-case success as completed large-scale validation. **TargetDate: 2026-11-27**
- Synchronize agreed theory and implementation changes into the English LaTeX document. **TargetDate: 2026-11-27**

## Priority 6 — Prototype optimal fixed-path control and compare search methods

**TargetDate: 2026-12-04 — first small-case prototype and comparison; provisional**

- Preserve the existing fixed-path examples and controller as baselines. Define whether each experiment fixes geometric paths only or fixes an existing tuple path and its precedence constraints. Record the optimization freedom explicitly. **TargetDate: 2026-12-04**
- Prototype exact search on small finite two- and three-object cases, using elapsed-time costs and admissible lower bounds. Compare A*, an uninformed/reference exact search, and bounded simulation-guided heuristics; report the model within which an optimum is claimed. **TargetDate: 2026-12-04**
- Reuse actual framework snapshots and the existing Plant/controller models when evaluating successor controls. Preserve motion and scheduling state; do not create duplicate physical robots or a different simplified simulator without first discussing the design. **TargetDate: 2026-12-04**
- Measure makespan, search/branch counts, wall-clock time, peak memory, and any optimality gap against the exact small-case reference. Separate infeasible cases from search-budget exhaustion and unfinished computation. **TargetDate: 2026-12-04**
- Validate selected controls on the real DEVS Plant with collision checking and collision stopping ON for both MRobot and PSensor, and require all destinations to be reached. Include tests where waiting/precedence choices change completion time. **TargetDate: 2026-12-04**
- Document the proposed optimizer's interface before introducing a new object or changing TrafficManager. This is a planned prototype, not an already implemented optimal controller. **TargetDate: 2026-12-04**

## Priority 7 — Introduce RobotPathPlanner for subsequent tasks

**TargetDate: 2026-12-07 — first planning baseline and task-transition example; provisional**

- First propose the RobotPathPlanner interface and its integration inside RMFSController. It will consult shared RoadMap; TrafficManager will retain responsibility for safe execution of the generated path. Obtain agreement on the design before implementation. **TargetDate: 2026-12-07**
- Add a separate test where an object completes its path, remains as a point sweep, receives another task/destination, and starts a newly generated path. Keep existing finite fixed-path tests intact. **TargetDate: 2026-12-07**
- Build a motion-time planning baseline and compare it with distance-only A*. Include heading/turn costs and continuous straight-run profiles without changing the original sensing/control points. Use current configurable robot motion parameters. **TargetDate: 2026-12-07**
- Add traffic-aware candidate evaluation with RoadMap residual sweeps, occupied goal footprints, waiting, and detours. If snapshot lookahead is used, evaluate the relevant ongoing interactions rather than only an isolated robot. **TargetDate: 2026-12-07**
- Before issuing a new path, validate its source against the actual reported state, recheck interference and precedence feasibility, and update RoadMap consistently. Do not remove the current occupied point merely because planning has begun. **TargetDate: 2026-12-07**
- Recompute affected interference groups after reassignment; test that new edges and group merges are allowed at that boundary, while fixed-path execution still only removes edges. Include snapshot/restore across task reassignment. **TargetDate: 2026-12-07**
- Compare planned versus realized arrival times, computation cost, and fleet makespan across repeated tasks and reproducible scenarios. Establish a new-path baseline before claiming improvements from lookahead or optimized control. **TargetDate: 2026-12-07**

See [DoneListImplementation_en.md](DoneListImplementation_en.md) for completed implementation and validation scope.

## Deferred research — Deadlock avoidance

- Evaluate low-cost deadlock-avoidance rules for route admission and approval extensions. Start with opposite-direction single aisles and waits for the next order leg.
- Evaluate the optional completion-plan search on cases where local rules are insufficient. Measure its computation cost before considering warehouse deployment.
