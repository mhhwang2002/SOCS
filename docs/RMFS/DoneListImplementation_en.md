# Completed implementation

As of: 2026-11-19

Date convention: AchievedDate is recorded as 2026-11-19 where the exact completion date is unknown. The assumptions and validation scope stated for each item still apply.

This list covers the new RMFS implementation. It does not count all features of `RMFS_Old` as completed features of the new system.

## 1. Basic Plant and controller architecture

**AchievedDate: 2026-11-19**

- Connected one RMFSController and one MRobot/PSensor pair per object inside the CoupledDEVS RMFS model. **AchievedDate: 2026-11-19**
- Implemented movement, rotation, waiting, and acceleration/deceleration motion profiles in the MRobot DEVSph model. **AchievedDate: 2026-11-19**
- Checked continuous straight travel, bent paths, and 180-degree turns with single-object examples. **AchievedDate: 2026-11-19**
- PSensor computes meaningful control-point clearance times and reports position without a periodic control-point release loop. **AchievedDate: 2026-11-19**
- The ordinary TrafficManager object inside RMFSController generates commands; the controller emits commands from its buffer. **AchievedDate: 2026-11-19**
- RoadMap distinguishes approval from actual Plant reports. Approval alone does not change position. **AchievedDate: 2026-11-19**

## 2. Path/sweep data and duplication validation

**AchievedDate: 2026-11-19**

- Implemented Path, PathPosition, Subpath, and explicit TuplePath data using n-dimensional floating-point vectors. **AchievedDate: 2026-11-19**
- The current sweep backend handles fixed rectangular footprints moving along 2D coordinate axes. **AchievedDate: 2026-11-19**
- WaitDuplicationSolver performs finite search over point-sweep duplications in a fixed original constituent sequence. **AchievedDate: 2026-11-19**
- A separate validate step rechecks original path order, point-wait duplication, and disjoint sweeps for every distinct object pair. **AchievedDate: 2026-11-19**
- Exhaustiveness is limited to the given finite constituent sequence and interference-free tuple semantics. The solver does not explore every possible continuous-time operation. **AchievedDate: 2026-11-19**
- Preserved runnable tests for two- and three-object examples, the 16 unit-pair types, and the large finite classification experiment. **AchievedDate: 2026-11-19**

## 3. Separate sweep merging from control units

**AchievedDate: 2026-11-19**

- Implemented concatenation of consecutive subpaths and a split function restoring an existing intermediate vertex as a boundary. **AchievedDate: 2026-11-19**
- Recent comparison experiments do not apply a separate split operation to change the control plan. **AchievedDate: 2026-11-19**
- Check every object pair's merged sweeps when combining tuples. **AchievedDate: 2026-11-19**
- Retain original control subpath sweeps and interference relations; combine only currently permitted consecutive parts into robot movement commands. **AchievedDate: 2026-11-19**
- Corrected the implementation that delayed an allowed first departure because a later part was blocked. **AchievedDate: 2026-11-19**
- Added regression checks for identical original control plans and identical initial departing objects with and without merging. **AchievedDate: 2026-11-19**

## 4. Clearance-driven control and RoadMap updates

**AchievedDate: 2026-11-19**

- Connected clearance report -> RoadMap update -> reconsider waiting movement -> departure approval. **AchievedDate: 2026-11-19**
- Use conservative checks preserving original control-subpath order and interference precedence. **AchievedDate: 2026-11-19**
- Record monotonically advancing residual-path progress in RoadMap. **AchievedDate: 2026-11-19**
- The Viewer reads residual paths to erase passed path and sweep portions. It does not infer and modify control state itself. **AchievedDate: 2026-11-19**
- Upon final arrival, the movement sweep disappears but the object's point sweep remains. **AchievedDate: 2026-11-19**

## 5. Dynamic interference groups: computation and display

**AchievedDate: 2026-11-19**

- Construct an interference graph by checking every distinct object pair's residual sweeps in RoadMap. **AchievedDate: 2026-11-19**
- Compute connected components as groups and identify singleton components as independent objects. **AchievedDate: 2026-11-19**
- Update at initialization and on clearance/arrival reports without adding a periodic control loop. **AchievedDate: 2026-11-19**
- Display group counts, sizes and members, independent objects, and interference-edge counts. **AchievedDate: 2026-11-19**
- Tested splitting, continued occupancy by arrived objects, custom footprint sizes, and snapshot restoration. **AchievedDate: 2026-11-19**
- Confirmed in real 10- and 20-robot runs that residual edges never appeared anew and all objects eventually became independent. **AchievedDate: 2026-11-19**
- **Current group information is for tracking and display. Independent scheduling/control per group has not yet been applied.** **AchievedDate: 2026-11-19**

## 6. Collision validation, reproduction, and Viewer

**AchievedDate: 2026-11-19**

- Implemented optional AABB collision checking in RMFS. **AchievedDate: 2026-11-19**
- Separated collision-check and collision-stop options. Recent validation runs enabled collision checking and stopping for both MRobot and PSensor. **AchievedDate: 2026-11-19**
- Compared DEVS-only execution with collision validation at a 0.01-second animation interval. **AchievedDate: 2026-11-19**
- Included dynamic controller, TrafficManager, and RoadMap state in snapshots. **AchievedDate: 2026-11-19**
- Restored a snapshot taken after a clearance report triggered another departure and confirmed identical completion time and final state. Reset/replay was also checked. **AchievedDate: 2026-11-19**
- Provide automatic/manual modes, event stepping, reset, Top/Iso views, and speed controls. **AchievedDate: 2026-11-19**
- Passed 66 recent regression checks related to dynamic groups. This does not mean every separate large-scale example was revalidated. **AchievedDate: 2026-11-19**

## 7. Reproducible experimental results

**AchievedDate: 2026-11-19**

Recent 2x2 comparison using identical paths and initial conditions; makespan in seconds:

| Objects | Sweep merging | Departure after tuple completion | Clearance-driven early departure |
|---:|---|---:|---:|
| 10 | OFF | 431.798 | 128.432 |
| 10 | ON | 345.844 | 112.194 |
| 20 | OFF | 729.582 | 211.108 |
| 20 | ON | 590.257 | 164.416 |

- Used a 10x10 point grid, 2m spacing, and fixed seed 20261119. **AchievedDate: 2026-11-19**
- Ran all eight combinations twice with identical completion times. All objects arrived with zero detected collisions at 0.01-second checks and collision stopping enabled. **AchievedDate: 2026-11-19**
- Compared input paths and original control plans for equality and kept pre-correction raw results separately. **AchievedDate: 2026-11-19**
- These are results for the specific examples, not proof of general performance superiority or optimality. **AchievedDate: 2026-11-19**

### Main execution commands

```powershell
python -m examples.RMFS.Tests.10_03_RobotPath_EventRelease
python -m examples.RMFS.Tests.20_03_RobotPath_EventRelease
python -m examples.RMFS.Tests.run_merge_release_audit
python -m examples.RMFS.Tests.run_unit_pair_classification --max-edges 5
```

The latest tables and correction history are reflected in the Korean LaTeX document. Full synchronization of the English LaTeX document is still pending.

See [ToDoListImplementation_en.md](ToDoListImplementation_en.md) for remaining implementation tasks.
