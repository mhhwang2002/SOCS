# Compiled straight-through motion graphs

Compile once and reuse while layout and motion parameters remain the same:

```powershell
python -m examples.RMFS.motion_graph
python -m examples.RMFS.motion_graph --layout l-shape
python -m examples.RMFS.motion_graph --layout warehouse
```

Defaults: 4 by 4 **points**, 2 m spacing. `--points`, `--spacing`, `--output`
and `--force` are available. A matching cache is loaded without recompilation;
layout or profile changes invalidate it. The fingerprint includes directed
adjacency, coordinates, motion parameters and the schema version.

| Layout | Points | Original directed arcs | Added straight-through arcs | Total |
|---|---:|---:|---:|---:|
| L with four points on each arm | 7 | 12 | 12 | 24 |
| 4 by 4 points | 16 | 48 | 48 | 96 |
| Existing warehouse | 401 | 1,054 | 5,910 | 6,964 |

All arcs follow existing horizontal or vertical connections. No diagonal,
corner-cutting, missing-segment or wrong-way shortcuts are created. Each edge
stores `source`, `target`, complete `path`, `intermediate_points`, length,
entry/exit heading and `travel_time_s` for `empty` and `loaded` profiles.
The red curved links in the illustration are a diagram convention, not curved
physical trajectories.

The duration uses the existing `StraightProfile` for one uninterrupted
rest-to-rest straight run. It is **not** the sum of separate stop-to-stop
durations on elementary edges. Turns between edges, initial heading alignment,
traffic waits and changes caused by a forced stop are not included. Yaw rates
and a separate turn-time rule are stored for later planning. The original
motion-parameter assumptions have not been changed or re-certified.

On a straight line with n points, there are n(n-1) directed arcs in total,
2(n-1) original arcs, and (n-1)(n-2) added directed arcs. Counting each physical
pair only once gives (n-1)(n-2)/2 added links: **three when n=4**.

Execution must expand an edge's complete `path` into the MRobot command. Do not
send only its endpoints: the retained intermediate points are needed by the
PSensor clearance schedule. Existing MRobot combines the collinear points into
one acceleration/cruise/braking run. PSensor emits leaving-position reports
after the footprint clears each passed point; MRobot emits final arrival.
For a 6 m horizontal run with 1.5 m footprint, the clearance-position reports
are x=0.75, 2.75 and 4.75 m, followed by arrival at x=6 m. It does not release
the destination footprint while the robot remains there.

This compiler saves planning input only. It has not replaced the active
TrafficManager or installed a new A* route-selection policy.
