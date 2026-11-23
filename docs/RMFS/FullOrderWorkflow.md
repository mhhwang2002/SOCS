# Shipping, rack return and robot home return

```powershell
python -m examples.RMFS.DEVSModelCreator --robots 10 --orders 100 --seed 0
```

This command runs headlessly with pure DEVS events. No animation samples,
fixed-step release polling or optional sampled collision checking are enabled
by default. For a separate stop-on-collision validation replay:

```powershell
python -m examples.RMFS.DEVSModelCreator --robots 10 --orders 100 --seed 0 --collision-check --collision-interval 0.1
```

The Python factory enables this mode with `full_cycle=True`; its default remains
`False` for callers of the earlier first-leg API. The CLI uses the full workflow
unless `--first-leg-only` is supplied. `--orders` defaults to 100.

## Event flow

```text
Initial order batch at t=0
 -> idle/home robot + FIFO order + reserved random available rack
 -> empty movement to rack
 -> reserve an available workstation before lifting
 -> MRobot lifts rack (1.4 s), reports !lifted
 -> plan loaded route to workstation, then TrafficManager admits movement
 -> workstation service (3.0 s), scheduled by RMFSController
 -> mark shipment complete; plan loaded route to the original rack position
 -> MRobot lowers the same rack (1.4 s), reports !lowered
 -> mark order returned/complete and release its rack reservation
 -> assign a waiting order, or plan an empty route to the robot's own home
```

Lift/lower are physical MRobot phases (`LiftRack`, `LowerRack`). Their durations
are factory/constructor parameters. Workstation service duration is a controller
parameter. These defaults are the earlier example assumptions, not newly
verified hardware specifications. Service completions are held in a deadline
heap; the earliest deadline determines the controller's next DEVS event. There
is no periodic timer loop or additional service DEVS model.

Every leg is planned at its own stage, after the preceding plant report or
service-completion event. Failed temporary route attempts are retried only at
subsequent meaningful events. Loaded/empty costs use their respective compiled
motion profiles; final heading is retained for the next route. Commands retain
intermediate points for PSensor clearance reports.

RobotPathPlanner and TrafficManager remain ordinary components sharing the same
RoadMap inside one RMFSController. TrafficManager appends new legs without
resetting any other robot, outstanding command or reported progress. Its FIFO,
required-order and swept-occupancy checks still apply. The planner conservatively
protects current positions and reserved route endpoints. A new leg's destination
must be clear of other remaining routes. This is an initial conservative policy,
not a global optimality or arbitrary-instance completion guarantee.

Workstations use minimum cumulative assignment count among available eligible
stations, with station ID ties. A reserved station is not freed just because
service is finished: the departing robot's remaining footprint/sweep must clear
it. Rack reservations remain until the physical lower-completion report. Idle
robot choice uses minimum completed shipment count, with robot ID ties; rack
selection is seeded random among eligible unreserved racks. An already moving
homebound robot is not interrupted by another assignment.

Snapshots cover timers, service command buffers, assignments, station and rack
reservations, queues, random state, dynamic routes and traffic tasks. Tests
restore during lift, service and lowering, and check deterministic reset.
The fixed fleet has one persistent MRobot and one PSensor per robot; no per-order
robot copy, LegEvents model or extra controller DEVS model is created.

## Verified seed-0 warehouse workload

### Departure policies

`--departure-policy destination-clear` (default) preserves the conservative
whole-leg mode: route generation waits if the destination intersects another
robot's residual sweep, and traffic approval requires the whole leg to be clear.

`--departure-policy advance` is an experimental safe-prefix mode. It plans the
route without that destination gate, splits it at existing vertices, and uses
TrafficManager to merge the longest currently permitted prefix into one motion
command. A blocked later part does not prevent an allowed earlier part.
Intermediate arrival is not an order-stage completion; only arrival at the leg's
final destination triggers lift, service or lowering. Reconsideration is driven
by plant events, not periodic polling. Exclusive workstation reservations before
lift are still required in both modes.

Safe-prefix progress does not yet guarantee completion of dynamically appended
order legs. Small multi-robot workflows exposed cyclic waits; no general
deadlock-free claim or completed 30-robot benchmark is made for this option.

Example: add `--departure-policy advance` to the animation command below.

### Animation

```powershell
python -m examples.RMFS.DEVSModelCreator --robots 10 --orders 100 --seed 0 --animate --non-interactive --fps 10 --hide-sweeps
```

The shared white-background viewer displays live routes, carried racks and
shipping/return counts. At pickup the stationary rack disappears; after lowering
it reappears at its original position. Collision checking is off by default.
Use A for manual/auto, Space for the next event, U/D for speed, R for reset,
T/I for Top/Iso, H for information, +/- for zoom, arrows for pan and S for sweeps.
P pauses/resumes playback. V switches chassis/racks between wire and surface
geometry; wire is the default (`--model-style surface` starts with surfaces).
Eyes and pupils remain polygon surfaces in both modes. Each robot's stable
color is shared by its chassis, route and carried rack. Collision highlighting
temporarily overrides the chassis color. To validate with collision stopping,
add `--collision-check --stop-on-collision` to the animation command.
Omit `--non-interactive` to start paused before the initial order event.

When traffic blocks translation, a known next heading can be aligned in place
before the region clears. `?orient` uses the existing rotation phase and reports
rotation start/end; it neither approves translation nor creates sensor leaving
events. This is valid for the current fixed-orientation rack envelope. A rotating
footprint model would require a separate rotational-sweep clearance check.

For 30 robots with the same 100 orders:

```powershell
python -m examples.RMFS.DEVSModelCreator --robots 30 --orders 100 --seed 0 --animate --non-interactive --fps 10 --hide-sweeps
```

The original ten bottom homes remain unchanged. Larger fleets also use outer
side bays; the 30-robot layout adds ten side spurs, giving 30 dedicated homes.
Rack and workstation locations are unchanged. Its straight-run motion graph is
cached separately as `precomputed/warehouse30_motion_graph.json`.

Ten robots received 100 orders, shipped and returned all 100 racks, and all ten
returned home at simulation time 7035.124 seconds (rounded).
Per-robot shipments were 10, 9, 11, 10, 10, 10, 10, 10, 10 and 10. A separate
0.1-second stop-on-collision replay also completed all 100 orders and home
returns. This is sampled validation, not a continuous-geometry proof.

The CLI distinguishes last rack-lowering time from all-home time and raises an
error if events end with unfinished jobs. It does not report a partial stopped
run as a completed makespan. No claim of makespan minimization is made.
