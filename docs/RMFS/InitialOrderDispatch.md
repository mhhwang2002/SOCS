# Initial order-to-rack dispatch

This optional mode preserves the first-stage-only experiment. The CLI now runs
the full workflow by default; see [FullOrderWorkflow.md](FullOrderWorkflow.md).

```powershell
python -m examples.RMFS.DEVSModelCreator --robots 10 --orders 100 --seed 0 --first-leg-only
```

The factory creates one RMFSController, one OrderGenerator and one persistent
MRobot/PSensor pair per physical robot. The old per-order coupled robot model
and LegEvents model are not used.

```text
OrderGenerator.!orders (one batch at t=0)
    -> RMFSController.?orders / FIFO pending queue
    -> select idle robot and reserve a seeded-random available rack
    -> RobotPathPlanner.plan(robot, assignment)
    -> validate a first-leg completion tuple
    -> TrafficManager installs paths in the shared RoadMap
    -> TrafficManager approves and fills its command buffer
    -> RMFSController emits the command to MRobot
    -> PSensor clearance / MRobot arrival update that same RoadMap
```

RobotPathPlanner and TrafficManager are ordinary objects owned by the single
controller DEVS model. `controller.path_planner.roadmap is
controller.traffic_manager.roadmap` is true. The RoadMap holds static layout
geometry plus per-robot routes and reported progress.

The default factory uses the existing warehouse and its exterior bottom leaf
vertices as initial home positions. A custom layout can supply `robot_starts`
(node IDs) and `rack_positions` (metre coordinates). Initial robots are empty.
Orders follow input order; initially idle robots follow factory order (default
R01, R02, ...); racks are selected by a seeded random permutation, excluding
reserved racks and destinations that cannot be reached under the current
endpoint protection. Failed candidates do not consume an order or reserve a rack.

The first planner is heading-aware A* on the compiled straight-through time
graph. The objective is unimpeded empty-robot travel time including the stored
turn rates; it is not fleet makespan or lookahead. Commands retain intermediate
points. The planner conservatively avoids other robots' initial footprints and
already assigned rack endpoints, enabling the first-leg completion certificate.
This is an intentionally restricted initial policy, not an unrestricted routing
algorithm. TrafficManager retains FIFO/required-order guards; disjoint routes
can depart together and passed regions release through sensor events.

At rack arrival a robot becomes `at_rack`, not idle. Its rack reservation stays
held. No lift, workstation allocation, outbound shipping, rack return, home
return, later order batch or next-order dispatch is implemented yet. The remaining
orders stay pending and `completed_orders` remains zero. Once all first-leg
events finish, the headless DEVS run ends even though pending orders remain.

The default ten-robot seed-0 check received 100 orders at t=0, assigned ten
distinct racks, and delivered ten robots to those racks by simulation time
158.1225535884 s. Ninety orders remained pending. This is a first-leg completion
time, **not** a 100-order makespan. A separate stop-on-collision replay with
0.1-second validation intervals detected no contacts; sampled replay is not a
general continuous collision-freedom proof.

Snapshots include the order queue, reservations, assignments, random-generator
state and installed dynamic paths/tuple. Small-scene tests cover restoration
before order receipt, restoration during movement, and deterministic reset.
