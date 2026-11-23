"""One DEVSph controller containing TrafficManager and the shared RoadMap."""

from math import inf, isfinite, atan2, degrees
from heapq import heappush, heappop
from collections import deque
from copy import deepcopy
from itertools import combinations
from random import Random

from socs.devs import DEVSph
from socs.devs.TESBase import PortValue

from .RoadMap import RoadMap
from .TrafficManager import TrafficManager
from .path_geometry import Path, PathPosition, Subpath, TuplePath, sweep_2d, include_path_points


class RMFSController(DEVSph):
    def __init__(self, paths, tuple_path, *, planning_error=None,
                 event_release=False, footprints=None, command_blocks=None,
                 layout=None, rack_positions=None, motion_graph=None, seed=0,
                 initial_headings=None, full_cycle=False, workstations=None,
                 service_duration=3.0, departure_policy='destination-clear', safe_prefix=False,
                 motion_extension=True, flexible_crossing=False, crossing_policy='fifo'):
        super().__init__("RMFSController")
        self.planning_error = planning_error
        self.initial_paths = dict(paths)
        self.full_cycle = full_cycle
        if departure_policy not in ('destination-clear', 'advance'):
            raise ValueError('Unknown departure policy.')
        if departure_policy == 'advance' and not full_cycle:
            raise ValueError('The advance departure policy currently requires full_cycle=True.')
        self.departure_policy = departure_policy
        self.workstations = dict(workstations or {})
        self.service_duration = service_duration
        if full_cycle and (not self.workstations or not isfinite(service_duration) or service_duration <= 0):
            raise ValueError("Full-cycle orders need workstations and a positive service duration.")
        self.order_mode = rack_positions is not None
        self.rack_positions = dict(rack_positions or {})
        self.seed = seed
        self.initial_headings = dict(initial_headings or {})
        self.roadmap = RoadMap(paths,footprints=footprints,layout=layout)
        self.path_planner = None
        if self.order_mode:
            if not motion_graph or not self.rack_positions:
                raise ValueError("Order mode needs a motion graph and rack positions.")
            if len(set(self.roadmap.footprints.values())) != 1:
                raise ValueError("Initial rack dispatch currently uses one common fleet envelope.")
            from .RobotPathPlanner import RobotPathPlanner
            self.path_planner = RobotPathPlanner(self.roadmap, motion_graph)
            self.add_input_ports("?orders")
            self._reset_orders()
        self.traffic_manager = TrafficManager(self.roadmap, tuple_path,
                                             event_release=event_release,footprints=footprints,
                                             command_blocks=command_blocks, preorient=True,
                                             safe_prefix=safe_prefix,motion_extension=motion_extension,
                                             flexible_crossing=flexible_crossing,
                                             crossing_policy=crossing_policy)
        self.add_phase("Command", tau=0, initial=True)
        self.add_phase("Wait", tau=lambda m: m._wait_delay())
        self.add_internal_transition("Command", "Wait")
        if full_cycle:
            self.add_internal_transition("Wait", "Command", action=RMFSController._work_finished)
        for name in paths:
            self.add_output_ports(f"!{name}", f"!orient_{name}", f"!extend_{name}")
            self.add_input_ports(f"?{name}", f"?arrived_{name}",
                                 f"?rotation_started_{name}", f"?rotation_finished_{name}",
                                 f"?collision_{name}", f"?passage_request_{name}")
            if full_cycle:
                self.add_output_ports(f"!lift_{name}", f"!lower_{name}")
                self.add_input_ports(f"?lifted_{name}", f"?lowered_{name}")

    def set_initial_state(self):
        super().set_initial_state()
        if self.order_mode:
            self._reset_orders()
            self.roadmap.paths = dict(self.initial_paths)
            self.traffic_manager.tuple_path = TuplePath(tuple(self.initial_paths), ())
            self.traffic_manager.command_blocks = None
            self.traffic_manager._build_tasks()
        self.traffic_manager.reset(self.tNow)
        for name, path in self.initial_paths.items():
            heading = self.initial_headings.get(name)
            if heading is None and len(path.vertices) > 1:
                a,b = path.vertices[:2]
                heading = degrees(atan2(b[1]-a[1],b[0]-a[0]))
            if heading is not None:
                self.roadmap.robots[name]['heading_deg'] = heading
        if self.traffic_manager.task_control:
            self.traffic_manager.reconsider(self.tNow)
        if self.planning_error:
            self.traffic_manager.done = False
            self.traffic_manager.halted = True
        self.phase = "Command" if self.traffic_manager.has_commands else "Wait"

    def delta_x(self, event):
        # A report can fill the command buffer. The target phase depends on
        # the post-report buffer, so this hook implements the external graph.
        if event is None:
            return False
        if self.full_cycle:
            return self._cycle_event(event)
        manager = self.traffic_manager
        if self.order_mode and event.port == "?orders":
            self.receive_orders(event.value)
            old_phase = self.phase
            self.phase = "Command" if manager.has_commands else "Wait"
            return self.phase != old_phase or self.phase == "Command"
        for name in self.roadmap.paths:
            if event.port == f"?passage_request_{name}":
                manager.passage_request(name, event.value, self.tNow)
                break
            if event.port == f"?arrived_{name}":
                manager.arrived(name, event.value, self.tNow)
                if self.order_mode:
                    self.robot_status[name] = "at_rack"
                    self.order_history.append(dict(event="rack_arrival", time=self.tNow,
                                                   robot=name, **self.assignments[name]))
                break
            if event.port == f"?{name}":
                manager.leaving(name, event.value, self.tNow)
                break
            if event.port in (f"?rotation_started_{name}", f"?rotation_finished_{name}"):
                kind = "rotation_started" if "started" in event.port else "rotation_finished"
                manager.rotation_report(name, kind, event.value, self.tNow)
                break
            if event.port == f"?collision_{name}":
                manager.halt(name, event.value, self.tNow)
                break
        else:
            raise ValueError(f"Unknown controller report {event.port}.")
        old_phase = self.phase
        self.phase = "Command" if manager.has_commands else "Wait"
        return self.phase != old_phase or self.phase == "Command"

    def _reset_orders(self):
        self.pending_orders = deque()
        self.assignments = {}
        self.rack_reservations = {}
        self.robot_status = {name: "home" if self.full_cycle else "idle" for name in self.initial_paths}
        self.order_history = []
        self.received_orders = 0
        self.completed_orders = 0  # Reaching a rack is NOT completing an order.
        self.random = Random(self.seed)
        if self.full_cycle:
            self.service_commands = deque()
            self.work_events = []
            self.station_reservations = {}
            self.station_counts = dict.fromkeys(self.workstations, 0)
            self.robot_counts = dict.fromkeys(self.initial_paths, 0)
            self.shipped_orders = 0
            self.finished_assignments = []

    def receive_orders(self, orders):
        """Initial batch only; later lifecycle/rolling dispatch is not implemented."""
        orders = tuple(orders)
        if self.received_orders:
            raise ValueError("This first dispatch experiment accepts one initial order batch.")
        if len(set(orders)) != len(orders):
            raise ValueError("Order IDs must be unique.")
        self.pending_orders.extend(orders)
        self.received_orders += len(orders)
        self.order_history.append(dict(event="orders_received", time=self.tNow, count=len(orders)))
        paths = {name: Path(name, (record["reported_position"],))
                 for name, record in self.roadmap.robots.items()}
        selected = []
        # FIFO orders, robot ID tie order, seeded random rack choice.
        for name in self.initial_paths:
            if not self.pending_orders:
                break
            candidates = [rack for rack in sorted(self.rack_positions, key=repr)
                          if rack not in self.rack_reservations]
            self.random.shuffle(candidates)
            blocked = [r["reported_position"] for other, r in self.roadmap.robots.items()
                       if other != name]
            blocked += [a["rack_position"] for a in self.assignments.values()]
            for rack in candidates:
                assignment = dict(order=self.pending_orders[0], rack=rack,
                                  rack_position=tuple(self.rack_positions[rack]))
                try:
                    route = self.path_planner.plan(name, assignment, blocked_positions=blocked)
                except ValueError:
                    continue
                self.pending_orders.popleft()
                self.assignments[name] = assignment
                self.rack_reservations[rack] = name
                self.robot_status[name] = "to_rack" if len(route) > 1 else "at_rack"
                paths[name] = Path(name, route)
                if len(route) > 1:
                    selected.append(name)
                self.order_history.append(dict(event="assigned", time=self.tNow,
                                               robot=name, **assignment))
                break
            else:
                self.order_history.append(dict(event="dispatch_blocked", time=self.tNow, robot=name))
        # A simple, validated completion certificate for just this first leg.
        # Endpoint avoidance permits serialization; event release still admits
        # disjoint runs concurrently and starts others after measured clearance.
        rows, finished = [], set()
        for mover in selected:
            row = tuple(Subpath(path, PathPosition(0), PathPosition(len(path.vertices)-1))
                        if name == mover else path.point(len(path.vertices)-1 if name in finished else 0)
                        for name, path in paths.items())
            sweeps = [sweep_2d(part, self.roadmap.footprints[name], self.roadmap.policy)
                      for name, part in zip(paths, row)]
            if any(a.intersects(b, self.roadmap.policy.tolerance) for a, b in combinations(sweeps, 2)):
                raise ValueError("Initial rack-dispatch certificate contains interfering waits.")
            rows.append(row)
            finished.add(mover)
        self.traffic_manager.install_plan(paths, TuplePath(tuple(paths), tuple(rows)), self.tNow)

    def _wait_delay(self):
        if not self.full_cycle or self.traffic_manager.halted or not self.work_events:
            return inf
        return max(0., self.work_events[0][0]-self.tNow)

    def _cycle_log(self, event, name=None, **values):
        self.order_history.append(dict(event=event, time=self.tNow, robot=name,
                                       **self.assignments.get(name, {}), **values))

    def _target_clear(self, name, position):
        point = Path("destination", (tuple(position),)).point(0)
        target = sweep_2d(point, self.roadmap.footprints[name], self.roadmap.policy)
        return all(not target.intersects(sweep_2d(self.roadmap.remaining_subpath(other),
                       self.roadmap.footprints[other], self.roadmap.policy), self.roadmap.policy.tolerance)
                   for other in self.roadmap.paths if other != name)

    def _plan_leg(self, name, goal, *, loaded):
        if self.departure_policy == 'advance':
            # Geometry first; TrafficManager grants only the safe prefix.
            try:
                return self.path_planner.plan(name, {'goal_position': goal}, loaded=loaded)
            except ValueError:
                return None
        if not self._target_clear(name, goal):
            return None
        blocked = [record["reported_position"] for other, record in self.roadmap.robots.items()
                   if other != name]
        blocked += [path.vertices[-1] for other, path in self.roadmap.paths.items() if other != name]
        try:
            return self.path_planner.plan(name, {"goal_position": goal},
                                          blocked_positions=blocked, loaded=loaded)
        except ValueError:
            return None

    def _start_leg(self, name, goal, status, *, loaded, route=None):
        route = self._plan_leg(name, goal, loaded=loaded) if route is None else route
        if route is None:
            return False
        self.robot_status[name] = status
        self._cycle_log("leg_planned", name, leg=status, path=route, loaded=loaded)
        if len(route) == 1:
            self._leg_arrived(name)
        else:
            self.traffic_manager.append_route(name, Path(name, route), self.tNow,
                                              advance=self.departure_policy == 'advance')
        return True

    def _dispatch_cycle_orders(self):
        idle = sorted((n for n, s in self.robot_status.items() if s in ("idle", "home")),
                      key=lambda n: (self.robot_counts[n], n))
        for name in idle:
            if not self.pending_orders:
                break
            candidates = [r for r in sorted(self.rack_positions, key=repr) if r not in self.rack_reservations]
            self.random.shuffle(candidates)
            for rack in candidates:
                goal = tuple(self.rack_positions[rack])
                route = self._plan_leg(name, goal, loaded=False)
                if route is None:
                    continue
                assignment = dict(order=self.pending_orders.popleft(), rack=rack,
                                  rack_position=goal)
                self.assignments[name] = assignment
                self.rack_reservations[rack] = name
                self._cycle_log("assigned", name)
                self._start_leg(name, goal, "to_rack", loaded=False, route=route)
                break

    def _leg_arrived(self, name):
        status = self.robot_status[name]
        if status == "to_rack":
            self.robot_status[name] = "waiting_station"
            self._cycle_log("rack_arrival", name)
        elif status == "to_station":
            self.robot_status[name] = "processing"
            heappush(self.work_events, (self.tNow+self.service_duration, name))
            self._cycle_log("service_started", name)
        elif status == "returning":
            self.robot_status[name] = "lowering"
            self.service_commands.append(PortValue(f"!lower_{name}", self.assignments[name]["rack"]))
            self._cycle_log("rack_return_arrival", name)
        elif status == "to_home":
            self.robot_status[name] = "home"
            self._cycle_log("home_arrival", name)
        else:
            raise ValueError(f"Unexpected movement completion in {status}.")

    def _release_departed_stations(self):
        for station, owner in list(self.station_reservations.items()):
            if not self.assignments.get(owner, {}).get("shipped"):
                continue
            point = Path("station", (self.workstations[station],)).point(0)
            region = sweep_2d(point, self.roadmap.footprints[owner], self.roadmap.policy)
            remaining = sweep_2d(self.roadmap.remaining_subpath(owner),
                                 self.roadmap.footprints[owner], self.roadmap.policy)
            if not region.intersects(remaining, self.roadmap.policy.tolerance):
                del self.station_reservations[station]
                self._cycle_log("station_released", owner, station_id=station)

    def _advance_cycle(self):
        if self.traffic_manager.halted:
            return
        self._release_departed_stations()
        self._dispatch_cycle_orders()
        for name in self.robot_status:
            status = self.robot_status[name]
            assignment = self.assignments.get(name)
            if status == "waiting_station":
                # Reserve a clear station BEFORE lifting. Plan its route only
                # after the plant confirms lifting has actually finished.
                stations = sorted((s for s in self.workstations if s not in self.station_reservations),
                                  key=lambda s: (self.station_counts[s], s))
                for station in stations:
                    if (self.departure_policy == 'destination-clear' and
                            not self._target_clear(name, self.workstations[station])):
                        continue
                    assignment["station"] = station
                    self.station_reservations[station] = name
                    self.station_counts[station] += 1
                    self.robot_status[name] = "lifting"
                    self.service_commands.append(PortValue(f"!lift_{name}", assignment["rack"]))
                    self._cycle_log("station_reserved", name)
                    break
            elif status == "waiting_station_route":
                self._start_leg(name, self.workstations[assignment["station"]], "to_station", loaded=True)
            elif status == "waiting_return_route":
                self._start_leg(name, assignment["rack_position"], "returning", loaded=True)
            elif status in ("idle", "home") and not self.pending_orders:
                home = self.initial_paths[name].vertices[0]
                if tuple(self.roadmap.robots[name]["reported_position"]) == home:
                    self.robot_status[name] = "home"
                else:
                    self._start_leg(name, home, "to_home", loaded=False)

    def _work_finished(self, event):
        while self.work_events and self.work_events[0][0] <= self.tNow+1e-9:
            _, name = heappop(self.work_events)
            if self.robot_status[name] != "processing":
                raise ValueError("Unexpected station service completion.")
            self.assignments[name]["shipped"] = True
            self.shipped_orders += 1
            self.robot_counts[name] += 1
            self.robot_status[name] = "waiting_return_route"
            self._cycle_log("shipped", name)
        self._advance_cycle()

    def _cycle_event(self, event):
        manager = self.traffic_manager
        if event.port == "?orders":
            orders = tuple(event.value)
            if self.received_orders or len(set(orders)) != len(orders):
                raise ValueError("Expected one initial batch of unique orders.")
            self.received_orders = len(orders)
            self.pending_orders.extend(orders)
            self._cycle_log("orders_received", count=len(orders))
        else:
            for name in self.roadmap.paths:
                if event.port == f"?arrived_{name}":
                    path = self.roadmap.paths[name].vertices
                    approved = self.roadmap.robots[name]['approved_path']
                    command_path = approved.coordinates()
                    if len(command_path) > 1:
                        a, b = command_path[-2:]
                        self.roadmap.robots[name]["heading_deg"] = degrees(atan2(b[1]-a[1], b[0]-a[0]))
                    manager.arrived(name, event.value, self.tNow)
                    if tuple(event.value) == path[-1]:
                        self._leg_arrived(name)
                    else:
                        self._cycle_log('waypoint_wait', name, position=tuple(event.value))
                    break
                if event.port == f"?{name}":
                    manager.leaving(name, event.value, self.tNow)
                    break
                if event.port in (f"?rotation_started_{name}", f"?rotation_finished_{name}"):
                    kind = "rotation_started" if "started" in event.port else "rotation_finished"
                    manager.rotation_report(name, kind, event.value, self.tNow)
                    break
                if event.port == f"?lifted_{name}":
                    if self.robot_status[name] != "lifting" or event.value != self.assignments[name]["rack"]:
                        raise ValueError("Unexpected lift completion.")
                    self.robot_status[name] = "waiting_station_route"
                    self.roadmap.robots[name]["lift_up"] = True
                    self._cycle_log("lifted", name)
                    break
                if event.port == f"?lowered_{name}":
                    if self.robot_status[name] != "lowering" or event.value != self.assignments[name]["rack"]:
                        raise ValueError("Unexpected lower completion.")
                    self._cycle_log("returned", name)
                    self.finished_assignments.append(dict(robot=name, time=self.tNow, **self.assignments[name]))
                    del self.rack_reservations[self.assignments[name]["rack"]]
                    del self.assignments[name]
                    self.completed_orders += 1
                    self.roadmap.robots[name]["lift_up"] = False
                    self.robot_status[name] = "idle"
                    break
                if event.port == f"?collision_{name}":
                    manager.halt(name, event.value, self.tNow)
                    self.service_commands.clear()
                    self.work_events.clear()
                    break
            else:
                raise ValueError(f"Unknown controller report {event.port}.")
        self._advance_cycle()
        self.phase = "Command" if manager.has_commands or self.service_commands else "Wait"
        return True  # Service deadlines may change even if the phase stays Wait.

    def delta_y(self):
        points = (self.roadmap.layout or {}).get('control_points', {}).values()
        outputs = [(PortValue(f"!extend_{command['robot']}", dict(command['extension'],
                        path=include_path_points(command['extension']['path'], points)))
                    if 'extension' in command else
                    PortValue(f"!orient_{command['robot']}", command['orientation'])
                    if 'orientation' in command else
                    PortValue(f"!{command['robot']}", include_path_points(command['path'], points)))
                   for command in self.traffic_manager.emit_commands(self.tNow)]
        if self.full_cycle and self.phase == "Command":
            outputs.extend(self.service_commands)
            self.service_commands.clear()
        super().delta_y()
        return outputs or None

    def get_snapshot(self):
        state = dict(phase=self.phase, manager=self.traffic_manager.get_snapshot())
        if self.order_mode:
            state["orders"] = deepcopy(dict(pending_orders=self.pending_orders,
                assignments=self.assignments, rack_reservations=self.rack_reservations,
                robot_status=self.robot_status, order_history=self.order_history,
                received_orders=self.received_orders, completed_orders=self.completed_orders,
                random_state=self.random.getstate()))
            if self.full_cycle:
                state["cycle"] = deepcopy({name: getattr(self, name) for name in (
                    "service_commands", "work_events", "station_reservations", "station_counts",
                    "robot_counts", "shipped_orders", "finished_assignments")})
        return state

    def set_snapshot(self, state):
        self.phase = state["phase"]
        self.traffic_manager.set_snapshot(state["manager"])
        if self.order_mode:
            orders = deepcopy(state["orders"])
            self.random.setstate(orders.pop("random_state"))
            for name, value in orders.items():
                setattr(self, name, value)
            if self.full_cycle:
                for name, value in deepcopy(state["cycle"]).items():
                    setattr(self, name, value)

    def get_state(self):
        return self.get_snapshot()

    def set_state(self, state):
        self.set_snapshot(state)
