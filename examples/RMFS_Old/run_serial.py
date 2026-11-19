"""Animate the actual A* serial-admission experiment in the fleet viewer."""

from argparse import ArgumentParser
from collections import Counter
from math import inf, isfinite

from panda3d.core import loadPrcFileData

from .fleet import parking_fleet
from .leg_dispatch import STRATEGIES, experiment
from .motion import EMPTY, LOADED, build_actions
from .run_fleet import FleetViewer, MAX_ANIMATION_SPEED, MIN_SPEED
from .run_panda import Viewer, draw_route
from .traffic_layout import RACK_POSITIONS


class SerialViewer(FleetViewer):
    def __init__(self, trace, makespan, speed, *, start_at=0.0, parallel=False,
                 strategy="A*"):
        self.trace = trace
        self.makespan = makespan
        self.start_at = start_at
        self.mode_title = f"{strategy} PARALLEL" if parallel else f"{strategy} SERIAL ADMISSION"
        self.speed = inf if speed >= MAX_ANIMATION_SPEED else max(MIN_SPEED, speed)
        self.fleet = parking_fleet(10)
        self.show_home_counts = True
        Viewer.__init__(self)

    def reset(self):
        self.time = self.start_at
        self.paused = True
        self.cursor = 0
        self.moves = {}
        self.assigned = set()
        self.carrying = {}
        self.shipments = Counter()
        self.returned = 0
        for rack, node in self.racks.items():
            node.setPos(*RACK_POSITIONS[rack], 0)
        self.advance(0)

    def advance(self, elapsed):
        if elapsed > 0:
            if isfinite(self.speed):
                self.time = min(self.makespan, self.time + min(elapsed, .1) * self.speed)
            else:
                current = [boundary for _, leg, _ in self.moves.values()
                           for boundary in (leg.start, leg.finish) if boundary > self.time]
                if self.cursor < len(self.trace):
                    current.append(self.trace[self.cursor][0])
                self.time = min(current, default=self.makespan)
        while self.cursor < len(self.trace) and self.trace[self.cursor][0] <= self.time:
            _, kind, index, *data = self.trace[self.cursor]
            self.cursor += 1
            if kind == "move":
                stage, leg, heading = data
                limits = LOADED if stage in {"station", "return"} else EMPTY
                self.moves[index] = stage, leg, tuple(build_actions(leg.route, heading, limits))
            elif kind == "assigned":
                self.assigned.add(index)
            elif kind == "lift":
                self.carrying[index] = data[0]
            elif kind == "shipped":
                self.shipments[index] += 1
            elif kind == "returned":
                self.returned += 1
                self.assigned.remove(index)
                rack = self.carrying.pop(index)
                self.racks[rack].setPos(*RACK_POSITIONS[rack], 0)
        lines = []
        for index, robot in enumerate(self.fleet):
            point, heading, status, message = robot.position, -90, "", "Idle"
            remaining = ()
            if index in self.moves:
                stage, leg, actions = self.moves[index]
                point, heading = leg.route[-1], leg.heading
                if self.time < leg.finish:
                    local = self.time - leg.start
                    message = stage.capitalize()
                    if local < 0:
                        point = leg.route[0]
                        if actions:
                            heading = actions[0].heading
                        status, message = "W", "Reserved departure"
                    else:
                        for action in actions:
                            if local < action.duration:
                                point, heading = action.pose(local)
                                if action.turn:
                                    status = "R"
                                remaining = tuple(leg.route)
                                break
                            local -= action.duration
                elif index in self.assigned:
                    status, message = "W", "Waypoint approval"
            elif index in self.assigned:
                status, message = "W", "First departure"
            node = self.robot_nodes[robot]
            node.setPos(*point, 0)
            node.setH(heading)
            marker, label = self.status_labels[robot]
            marker.setPos(*point, 2.25 if index in self.carrying else 1.15)
            label.setText(status)
            label = self.home_labels[robot]
            label.setText(f"{index + 1:02}: {self.shipments[index]}")
            if index in self.carrying:
                self.racks[self.carrying[index]].setPos(*point, .10)
            previous = self.route_lines.get(robot)
            key = (stage, remaining) if len(remaining) > 1 else None
            if previous is not None and previous[0] != key:
                previous[1].removeNode()
                del self.route_lines[robot]
            if key is not None and robot not in self.route_lines:
                phase = {"station": "TravelToWorkstation", "return": "ReturnRack"}.get(
                    stage, "TravelToRack")
                self.route_lines[robot] = (key, draw_route(self.render, phase, remaining))
            lines.append(f"{robot.name}  {status}  {message}")
        speed = f"{self.speed:g}x" if isfinite(self.speed) else "Events only"
        self.monitor.setText(
            f"{self.mode_title}\n\nTime  {self.time:.1f} s  ({speed})\n"
            f"Makespan  {self.makespan:.1f} s\n"
            f"Shipped  {self.shipments.total()} / 100\n"
            f"Returned  {self.returned} / 100\n\n" + "\n".join(lines)
            + "\n\nSpace  Play / Pause\nU / D  Speed    R  Reset")

    def update(self, task):
        if self.time >= self.makespan:
            return task.cont
        return Viewer.update(self, task)


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=20)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--parallel", dest="parallel", action="store_true",
                      help="run waypoint traffic control (default)")
    mode.add_argument("--serial", dest="parallel", action="store_false",
                      help="run the serial admission baseline")
    parser.set_defaults(parallel=True)
    parser.add_argument("--path_planning", "--path-planning", choices=STRATEGIES, default="A*",
                        help="path planning strategy (default: A*)")
    parser.add_argument("--start-at", type=float, default=0.0,
                        help="open paused at the first event at or after this time in seconds")
    args = parser.parse_args()
    if not args.parallel and args.path_planning != "A*":
        parser.error("--serial supports only A*")
    if not isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed must be finite and positive")
    if not isfinite(args.start_at) or args.start_at < 0:
        parser.error("--start-at must be finite and nonnegative")
    trace = []
    makespan, returned = experiment(args.path_planning, serial_admission=not args.parallel,
                                   waypoint_control=args.parallel, trace=trace,
                                   snapshot_at=args.start_at)
    end = makespan if makespan is not None else trace[-1][0]
    start = next((t for t, kind, *_ in trace if kind == "snapshot"), None)
    if start is None:
        parser.error(f"No snapshot event at or after {args.start_at:g} s (run ended at {end:g} s)")
    if makespan is None:
        print(f"Simulation stalled at {end:.3f} s after {returned} returns.")
    mode_label = "Parallel" if args.parallel else "Serial Admission"
    title = f"{args.path_planning} {mode_label}"
    loadPrcFileData("", f"win-size 1320 800\nwindow-title RMFS | {title}")
    SerialViewer(trace, end, args.speed,
                 start_at=start, parallel=args.parallel, strategy=args.path_planning).run()


if __name__ == "__main__":
    main()
