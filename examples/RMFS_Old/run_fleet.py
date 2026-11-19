"""Panda3D playback of the 10-robot RMFS reservation baseline."""

from argparse import ArgumentParser
from collections import Counter
from math import inf, isfinite

from panda3d.core import TextNode, loadPrcFileData
from socs.devs.TESEngin import TESEnginBase

from .fleet import dynamic_route, make_fleet_model, mission_pose, parking_fleet, schedule_orders
from .run_panda import Viewer, box, draw_route
from .run_collision import scenario_orders
from .segment_experiment import Move
from .traffic_layout import HORIZONTAL_LANES, INNER_VERTICAL_LANES, PARKING

MIN_SPEED = 0.1
MAX_ANIMATION_SPEED = 100.0
SHIPPED_PHASES = {"ReturnRack", "LowerRack", "ReturnToStandby", "Done"}


def is_turning(mission, phase, time):
    if phase not in mission.motion.routes:
        return False
    elapsed = max(0, time - (mission.robot.tNext - mission.motion.travel_times[phase]))
    for action in mission.motion.actions[phase]:
        if elapsed < action.duration:
            return bool(action.turn)
        elapsed -= action.duration
    return False


class FleetViewer(Viewer):
    def __init__(self, orders, speed, *, fleet=None, check_collisions=False,
                 show_home_counts=True):
        self.check_collisions = check_collisions
        self.show_home_counts = show_home_counts
        self.orders = orders
        self.fleet = fleet if fleet is not None else tuple(dict.fromkeys(order.robot for order in orders))
        self.speed = inf if speed >= MAX_ANIMATION_SPEED else max(MIN_SPEED, speed)
        self.makespan = max(order.finish for order in orders)
        super().__init__()

    def build_scene(self):
        super().build_scene()
        box(self.render, "parking floor", (21, -19, -.15), (47, 5, .25),
            (.21, .29, .32, 1))
        self.robot_nodes = {}
        self.home_labels = {}
        self.status_labels = {}
        for robot in self.fleet:
            node = self.robot.copyTo(self.render)
            node.setPos(*robot.position, 0)
            self.robot_nodes[robot] = node
            status = TextNode(f"{robot.name} motion status")
            status.setAlign(TextNode.ACenter)
            status.setTextColor(1, .92, .42, 1)
            status.setShadow(.06, .06)
            marker = self.render.attachNewNode(status)
            marker.setScale(.65)
            marker.setBillboardPointEye()
            self.status_labels[robot] = (marker, status)
            if self.show_home_counts:
                label = TextNode(f"{robot.name} home shipment count")
                label.setAlign(TextNode.ACenter)
                label.setTextColor(.94, .98, 1, 1)
                label.setShadow(.06, .06)
                marker = self.render.attachNewNode(label)
                x, y = robot.position
                marker.setPos(x, y - .9, .65)
                marker.setScale(.46)
                marker.setBillboardPointEye()
                self.home_labels[robot] = label
        self.robot.hide()

    def reset(self):
        for node in self.robot_nodes.values():
            node.clearColor()
        for _, line in self.route_lines.values():
            line.removeNode()
        self.route_lines.clear()
        self.model = make_fleet_model(self.orders)
        for mission in self.model.children:
            mission.slot = mission.motion.routes["TravelToRack"][-1]
        # Panda3D renders interpolated frames itself. The engine schedules DEVS events only.
        self.engine = TESEnginBase(self.model, TimeIntervalAnimation=inf)
        self.engine.bVerbose = False
        self.time = 0.0
        self.paused = True
        self.shipped_orders = set()
        self.shipped_by_robot = Counter()
        for robot, label in self.home_labels.items():
            label.setText(f"{robot.name.rsplit('-', 1)[-1]}: 0")
        for node in self.racks.values():
            node.setZ(0)
            node.clearColor()
        for mission in self.model.children:
            self.racks[mission.assignment.rack].setPos(*mission.slot, 0)
        self.advance(0)

    def toggle_pause(self):
        super().toggle_pause()
        self.advance(0)

    def next_order(self):
        """The fleet run dispatches its queued orders automatically."""

    def advance(self, elapsed):
        if elapsed > 0:
            if isfinite(self.speed):
                # A stalled frame should slow playback rather than teleport robots.
                self.time = min(self.time + min(elapsed, .1) * self.speed, self.makespan)
            else:
                next_time = self.engine.pick_next_execution_time_type()[0]
                if not isfinite(next_time):
                    return
                self.time = min(next_time, self.makespan)
        while self.engine.pick_next_execution_time_type()[0] <= self.time + 1e-8:
            self.engine.execute_one_step()
        if self.check_collisions:
            self.model.check_collisions(max(self.time, self.engine.tNow))
        active = {}
        pending = {}
        finished = {}
        completed = 0
        for index, (order, mission) in enumerate(zip(self.orders, self.model.children)):
            phase = mission.robot.phase
            if phase in SHIPPED_PHASES and index not in self.shipped_orders:
                self.shipped_orders.add(index)
                self.shipped_by_robot[order.robot] += 1
                label = self.home_labels.get(order.robot)
                if label is not None:
                    label.setText(f"{order.robot.name.rsplit('-', 1)[-1]}: "
                                  f"{self.shipped_by_robot[order.robot]}")
            if phase == "Done":
                completed += 1
                previous = finished.get(order.robot)
                if previous is None or previous.finish < order.finish:
                    finished[order.robot] = order
            elif phase != "Idle":
                active[order.robot] = (mission, phase)
            else:
                pending.setdefault(order.robot, order)
        for mission in self.model.children:
            rack = self.racks[mission.assignment.rack]
            rack.setPos(*mission.slot, 0)
        self.update_routes(active)
        statuses = {}
        for robot, node in self.robot_nodes.items():
            marker, label = self.status_labels[robot]
            label.setText("")
            if robot not in active:
                waiting = pending.get(robot)
                if waiting is None:
                    last = finished.get(robot)
                    point = robot.position if last is None else last.motion.routes["ReturnToStandby"][-1]
                    node.setPos(*point, 0)
                    node.setH(-90 if last is None else last.motion.end_heading)
                    statuses[robot] = "Parked"
                else:
                    motion = waiting.motion
                    point = motion.routes["TravelToRack"][0]
                    node.setPos(*point, 0)
                    node.setH(motion.start_headings["TravelToRack"])
                    marker.setPos(*point, 1.15)
                    label.setText("W")
                    statuses[robot] = (f"W  Dispatch at {waiting.start:.1f} s"
                                       if waiting.start > self.time else "W  Dispatch pending")
                continue
            mission, phase = active[robot]
            if phase == "Collided":
                node.setColor(1, 0, 0, 1, 1)
            point, heading, carrying = mission_pose(mission, self.time)
            node.setPos(*point, 0)
            node.setH(heading)
            statuses[robot] = phase
            if is_turning(mission, phase, self.time):
                marker.setPos(*point, 2.25 if carrying else 1.15)
                label.setText("R")
                statuses[robot] = f"R  {phase}"
            if carrying:
                rack = self.racks[mission.assignment.rack]
                rack.setPos(*point, .10)
                if phase == "Collided":
                    rack.setColor(1, 0, 0, 1, 1)
        lines = [f"{robot.name}  {statuses[robot]}" for robot in self.robot_nodes]
        lines.extend(
            f"{robot.name}: {', '.join(mission.robot.collided_with)}"
            for robot, (mission, phase) in active.items() if phase == "Collided"
        )
        playback = f"{self.speed:g}x" if isfinite(self.speed) else "DEVS events only"
        self.monitor.setText(
            f"RMFS FLEET\n\nTime   {self.time:.1f} s\n"
            f"Playback   {playback}  {'Paused' if self.paused else 'Playing'}\n"
            f"Makespan   {self.makespan:.1f} s\n"
            f"Completed   {completed} / {len(self.orders)}\n"
            f"Shipped   {len(self.shipped_orders)} / {len(self.orders)}\n"
            f"Queued   {len(self.orders) - completed - len(active)}\n\n"
            + "\n".join(lines) + "\n\nGreen  Outbound\nOrange  Rack return\nYellow  Unloaded\n\nU / D  Speed\nSpace  Play / Pause    R  Reset"
        )

    def update_routes(self, active):
        for robot in self.robot_nodes:
            entry = active.get(robot)
            route = ()
            if entry is not None:
                mission, phase = entry
                if phase in mission.motion.routes:
                    elapsed = self.time - mission.robot.tNext + mission.motion.travel_times[phase]
                    route = dynamic_route(mission.motion, phase, elapsed)
            key = (entry, route)
            current = self.route_lines.get(robot)
            if current is not None and current[0] == key:
                continue
            if current is not None:
                current[1].removeNode()
                del self.route_lines[robot]
            if len(route) > 1:
                self.route_lines[robot] = (key, draw_route(self.render, phase, route))

    def update(self, task):
        if self.time >= self.makespan or not isfinite(self.engine.pick_next_execution_time_type()[0]):
            return task.cont
        return super().update(task)


def warehouse_collision_orders():
    x = INNER_VERTICAL_LANES[3]
    y = HORIZONTAL_LANES[3]
    return scenario_orders((
        "Warehouse crossing",
        Move((x - 4, y), (x + 4, y), "horizontal"),
        Move((x, y - 4), (x, y + 4), "vertical"),
        0.0,
    ))


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--robots", type=int, choices=range(1, len(PARKING) + 1), default=10)
    parser.add_argument("--orders", type=int, default=100)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--speed", type=float)
    parser.add_argument("--collision-demo", action="store_true",
                        help="run two conflicting routes in the warehouse layout")
    parser.add_argument("--check-collisions", action="store_true")
    args = parser.parse_args()
    if args.speed is None:
        args.speed = 1 if args.collision_demo else 20
    if not isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed must be finite and positive")
    if args.orders < 1:
        parser.error("--orders must be positive")
    if args.collision_demo:
        orders = warehouse_collision_orders()
        fleet = tuple(order.robot for order in orders)
        title = "RMFS | Warehouse crossing collision"
    else:
        orders = schedule_orders(args.orders, args.seed, robot_count=args.robots, trials=args.trials)
        fleet = parking_fleet(args.robots)
        title = f"RMFS | Robots: {args.robots}"
    loadPrcFileData("", f"win-size 1320 800\nwindow-title {title}")
    FleetViewer(orders, args.speed, fleet=fleet,
                check_collisions=args.check_collisions or args.collision_demo,
                show_home_counts=not args.collision_demo).run()


if __name__ == "__main__":
    main()


