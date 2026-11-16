"""Compare yielding policies with five or ten loaded robots at warehouse crossings."""

from argparse import ArgumentParser
from copy import deepcopy
from itertools import combinations
import json
from math import isfinite, sqrt
from pathlib import Path

from socs.devs.DEVS import DEVS
from socs.devs.TESEngin import TESEnginBase
from socs.devs.TESBase import PortValue

from .bidirectional_experiment import STEP, Truck, swept_area_is_clear
from .fleet import footprints_overlap
from .motion import Action, LOADED, Straight, build_actions, final_heading
from .traffic_layout import LANE_SEGMENTS, RACK_POSITIONS, VERTICAL_LANES, shortest_route
from .warehouse_snapshot import B_START, OPPOSING, START, Snapshot, candidates
from .warehouse_three import C_ROUTE, FORK_TICK, frame, permit, remaining

# Adjacent northbound lanes cross the same east-west aisle used by A and B.
CROSSING_LANES = VERTICAL_LANES[4:11]
SNAPSHOT_VERSION = 1
ANIMATION_INTERVAL = 0.01
RACK_HALF_WIDTH = 0.75
CLEARANCE_EPSILON = 1e-8


def save_state(trucks):
    """Encode the state immediately before the first policy decision."""
    return json.dumps({
        "version": SNAPSHOT_VERSION,
        "tick": FORK_TICK,
        "robots": [
            [truck.position, truck.heading, truck.elapsed, [
                [action.start, action.end, action.heading, action.turn, action.duration,
                 action.straight.length if action.straight else None]
                for action in truck.actions
            ]]
            for truck in trucks
        ],
    }, separators=(",", ":"))


def restore_state(snapshot, robot_count):
    state = json.loads(snapshot)
    if (state["version"], state["tick"], len(state["robots"])) != (
        SNAPSHOT_VERSION, FORK_TICK, robot_count
    ):
        raise ValueError("Snapshot version, decision tick or robot count does not match this scenario")
    trucks = []
    for position, heading, elapsed, actions in state["robots"]:
        truck = Truck(tuple(position), heading)
        truck.elapsed = elapsed
        truck.actions = [
            Action(tuple(start), tuple(end), angle, turn, duration,
                   Straight(length, LOADED) if length is not None else None)
            for start, end, angle, turn, duration, length in actions
        ]
        trucks.append(truck)
    return tuple(trucks)


def routes_for(robot_count):
    # Extend each northern exit past the east-west lane at y=15.
    return tuple(
        (*shortest_route((x, -14), (x, 15)), (x, 17))
        for x in CROSSING_LANES[:robot_count - 3]
    )


def evaluate(robot_count, *, check_collisions=False, record_frames=True,
             snapshot_in=None, snapshot_out=None, auto=False, crossing_delay=0.0):
    if not isfinite(crossing_delay) or not 0 <= crossing_delay < 285:
        raise ValueError("Crossing delay must be finite and in [0, 285) seconds.")
    options = tuple(candidates(Snapshot(0)))
    crossing_routes = routes_for(robot_count)
    routes = (OPPOSING, C_ROUTE, *crossing_routes)
    assert all(swept_area_is_clear(route, RACK_POSITIONS.values()) for route in routes)
    goals = (options[0].first[-1], *(route[-1] for route in routes))
    trucks = (Truck(START, 0), Truck(B_START, -90), Truck(C_ROUTE[0], 180),
              *(Truck(route[0], 90) for route in crossing_routes))
    trucks[2].start(C_ROUTE)
    history = []
    fork_time = FORK_TICK * STEP
    fork_sample = round(fork_time / ANIMATION_INTERVAL)
    if record_frames or check_collisions:
        for sample in range(fork_sample):
            time = sample * ANIMATION_INTERVAL
            if record_frames:
                history.append(frame(time / STEP, trucks, goals))
            if check_collisions and racks_overlap(trucks):
                return [(option, "collision", None, history.copy(), 0) for option in options]
            trucks[2].advance(ANIMATION_INTERVAL)
    else:
        trucks[2].advance(fork_time)
    snapshot = save_state(trucks)
    if snapshot_in is not None:
        snapshot = snapshot_in.read_text(encoding="utf-8")
    if snapshot_out is not None:
        snapshot_out.write_text(snapshot, encoding="utf-8")
    model = CrossingTrial(restore_state(snapshot, robot_count), goals, crossing_routes, crossing_delay)
    engine = TESEnginBase(model, TimeIntervalContinuous=float("inf"),
                          TimeIntervalAnimation=float("inf"))
    engine.bVerbose = False
    engine.execute_one_step()
    checkpoint = engine.capture_snapshot()

    def run(option, *, record_frames=False, check_collisions=False):
        engine.restore_snapshot(checkpoint)
        engine.XBuffer.append((model, PortValue("policy", option)))
        engine.execute_one_step()
        frames = history.copy() if record_frames else []
        sample = fork_sample
        while model.status is None:
            if record_frames or check_collisions:
                probes = deepcopy(model.trucks)
                previous = engine.tNow
                while sample * ANIMATION_INTERVAL < model.tNext - 1e-9:
                    time = sample * ANIMATION_INTERVAL
                    for truck in probes:
                        truck.advance(time - previous)
                    if record_frames:
                        frames.append(frame(time / STEP, probes, goals))
                    if check_collisions and racks_overlap(probes):
                        return option, "collision", None, frames, len(model.delayed_entries)
                    sample += 1
                    previous = time
            engine.execute_one_step()
        if record_frames:
            frames.append(frame(engine.tNow / STEP, model.trucks, goals))
        makespan = engine.tNow if model.status == "complete" else None
        return option, model.status, makespan, frames, len(model.delayed_entries)

    if not auto:
        return [
            run(option, record_frames=record_frames, check_collisions=check_collisions)
            for option in options
        ]

    # Cover waiting, both detour directions, and one bay before expanding.
    yields = sorted(options[4:], key=route_duration)
    first = (*options[:3], yields[0])
    results = [run(option) for option in first]
    completed = [row[2] for row in results if row[1] == "complete"]
    best = min(completed, default=float("inf"))
    for option in (options[3], *yields[1:]):
        # Even uninterrupted motion must cover this route and its turns.
        if FORK_TICK * STEP + route_duration(option) <= best + 1e-8:
            row = run(option)
            results.append(row)
            if row[1] == "complete":
                best = min(best, row[2])
    if record_frames or check_collisions:
        feasible = [index for index, row in enumerate(results) if row[1] == "complete"]
        if feasible:
            selected = min(feasible, key=lambda index: results[index][2])
            results[selected] = run(results[selected][0], record_frames=record_frames,
                                    check_collisions=check_collisions)
    return results


def route_duration(option):
    approach = build_actions(option.first, 0, LOADED)
    onward = build_actions(option.onward, final_heading(option.first), LOADED) if option.onward else ()
    return sum(action.duration for actions in (approach, onward) for action in actions)


def racks_overlap(trucks):
    return any(
        footprints_overlap((a.position, a.heading, True), (b.position, b.heading, True))
        for a, b in combinations(trucks, 2)
    )


def time_at_distance(straight, distance):
    speed = straight.peak
    a, b = straight.limits.acceleration, straight.limits.deceleration
    if distance < speed ** 2 / (2 * a):
        return sqrt(2 * distance / a)
    remaining = straight.length - distance
    if remaining < speed ** 2 / (2 * b):
        return straight.duration - sqrt(2 * remaining / b)
    return distance / speed + speed / (2 * a)


class CrossingTrial(DEVS):
    """An event-driven crossing simulation whose dynamic state can be branched by SOCS."""

    def __init__(self, trucks, goals, routes, crossing_delay):
        super().__init__("CrossingTrial")
        self.add_input_ports("policy")
        self.add_output_ports("segment_released")
        self.initial_trucks = trucks
        self.goals = goals
        self.routes = routes
        self.release_times = tuple(FORK_TICK * STEP + crossing_delay + index
                                   for index in range(len(routes)))

    def set_initial_state(self):
        self.trucks = deepcopy(self.initial_trucks)
        self.motion_time = FORK_TICK * STEP
        self.next_event = self.motion_time
        self.segment_releases = ()
        self.approach = self.onward = ()
        self.opposing = OPPOSING
        self.yielded = False
        self.delayed_entries = set()
        self.crossing_remaining = list(self.routes)
        self.status = None

    def get_state(self):
        return (self.trucks, self.motion_time, self.next_event, self.segment_releases,
                self.approach, self.onward, self.opposing,
                self.yielded, self.delayed_entries, self.crossing_remaining, self.status)

    def set_state(self, state):
        (self.trucks, self.motion_time, self.next_event, self.segment_releases,
                self.approach, self.onward, self.opposing,
         self.yielded, self.delayed_entries, self.crossing_remaining, self.status) = state

    def tau(self):
        return self.next_event - self.tNow

    def delta_x(self, event):
        if event.port == "policy":
            self.approach, self.onward = event.value.first, event.value.onward
            self.next_event = self.tNow
            return True
        return False

    def schedule_next(self):
        now = self.tNow
        times = [300.0]
        releases = []
        pending = (self.approach, self.onward, self.opposing, *self.crossing_remaining)
        # A moving end of a reserved sweep clears an edge once it passes
        # that edge's endpoint plus the two rack half-widths.
        boundaries = [
            {point[axis] + sign * (2 * RACK_HALF_WIDTH + CLEARANCE_EPSILON)
             for route in pending if len(route) > 1
             for point in route for sign in (-1, 1)}
            for axis in (0, 1)
        ]
        for index, truck in enumerate(self.trucks):
            if not truck.actions:
                continue
            action = truck.actions[0]
            times.append(now + action.duration - truck.elapsed)
            if action.straight is None:
                continue
            axis = 0 if action.start[0] != action.end[0] else 1
            start, end = action.start[axis], action.end[axis]
            cross_axis = 1 - axis
            direction = 1 if end > start else -1
            for segment in LANE_SEGMENTS:
                low, high = sorted((segment.start[cross_axis], segment.end[cross_axis]))
                if not low - RACK_HALF_WIDTH <= action.start[cross_axis] <= high + RACK_HALF_WIDTH:
                    continue
                edge = max(segment.start[axis] * direction, segment.end[axis] * direction)
                distance = edge + RACK_HALF_WIDTH + CLEARANCE_EPSILON - start * direction
                if 0 < distance < action.straight.length:
                    delay = time_at_distance(action.straight, distance) - truck.elapsed
                    if delay > 1e-9:
                        time = now + delay
                        times.append(time)
                        releases.append((time, index, segment.start, segment.end))
            for boundary in boundaries[axis]:
                fraction = (boundary - start) / (end - start)
                if 0 < fraction < 1:
                    delay = time_at_distance(action.straight, fraction * action.straight.length)
                    delay -= truck.elapsed
                    if delay > 1e-9:
                        times.append(now + delay)
        times.extend(
            release for release, route in zip(self.release_times, self.crossing_remaining)
            if len(route) > 1 and release > now
        )
        self.next_event = min(times)
        self.segment_releases = tuple(
            (index, start, end) for time, index, start, end in releases
            if abs(time - self.next_event) < 1e-9
        )

    def delta_y(self):
        if not self.approach:
            self.next_event = float("inf")
            return
        outputs = [
            PortValue("segment_released", (index, start, end))
            for index, start, end in self.segment_releases
        ]
        trucks = self.trucks
        for truck in trucks:
            truck.advance(self.tNow - self.motion_time)
        self.motion_time = self.tNow
        a, b, c, *crossers = trucks
        approach, onward, opposing = self.approach, self.onward, self.opposing
        yielded = self.yielded
        crossing_remaining = self.crossing_remaining
        for index, (truck, route) in enumerate(zip(crossers, crossing_remaining)):
            if len(route) == 1 or self.tNow < self.release_times[index]:
                continue
            obstacles = tuple(remaining(other) for other in trucks if other is not truck)
            if onward and len(opposing) > 1:
                obstacles += (opposing,)
            if onward and not yielded:
                # Preserve access to the selected bay while A approaches it.
                obstacles += (approach,)
            crossing_remaining[index] = permit(truck, route, *obstacles)
            if not truck.actions:
                self.delayed_entries.add(index)
        traffic = tuple(remaining(truck) for truck in (c, *crossers))
        if onward and not yielded:
            approach = permit(a, approach, *traffic, remaining(b))
            yielded = len(approach) == 1 and not a.actions
        if yielded or not onward:
            opposing = permit(b, opposing, *traffic, remaining(a))
            b_path = remaining(b) if len(opposing) == 1 else OPPOSING
            if yielded:
                onward = permit(a, onward, *traffic, b_path)
            else:
                approach = permit(a, approach, *traffic, b_path)
        self.approach, self.onward, self.opposing = approach, onward, opposing
        self.yielded = yielded
        if all(truck.position == goal and not truck.actions
                 for truck, goal in zip(trucks, self.goals)):
            self.status = "complete"
        elif self.tNow >= self.release_times[-1] and not any(
            truck.actions for truck in trucks
        ):
            self.status = "stalled"
        elif self.tNow >= 300.0:
            self.status = "timeout"

        if self.status is None:
            self.schedule_next()
        else:
            self.next_event = float("inf")
            self.segment_releases = ()
        return outputs or None


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, choices=(5, 10), required=True)
    parser.add_argument("--check-collisions", action="store_true",
                        help="check robot footprints at each simulation sample")
    snapshots = parser.add_mutually_exclusive_group()
    snapshots.add_argument("--save-snapshot", type=Path)
    snapshots.add_argument("--load-snapshot", type=Path)
    parser.add_argument("--auto", action="store_true", help="select a policy by snapshot lookahead")
    parser.add_argument("--crossing-delay", type=float, default=0.0,
                        help="delay all crossing robot entries by this many seconds")
    args = parser.parse_args()
    results = evaluate(
        args.robots, check_collisions=args.check_collisions, record_frames=False,
        snapshot_in=args.load_snapshot, snapshot_out=args.save_snapshot, auto=args.auto,
        crossing_delay=args.crossing_delay,
    )
    for option, status, makespan, _, delayed in results:
        duration = "-" if makespan is None else f"{makespan:.2f} s"
        print(f"{option.name:20} {status:9} {duration:10} delayed entries: {delayed}")

    if args.auto:
        feasible = [row for row in results if row[1] == "complete"]
        selected = min(feasible, key=lambda row: row[2], default=None)
        verified_collision = next((row for row in results if row[1] == "collision"), None)
        if verified_collision is not None:
            print("Collision verification failed.")
            return
        print(f"Selected: {selected[0].name if selected else 'no feasible policy'}"
              f" ({len(results)} branches)")


if __name__ == "__main__":
    main()
