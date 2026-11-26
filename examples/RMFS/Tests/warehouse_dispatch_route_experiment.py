"""Compare one post-return dispatch route from a shared warehouse snapshot."""

from argparse import ArgumentParser
from functools import cache
from itertools import pairwise
from json import dumps, loads
from math import isfinite
from pathlib import Path
from statistics import mean, median

from socs.devs.TESEngin import TESEnginBase

from ..DEVSModelCreator import create_model
from ..MotionEnergyCollector import attach_motion_collector
from ..RobotPathPlanner import RobotPathPlanner, _key
from ..WarehouseLayout import shortest_route
from .crossed_candidate_screening import bundle_metrics, route_timing
from .run_crossing_energy_comparison import DEFAULT_CONFIG
from .run_full_workflow_energy import summarize_run


def alternatives(controller, name, goal, baseline):
    planner = controller.path_planner
    graph = planner.graph
    blocked = [r["reported_position"] for n, r in controller.roadmap.robots.items() if n != name]
    blocked += [p.vertices[-1] for n, p in controller.roadmap.paths.items() if n != name]
    assignment = {"goal_position": goal}
    routes = {baseline: "current"}
    fastest = planner.plan(name, assignment, blocked_positions=blocked)
    routes.setdefault(fastest, "free-flow")
    for a, b in pairwise(fastest):
        forbidden = (planner.by_position[a], planner.by_position[b])
        edges = [edge for edge in graph["edges"]
                 if forbidden not in pairwise(map(_key, edge["path"]))]
        detour = RobotPathPlanner(controller.roadmap, {**graph, "edges": edges})
        try:
            path = detour.plan(name, assignment, blocked_positions=blocked)
        except ValueError:
            continue
        if len(set(path)) == len(path):
            routes.setdefault(path, "detour")
    residual = [controller.roadmap.remaining_subpath(other).coordinates()
                for other in controller.roadmap.paths if other != name]
    residual = [path for path in residual if len(path) > 1 and len(set(path)) == len(path)]
    metrics = {}
    for path in routes:
        pairs = [bundle_metrics({"candidate": path, "remaining": other}) for other in residual]
        metrics[path] = {key: sum(pair[key] for pair in pairs)
                         for key in ("crossings", "reverse_runs", "reverse_length_m")}
        metrics[path]["free_flow_s"] = route_timing(path)[2]
    result = {label: path for path, label in routes.items() if label != "detour"}
    detours = {path for path, label in routes.items() if label == "detour"}
    priorities = (("free_flow_s",), ("crossings", "reverse_length_m"),
                  ("reverse_runs", "reverse_length_m"))
    for i, keys in enumerate(priorities, 1):
        if not detours:
            break
        path = min(detours, key=lambda path: (
            *(metrics[path][key] for key in keys), metrics[path]["free_flow_s"], path))
        result[f"detour-{i}"] = path
        detours.remove(path)
    return result, {label: metrics[path] for label, path in result.items()}


def wait_statistics(controller, since):
    # The strip between the workstation row and the uppermost rack row.
    front = max(y for _, y in controller.rack_positions.values())
    approvals = {name: [] for name in controller.robot_status}
    for event in controller.roadmap.history:
        if event["event"] == "approved":
            approvals[event["robot"]].append(event["time"])
    result = dict(interior_waypoint_to_approval_robot_s=0.,
                  front_waypoint_to_approval_robot_s=0.,
                  rack_arrival_to_station_reservation_robot_s=0.)
    rack_arrivals = {}
    for event in controller.order_history:
        name, time = event.get("robot"), event["time"]
        if event["event"] == "rack_arrival":
            rack_arrivals[name] = time
        elif event["event"] == "station_reserved":
            result["rack_arrival_to_station_reservation_robot_s"] += max(
                0., time - max(since, rack_arrivals.pop(name)))
        elif event["event"] == "waypoint_wait":
            release = next((stamp for stamp in approvals[name] if stamp >= time), None)
            if release is not None:
                region = "front" if event["position"][1] > front else "interior"
                result[f"{region}_waypoint_to_approval_robot_s"] += max(
                    0., release - max(since, time))
    return result


def fleet_progress(intervals, finished, since, until):
    """Integrate translation within the evaluation window, across route changes."""
    per_robot = {}
    for name, motion in intervals.items():
        distance = 0.
        for start, end, origin, profile in motion:
            low = max(start, since)
            high = min(end, until, origin + profile.duration)
            if low < high:
                distance += profile.distance_at(high - origin) - profile.distance_at(low - origin)
        per_robot[name] = distance
    total = sum(per_robot.values())
    return dict(until_s=until, duration_s=until - since, other_robot_metres=total,
                other_robot_metres_per_s=total / (until - since),
                other_rack_returns=sum(since < time <= until for time in finished),
                per_robot_metres=per_robot)


def free_flow_travel(layout):
    @cache
    def travel(start, goal, loaded=False):
        route = shortest_route(layout, start, goal)
        return route_timing(route, loaded=loaded)[2]

    return travel


def predict_completion(model, now, *, travel=None):
    """Estimate the workload tail from current legs, service, and queued orders."""
    controller = model.controller
    roadmap = controller.roadmap
    if travel is None:
        travel = free_flow_travel(roadmap.layout)

    @cache
    def station_for(rack):
        return min(controller.workstations.values(),
                   key=lambda p: travel(rack, p, True) + travel(p, rack, True))

    robots = {robot.name: robot for robot in model.robots}
    service = controller.service_duration
    available = {}
    position = {}
    for name, status in controller.robot_status.items():
        robot = robots[name]
        route = roadmap.remaining_subpath(name).coordinates()
        end = route[-1]
        assignment = controller.assignments.get(name)
        position[name] = end
        if assignment is None:
            available[name] = now + route_timing(route)[2]
            continue
        rack = assignment["rack_position"]
        station = (controller.workstations[assignment["station"]]
                   if "station" in assignment else station_for(rack))
        lift = robot.lift_duration
        lower = robot.lower_duration
        to_station = travel(rack, station, True)
        returning = travel(station, rack, True)
        if status in ("to_rack", "waiting_station"):
            remaining = route_timing(route)[2] + lift + to_station + service + returning + lower
        elif status in ("lifting", "waiting_station_route"):
            lift_left = (max(0., robot.tNext - now) if robot.phase == "LiftRack" else lift)
            remaining = (lift_left if status == "lifting" else 0.) + to_station + service + returning + lower
        elif status == "to_station":
            remaining = route_timing(route, loaded=True)[2] + service + returning + lower
        elif status == "processing":
            deadline = next(time for time, owner in controller.work_events if owner == name)
            remaining = max(0., deadline - now) + returning + lower
        elif status == "waiting_return_route":
            remaining = returning + lower
        elif status == "returning":
            remaining = route_timing(route, loaded=True)[2] + lower
        elif status == "lowering":
            remaining = max(0., robot.tNext - now) if robot.phase == "LowerRack" else lower
        else:
            raise ValueError(f"Unsupported assigned robot state: {status}")
        available[name] = now + remaining
        position[name] = rack

    # Pending orders have no assigned rack. Use a median workload drawn from
    # rack locations already seen by this branch at the evaluation epoch.
    pending = len(controller.pending_orders)
    if pending:
        known_racks = sorted({tuple(a["rack_position"]) for a in
                              (*controller.finished_assignments, *controller.assignments.values())})
        for _ in range(pending):
            name = min(available, key=lambda n: (available[n], n))
            robot = robots[name]
            costs = sorted((travel(position[name], rack)
                            + travel(rack, station_for(rack), True)
                            + travel(station_for(rack), rack, True)
                            + robot.lift_duration + robot.lower_duration + service, rack)
                           for rack in known_racks)
            available[name] += median(cost for cost, _ in costs)
            position[name] = costs[len(costs) // 2][1]

    finishes = {name: available[name] + travel(position[name], path.vertices[0])
                for name, path in controller.initial_paths.items()}
    tail = sorted(finishes, key=lambda name: (-finishes[name], name))
    return dict(epoch_s=now, pending_orders=pending, predicted_finish_s=finishes,
                predicted_last_robot=tail[0], predicted_max_s=finishes[tail[0]],
                predicted_top3_mean_s=mean(finishes[name] for name in tail[:3]))


def run(robot_count, order_count, seed, config):
    model = create_model(robot_count=robot_count, order_count=order_count, seed=seed,
                         full_cycle=True, collision_check=True,
                         rolling_passage="allow", crossing_stops="allow")
    collector = attach_motion_collector(model, model.robots, track_load=True)
    engine = TESEnginBase(model, TimeIntervalAnimation=.02)
    engine.bVerbose = False
    controller = model.controller
    start_leg = controller._start_leg
    chosen = {}
    routes = {}
    replacement = None
    can_discover = False
    applied = False

    def select_route(name, goal, status, *, loaded, route=None):
        nonlocal applied, routes
        if status == "to_rack" and controller.order_history[-1]["event"] == "assigned":
            order = controller.assignments[name]["order"]
            if chosen and (name, order) == (chosen["robot"], chosen["order"]):
                if route != routes["current"] or tuple(goal) != chosen["goal"]:
                    raise RuntimeError("Snapshot replay changed the baseline dispatch.")
                if replacement is not None:
                    route = replacement
                    applied = True
            elif (not chosen and can_discover and len(route) > 1
                  and (name, controller.tNow) in lower_completions):
                candidates, metrics = alternatives(controller, name, goal, route)
                if any(label.startswith("detour") for label in candidates):
                    routes = candidates
                    chosen.update(robot=name, order=order, goal=tuple(goal),
                                  time=controller.tNow, candidate_metrics=metrics)
        return start_leg(name, goal, status, loaded=loaded, route=route)

    controller._start_leg = select_route
    while isfinite(model.tNext) and not chosen:
        lower_completions = {(r.name, r.tNext) for r in model.robots
                             if controller.robot_status[r.name] == "lowering"}
        can_discover = any(time == model.tNext for _, time in lower_completions)
        if can_discover:
            snapshot = engine.capture_snapshot()
        engine.execute_one_step()
    if not chosen:
        raise RuntimeError("No post-return dispatch with a distinct detour was found.")

    results = {}
    traces = {}
    for label, replacement in routes.items():
        engine.restore_snapshot(snapshot)
        applied = False
        prediction = None
        checked = len(controller.order_history)
        while isfinite(model.tNext):
            engine.execute_one_step()
            if prediction is None and len(controller.order_history) != checked:
                if any(event["event"] == "rack_arrival" and event["order"] == chosen["order"]
                       for event in controller.order_history[checked:]):
                    prediction = predict_completion(model, engine.tNow)
                checked = len(controller.order_history)
        if not applied:
            raise RuntimeError("Route was not injected after snapshot restore.")
        try:
            stats = summarize_run(engine, collector, order_count, config)
        except RuntimeError as error:
            results[label] = dict(route=replacement, error=str(error),
                                  completed_orders=controller.completed_orders,
                                  contacts=len(model.collision_history))
            continue
        arrival = next(event["time"] for event in controller.order_history
                       if event["event"] == "rack_arrival" and event["order"] == chosen["order"])
        results[label] = dict(route=replacement, rack_arrival_s=arrival,
                              remaining_makespan_s=stats["makespan"] - chosen["time"],
                              contacts=len(model.collision_history),
                              waits=wait_statistics(controller, chosen["time"]), **stats)
        traces[label] = (
            {r.name: tuple(collector.motion_intervals(r.name))
             for r in model.robots if r.name != chosen["robot"]},
            [assignment["time"] for assignment in controller.finished_assignments
             if assignment["robot"] != chosen["robot"]],
        )
        home = {event["robot"]: event["time"] for event in controller.order_history
                if event["event"] == "home_arrival"}
        actual_tail = sorted(home, key=lambda name: (-home[name], name))
        results[label]["completion_prediction"] = {
            **prediction, "actual_last_robot": actual_tail[0],
            "actual_top3_mean_s": mean(home[name] for name in actual_tail[:3]),
            "actual_finish_s": home,
        }
    if traces:
        common_end = max(results[label]["rack_arrival_s"] for label in traces)
        for label, (intervals, finished) in traces.items():
            row = results[label]
            row["other_fleet_progress"] = {
                "at_target_arrival": fleet_progress(intervals, finished, chosen["time"],
                                                    row["rack_arrival_s"]),
                "at_common_horizon": fleet_progress(intervals, finished, chosen["time"],
                                                    common_end),
            }
    return dict(robot_count=robot_count, order_count=order_count, seed=seed,
                policy=["allow", "allow"], dispatch=chosen, candidates=results)


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, default=8)
    parser.add_argument("--orders", type=int, default=16)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    config = loads(DEFAULT_CONFIG.read_text(encoding="utf-8-sig"))
    report = dumps(run(args.robots, args.orders, args.seed, config), indent=2)
    if args.report:
        args.report.write_text(report + "\n", encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
