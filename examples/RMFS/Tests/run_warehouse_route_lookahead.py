"""Apply a completion-time objective at each post-return warehouse dispatch."""

from argparse import ArgumentParser
from json import dumps, loads
from math import isfinite
from pathlib import Path
from statistics import mean
from time import perf_counter

from socs.devs.TESEngin import TESEnginBase

from ..DEVSModelCreator import create_model
from ..MotionEnergyCollector import attach_motion_collector
from .run_crossing_energy_comparison import DEFAULT_CONFIG
from .run_full_workflow_energy import summarize_run
from .warehouse_dispatch_route_experiment import (
    alternatives, free_flow_travel, predict_completion, wait_statistics,
)


OBJECTIVES = {"max": "predicted_max_s", "top3": "predicted_top3_mean_s"}


def run(objective, seed, *, robots=30, orders=100):
    model = create_model(robot_count=robots, order_count=orders, seed=seed,
                         full_cycle=True, collision_check=True,
                         rolling_passage="allow", crossing_stops="allow")
    collector = attach_motion_collector(model, model.robots, track_load=True)
    sample_interval = .1
    engine = TESEnginBase(model, TimeIntervalAnimation=sample_interval)
    engine.bVerbose = False
    controller = model.controller
    travel = free_flow_travel(controller.roadmap.layout)
    start_leg = controller._start_leg
    decisions = []
    dispatch = None
    discovering = False
    replacement = None
    applied = False

    def select_route(name, goal, status, *, loaded, route=None):
        nonlocal dispatch, applied
        if status == "to_rack" and controller.order_history[-1]["event"] == "assigned":
            order = controller.assignments[name]["order"]
            if discovering and name in lowering and dispatch is None and len(route) > 1:
                routes, _ = alternatives(controller, name, goal, route)
                if len(routes) > 1:
                    dispatch = (name, order, tuple(goal), routes)
            elif replacement is not None and (name, order) == dispatch[:2]:
                if tuple(goal) != dispatch[2] or route != dispatch[3]["current"]:
                    raise RuntimeError("Baseline assignment changed after snapshot restore.")
                route = replacement
                applied = True
        return start_leg(name, goal, status, loaded=loaded, route=route)

    def trial(snapshot, route):
        nonlocal applied, replacement
        engine.restore_snapshot(snapshot)
        replacement, applied = route, False
        checked = len(controller.order_history)
        previous_time = engine.tNow
        repeated_steps = 0
        while isfinite(model.tNext):
            engine.execute_one_step()
            if engine.tNow <= previous_time:
                repeated_steps += 1
                if repeated_steps > 1000:
                    return {"failure": "simulation time did not advance", "time_s": engine.tNow}
            else:
                repeated_steps = 0
                previous_time = engine.tNow
            if model.collision_history:
                return {"failure": "contact"}
            history = controller.order_history
            if any(event["event"] == "rack_arrival" and event["order"] == dispatch[1]
                   for event in history[checked:]):
                if not applied:
                    raise RuntimeError("Candidate route was not applied.")
                prediction = predict_completion(model, engine.tNow, travel=travel)
                return {key: prediction[key] for key in OBJECTIVES.values()}
            checked = len(history)
        return {"failure": "no rack arrival"}

    controller._start_leg = select_route
    started = perf_counter()
    while isfinite(model.tNext):
        lowering = {r.name for r in model.robots
                    if r.phase == "LowerRack" and r.tNext == model.tNext}
        if not lowering or not controller.pending_orders:
            engine.execute_one_step()
            continue
        snapshot = engine.capture_snapshot()
        dispatch = None
        discovering = True
        engine.execute_one_step()
        discovering = False
        if dispatch is None:
            continue
        name, order, goal, routes = dispatch
        if not decisions:
            print(f"{objective}: first decision for {name}, order {order}, "
                  f"at {engine.tNow:.3f}s, {len(routes)} candidates", flush=True)
        scores = {label: trial(snapshot, route) for label, route in routes.items()}
        valid = [label for label, score in scores.items() if "failure" not in score]
        if not valid:
            raise RuntimeError(f"No candidate reached the rack for {name}, order {order}.")
        best = min(valid, key=lambda label: scores[label][OBJECTIVES[objective]])
        engine.restore_snapshot(snapshot)
        replacement, applied = routes[best], False
        engine.execute_one_step()
        if not applied:
            raise RuntimeError("Selected route was not committed.")
        replacement = None
        decisions.append(dict(time_s=engine.tNow, robot=name, order=order, goal=goal,
                              selected=best, scores=scores, routes=routes))
        if len(decisions) <= 3 or len(decisions) % 5 == 0:
            print(f"{objective}: {len(decisions)} decisions, {controller.completed_orders} "
                  f"returns, wall time {perf_counter()-started:.0f}s", flush=True)

    config = loads(DEFAULT_CONFIG.read_text(encoding="utf-8-sig"))
    stats = summarize_run(engine, collector, orders, config)
    finishes = sorted((event["time"] for event in controller.order_history
                       if event["event"] == "home_arrival"), reverse=True)
    return dict(objective=objective, seed=seed, robots=robots, orders=orders,
                collision_sample_interval_s=sample_interval, policy=["allow", "allow"],
                observed_top3_mean_s=mean(finishes[:3]), contacts=len(model.collision_history),
                wall_seconds=perf_counter()-started, decisions=decisions,
                waits=wait_statistics(controller, 0.), **stats)


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("objective", choices=OBJECTIVES)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--robots", type=int, default=30)
    parser.add_argument("--orders", type=int, default=100)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    result = run(args.objective, args.seed, robots=args.robots, orders=args.orders)
    args.report.write_text(dumps(result, indent=2) + "\n", encoding="utf-8")
    print({key: value for key, value in result.items() if key not in ("decisions", "per_robot")})


if __name__ == "__main__":
    main()
