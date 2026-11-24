"""Test depth-four waiting-contention lookahead on the existing 25-crossing grid."""

from argparse import ArgumentParser
from dataclasses import replace
from importlib import import_module
from math import inf, isfinite
from time import perf_counter

from socs.devs.TESEngin import TESEnginBase

from ..wait_duplication import WaitDuplicationSolver
from .incremental_routing import dag_witness
from .run_lookahead_comparison import make_candidates
from .run_waiting_lookahead import outcome
from .waiting_choice import attach, commit

GRID_CASES = {4: '04_03_RobotPath_FourCrossings', 6: '06_02_RobotPath_NineCrossings',
              10: '10_04_RobotPath_TwentyFiveCrossings'}


def run(depth, robots=10):
    grid_case = import_module(f'.{GRID_CASES[robots]}', __package__)
    # The waiting-choice adapter branches on competing stopped robots.
    # Moving Push/Pull extensions are handled by a different controller.
    model, _, _ = grid_case.create_model(push_pull=False, allow_crossing_stops=True,
                                         collision_check=True, merge_sweeps=True)
    original = model.controller.traffic_manager
    solver = WaitDuplicationSolver(original.roadmap.paths, original.footprints)
    vertical = {name for name, route in grid_case.PATHS.items() if route[0][0] == route[-1][0]}
    reverse = dag_witness(solver, {name: set() if name in vertical else vertical for name in solver.names})
    variants, _ = make_candidates(solver, model.plan_result, budget=12)
    model.waiting_reference_plans = tuple(dict.fromkeys(
        [result.tuple_path for _, result in variants] + [reverse.tuple_path]))
    controller = attach(model, lookahead=True)
    manager = controller.traffic_manager
    engine = TESEnginBase(model, TimeIntervalAnimation=.01)
    engine.bVerbose = False
    decisions = []
    stats = dict(rollouts=0, reached_depth=0, max_decision_seconds=0.)
    started = perf_counter()

    def advance(remaining, level=0):
        steps = 0
        while manager.pending_choice or isfinite(model.tNext):
            if not manager.pending_choice:
                engine.execute_one_step()
                steps += 1
                if steps >= 20000:
                    return outcome(engine, model, steps, 'event_budget')
                continue
            choices = manager.pending_choice
            selected = manager.select_by_rule(choices)
            scores = {}
            evaluated_at = perf_counter()
            if remaining:
                saved = engine.capture_snapshot()
                original_interval = engine.TimeIntervalAnimation
                virtual = saved if original_interval == .1 else replace(saved, t_next_animation=saved.t_now + .1)
                try:
                    engine.TimeIntervalAnimation = .1
                    for choice in choices:
                        engine.restore_snapshot(virtual)
                        commit(engine, controller, choice)
                        stats['rollouts'] += 1
                        stats['reached_depth'] = max(stats['reached_depth'], level + 1)
                        trial = advance(remaining - 1, level + 1)
                        if trial['completed']:
                            scores[choice] = trial['makespan']
                finally:
                    engine.TimeIntervalAnimation = original_interval
                    engine.restore_snapshot(saved)
                if not scores:
                    return outcome(engine, model, steps, 'no_completed_rollout')
                selected = min(choices, key=lambda choice: scores.get(choice, inf))
            if level == 0:
                decisions.append(dict(time=engine.tNow, selected=manager.tasks[selected][1],
                                      scores={manager.tasks[i][1]: value for i, value in scores.items()}))
                stats['max_decision_seconds'] = max(stats['max_decision_seconds'], perf_counter() - evaluated_at)
            commit(engine, controller, selected)
        return outcome(engine, model, steps)

    result = advance(depth)
    return dict(depth=depth, robots=robots, reference_plans=len(model.waiting_reference_plans),
                surviving_reference_plans=len(manager.viable_plans), decisions=decisions, wall_seconds=perf_counter() - started, **stats, **result)


if __name__ == '__main__':
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--depth', type=int, choices=range(5), default=4)
    parser.add_argument('--robots', type=int, choices=GRID_CASES, default=10)
    args = parser.parse_args()
    print(run(args.depth, args.robots))
