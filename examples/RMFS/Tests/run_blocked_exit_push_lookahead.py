"""Depth-one blocked-exit lookahead, triggered by the real R1 Push event."""

from argparse import ArgumentParser
from importlib import import_module
from json import loads
from math import isfinite
from pathlib import Path
from time import perf_counter
from types import MethodType

from socs.devs.TESEngin import TESEnginBase
from socs.devs.TESBase import PortValue

from ..MotionEnergyCollector import attach_motion_collector
from ..RMFSController import RMFSController
from ..TrafficManager import TrafficManager
from ..motion import Limits
from ..path_geometry import Subpath, TuplePath, sweep_2d
from . import crossing_stop_comparison as comparison
from .blocked_exit_fixture import hold_departure, serial_legs_witness
from .run_crossing_energy_comparison import CASES, DEFAULT_CONFIG, cumulative_energy


class BlockedExitManager(TrafficManager):
    def __init__(self, roadmap, plan, *, approach, outside_plan, **options):
        self.approach = approach
        self.outside_plan = outside_plan
        self.buffer_choice = None
        super().__init__(roadmap, plan, **options)

    def reset(self, time, **options):
        self.pending_push = None
        self.buffer_choice = None
        super().reset(time, **options)

    def passage_request(self, name, stop, time):
        if name == 'R1' and tuple(stop) == self.approach and self.buffer_choice is None:
            self.pending_push = (name, tuple(stop))
            return
        super().passage_request(name, stop, time)

    def _can_start(self, index):
        _, name, part = self.tasks[index]
        if (name == 'R1' and part.coordinates()[0] == self.approach
                and self.buffer_choice is None):
            return False
        return super()._can_start(index)

    def _released_extension(self, name, first):
        _, owner, part = self.tasks[first]
        if (name == owner == 'R1' and self.buffer_choice is True
                and part.coordinates() == (self.approach, (0., 0.))
                and self._can_start(first)):
            approved = self.roadmap.robots[name]['approved_path']
            if approved.end == part.start:
                return Subpath(part.path, approved.start, part.end), (first,)
        return super()._released_extension(name, first)

    def choose(self, use_buffer, time):
        name, stop = self.pending_push
        self.pending_push = None
        self.buffer_choice = use_buffer
        if not use_buffer:
            self.switch_plan(self.outside_plan)
        self.passage_request(name, stop, time)
        if use_buffer and not any('extension' in command and command['robot'] == name
                                  for command in self.command_buffer):
            raise RuntimeError('The Push could not safely extend into the buffer.')

    def switch_plan(self, plan):
        # Preserve committed motion and install only the new precedence
        # for the unissued part of the validated candidate plan.
        previous = self.tasks
        self.tuple_path = plan
        self._build_tasks()
        lookup = {(owner, part): i for i, (_, owner, part) in enumerate(self.tasks)}

        def remap(index):
            _, owner, part = previous[index]
            return lookup[owner, part]

        self.issued_tasks = {remap(i) for i in self.issued_tasks}
        self.finished_tasks = {remap(i) for i in self.finished_tasks}
        self.active_tasks = {owner: remap(i) for owner, i in self.active_tasks.items()}
        self.command_tasks = {owner: tuple(map(remap, indices))
                              for owner, indices in self.command_tasks.items()}
        self.task_wait_since = {lookup[owner, part]: stamp
                                for i, stamp in self.task_wait_since.items()
                                for _, owner, part in (previous[i],)
                                if (owner, part) in lookup}

    def _prefix_parts(self, part):
        pieces = super()._prefix_parts(part)
        if (self.buffer_choice is False and part.path.name == 'R1'
                and part.coordinates()[0] == self.approach):
            return (part,)
        return pieces

    def get_snapshot(self):
        return dict(super().get_snapshot(), pending_push=self.pending_push,
                    buffer_choice=self.buffer_choice)


def decision_input(controller, event):
    if event is not None and event.port == '?buffer_choice':
        controller.traffic_manager.choose(event.value, controller.tNow)
        controller.phase = 'Command' if controller.traffic_manager.has_commands else 'Wait'
        return True
    return RMFSController.delta_x(controller, event)


def build(case, config):
    example = import_module(f'examples.RMFS.Tests.{case}')
    paths = dict(example.PATHS)
    grid = dict(example.GRID)
    two = case == CASES[1]
    approach = (-1.55, 0.) if two else (-2., 0.)
    if two:
        paths['R1'] = (paths['R1'][0], approach, *paths['R1'][1:])
        grid['control_points'] = dict(grid['control_points'])
        grid['control_points'][approach] = approach
    outside = None

    def plans(solver):
        nonlocal outside
        count = next(i for i, part in enumerate(solver.words[0], 1)
                     if part.coordinates()[-1] == approach)
        goals = list(enumerate(solver.goal))
        order = (4, 3, 0, 1, 2) if two else (1, 2, 0)
        outside = serial_legs_witness(solver, [(0, count), *(goals[i] for i in order)])
        # Coalesce the remaining straight R1 moves into one sweep for the outside
        # candidate, validating the combined sweep in the same tuple geometry.
        rows = list(outside.tuple_path.stages)
        entry = next(i for i, row in enumerate(rows)
                     if not row[0].is_point and row[0].coordinates() == (approach, (0., 0.)))
        first = rows[entry][0]
        last = next(row[0] for row in reversed(rows) if not row[0].is_point)
        rows[entry] = (Subpath(first.path, first.start, last.end), *rows[entry][1:])
        rows = rows[:entry + 1] + [row for row in rows[entry + 1:]
                                    if any(not part.is_point for part in row[1:])]
        candidate = TuplePath(outside.tuple_path.object_names, tuple(rows))

        solver.validate(outside)
        sweeps = tuple(sweep_2d(part, shape, solver.policy)
                       for part, shape in zip(rows[entry], solver.footprints))
        if not solver._safe(sweeps):
            raise ValueError('Coalescing the outside candidate created an interfering sweep.')
        outside = candidate
        witness = example.buffer_witness if two else example.exit_blockage_witness
        return witness(solver, True)

    model, robots, _ = comparison.create_model(
        paths, grid, {(0, 0)}, push_pull=True, allow_crossing_stops=True, lift_up=True,
        motion_limits=Limits(**config['with_rack']), plan_builder=plans)
    hold_departure(model, robots[3 if two else 2], example.BLOCKER_DEPARTURE)
    controller = model.controller
    original = controller.traffic_manager
    manager = BlockedExitManager(controller.roadmap, original.tuple_path,
                                 approach=approach, outside_plan=outside, event_release=True,
                                 safe_prefix=True, footprints=original.footprints)


    controller.traffic_manager = manager
    controller.add_input_ports('?buffer_choice')
    controller.delta_x = MethodType(decision_input, controller)
    return model, robots, paths


def run(case, config):
    model, robots, paths = build(case, config)
    collector = attach_motion_collector(model, robots)
    engine = TESEnginBase(model, TimeIntervalAnimation=.01)
    engine.bVerbose = False
    manager = model.controller.traffic_manager

    def advance():
        for _ in range(10000):
            if manager.pending_push or not isfinite(model.tNext):
                return
            engine.execute_one_step()
        raise RuntimeError('Simulation event limit reached.')

    def outcome(use_buffer):
        if not manager.done or model.collision_history:
            raise RuntimeError('Candidate did not complete safely.')
        arrivals = {name: collector.arrival_time(name, path[-1]) for name, path in paths.items()}
        crossing_stop = any(event['event'] == 'arrived' and event['robot'] == 'R1'
                            and event['position'] == (0., 0.) for event in manager.roadmap.history)
        if crossing_stop != use_buffer:
            raise RuntimeError((use_buffer, [e for e in manager.roadmap.history if e['robot']=='R1']))
        mass = config['robot_mass_kg'] + config['rack_mass_kg']
        energy = sum(cumulative_energy(config, mass, tuple(collector.motion_intervals(name)), time, time)
                     for name, time in arrivals.items())
        return max(arrivals.values()), energy

    def apply(choice):
        engine.execute_devs_x(engine.tNow, model.controller, PortValue('?buffer_choice', choice))

    advance()
    if not manager.pending_push:
        raise RuntimeError('R1 never issued the target Push.')
    snapshot = engine.capture_snapshot()
    started = perf_counter()
    results = {}
    try:
        for choice in (True, False):
            engine.restore_snapshot(snapshot)
            apply(choice)
            advance()
            results[choice] = outcome(choice)
    finally:
        engine.restore_snapshot(snapshot)
    selected = min(results, key=results.get)
    elapsed = perf_counter() - started
    apply(selected)
    advance()
    actual = outcome(selected)
    if actual != results[selected]:
        raise RuntimeError('Committed replay differed from the selected rollout.')
    return snapshot.t_now, results, selected, actual, elapsed


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    config = loads(args.config.read_text(encoding='utf-8-sig'))
    for case in CASES:
        print(case, run(case, config))


if __name__ == '__main__':
    main()
