"""Replay coupled Push and Pull decisions using RMFS robots and sweep reservations."""

from argparse import ArgumentParser
from dataclasses import replace
from functools import partial
from importlib import import_module
from json import loads
from math import isfinite
from types import MethodType

from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import TESEnginBase

from ..MotionEnergyCollector import attach_motion_collector
from ..RMFSController import RMFSController
from ..motion import Limits
from ..path_geometry import Subpath, TuplePath, sweep_2d
from .blocked_exit_fixture import hold_departure, serial_legs_witness
from .crossing_stop_comparison import create_model
from .run_blocked_exit_push_lookahead import BlockedExitManager
from .run_crossing_energy_comparison import DEFAULT_CONFIG


LOOKAHEAD_ANIMATION_INTERVAL = .1
APPROACH = (-1.55, 0.)
PATHS = {
    'R1': ((-2, 0), APPROACH, (0, 0), (6, 0), (11, 0)),
    'R2': ((-2, -3), (-2, 0), (-2, 8), (8, 8), (8, 2), (8, -15)),
    'R3': ((-2, -5), (-2, -3), (-2, 0), (-2, 5)),
    'R4': ((2, 0), (2, 4)),
    'R5': ((0, -6), (0, -2), (0, 5)),
    'R6': ((8, 0), (8, -2), (11, -2)),
}
GRID = dict(
    control_points={p: tuple(map(float, p)) for route in PATHS.values() for p in route},
    segments=tuple((a, b) for route in PATHS.values() for a, b in zip(route, route[1:])),
    clearance_points={'R1': ((-1.24, 0), (.76, 0), (9.51, 0)),
                      'R2': ((-2, .76), (8, -1.51)), 'R3': ((-2, .76),),
                      'R4': ((2, .76),), 'R5': ((0, .76),), 'R6': ((9.52, -2),)},
    extent=32.,
)


class NestedManager(BlockedExitManager):
    third_plans = None

    def reset(self, time, **options):
        self.pending_third = False
        self.third_choice = None
        self.pull_choice = None
        self.pending_pull = False
        super().reset(time, **options)

    def _can_start(self, index):
        _, name, part = self.tasks[index]
        if (self.third_plans is not None and self.third_choice is None
                and (name, part.coordinates()[0]) in (('R1', (11, 0)), ('R7', (13, -2)))):
            return False
        entrance = {'R1': (6, 0), 'R2': (8, 2)}
        if self.buffer_choice is True and self.pull_choice is None and entrance.get(name) == part.coordinates()[0]:
            return False
        return super()._can_start(index)

    def leaving(self, name, position, time):
        super().leaving(name, position, time)
        if (name == 'R6' and self.buffer_choice is True and self.pull_choice is None
                and position[0] >= 9.51):
            waiting = sorted((event['time'], event['robot']) for event in self.roadmap.history
                             if event['event'] == 'arrived'
                             and (event['robot'], event['position']) in (('R1', (6, 0)), ('R2', (8, 2))))
            self.pending_pull = tuple(robot for _, robot in waiting)
            if set(self.pending_pull) != {'R1', 'R2'}:
                raise RuntimeError('The downstream blocker cleared before both contenders arrived.')

    def passage_request(self, name, stop, time):
        if (self.third_plans is not None and name == 'R1'
                and tuple(stop) == (11, 0) and self.third_choice is None):
            self.pending_third = True
            return
        super().passage_request(name, stop, time)

    def choose_third(self, robot, time):
        self.pending_third = False
        self.third_choice = robot
        if robot == 'R7':
            self.switch_plan(self.third_plans[self.buffer_choice, self.pull_choice])
        self.passage_request('R1', (11, 0), time)

    def choose_pull(self, robot, time):
        self.pending_pull = False
        self.pull_choice = robot
        if robot == 'R2':
            self.switch_plan(self.r2_plan)
        self.reconsider(time)

    def get_snapshot(self):
        return dict(super().get_snapshot(), pending_pull=self.pending_pull, pull_choice=self.pull_choice,
                    pending_third=self.pending_third, third_choice=self.third_choice)


def decision_input(controller, event):
    manager = controller.traffic_manager
    if event is not None and event.port in ('?buffer_choice', '?downstream_choice', '?third_choice'):
        if event.port == '?buffer_choice':
            manager.choose(event.value, controller.tNow)
        elif event.port == '?third_choice':
            manager.choose_third(event.value, controller.tNow)
        else:
            manager.choose_pull(event.value, controller.tNow)
        controller.phase = 'Command' if manager.has_commands else 'Wait'
        return True
    return RMFSController.delta_x(controller, event)


def build(config, *, exit_release=16., downstream_release=37., paths=PATHS, grid=GRID, **options):
    alternatives = {}
    third_plans = {} if "R7" in paths else None

    def plans(solver):
        def at(robot, point):
            return next(i for i, part in enumerate(solver.words[robot], 1)
                        if part.coordinates()[-1] == point)

        goals = list(enumerate(solver.goal))
        rear = (1, at(1, (8, 2)))
        common = [(0, at(0, (0, 0))), rear, goals[2], goals[3],
                  (0, at(0, (6, 0))), goals[4], goals[5]]
        outside_prefix = [(0, at(0, APPROACH)), goals[4], goals[3],
                          (0, at(0, (6, 0))), rear, goals[2], goals[5]]
        sequences = {
            (True, 'R1'): [*common, goals[0], goals[1]],
            (True, 'R2'): [*common, goals[1], goals[0]],
            (False, None): [*outside_prefix, goals[1], goals[0]],
        }
        if third_plans is not None:
            for key, legs in sequences.items():
                prefix = [(0, at(0, (11, 0))) if leg == goals[0] else leg for leg in legs]
                sequences[key] = [*prefix, goals[0], goals[6]]
                alternative = serial_legs_witness(solver, [*prefix, goals[6], goals[0]])
                solver.validate(alternative)
                third_plans[key] = alternative.tuple_path
        inside = serial_legs_witness(solver, sequences[True, 'R1'])
        reverse = serial_legs_witness(solver, sequences[True, 'R2'])
        solver.validate(reverse)
        alternatives['R2'] = reverse.tuple_path
        outside = serial_legs_witness(solver, sequences[False, None])
        solver.validate(outside)

        def coalesce_outside(plan):
            rows = list(plan.stages)
            entry = next(i for i, row in enumerate(rows)
                         if not row[0].is_point and row[0].coordinates()[0] == APPROACH)
            exit_index = next(i for i in range(entry + 1, len(rows))
                              if not rows[i][0].is_point and rows[i][0].coordinates()[-1] == (6, 0))
            first, last = rows[entry][0], rows[exit_index][0]
            rows[entry] = (Subpath(first.path, first.start, last.end), *rows[entry][1:])
            sweeps = tuple(sweep_2d(part, shape, solver.policy)
                           for part, shape in zip(rows[entry], solver.footprints))
            if not solver._safe(sweeps):
                raise ValueError('Unsafe outside passage.')
            return TuplePath(solver.names, tuple(rows[:entry + 1] + rows[exit_index + 1:]))

        alternatives['outside'] = coalesce_outside(outside.tuple_path)
        if third_plans is not None:
            third_plans[False, None] = coalesce_outside(third_plans[False, None])
        return inside

    options.setdefault('lift_up', True)
    model, robots, sensors = create_model(paths, grid, {(0, 0)}, push_pull=True,
                                          allow_crossing_stops=True, plan_builder=plans,
                                          motion_limits=Limits(**config['with_rack']), **options)
    hold_departure(model, robots[3], exit_release)
    hold_departure(model, robots[5], downstream_release)
    controller = model.controller
    original = controller.traffic_manager
    manager = NestedManager(controller.roadmap, original.tuple_path,
                            approach=APPROACH, outside_plan=alternatives['outside'],
                            event_release=True, safe_prefix=True, footprints=original.footprints)
    manager.r2_plan = alternatives['R2']
    manager.third_plans = third_plans
    controller.traffic_manager = manager
    controller.add_input_ports('?buffer_choice', '?downstream_choice', '?third_choice')
    controller.delta_x = MethodType(decision_input, controller)
    return model, robots, sensors


class NestedEngine(TESEnginBase):
    def __init__(self, model, *, depth, **options):
        super().__init__(model, **options)
        self.depth = depth
        self.decisions = []
        self.rollouts = 0
        self.reached_depth = 0

    def rollout_cutoff(self):
        return None

    def _advance(self, depth):
        for _ in range(20000):
            if not isfinite(self.model.tNext):
                manager = self.model.controller.traffic_manager
                return self.tNow if manager.done and not self.model.collision_history else float('inf')
            super().execute_one_step()
            cutoff = self.rollout_cutoff()
            if cutoff is not None:
                return cutoff
            self._decide(depth, record=False)
        raise RuntimeError('Rollout event limit reached.')

    def _decide(self, depth, *, record):
        controller = self.model.controller
        manager = controller.traffic_manager
        if manager.pending_push:
            event, choices = '?buffer_choice', (True, False)
        elif manager.pending_pull:
            event, choices = '?downstream_choice', manager.pending_pull
        elif manager.pending_third:
            event, choices = '?third_choice', ('R1', 'R7')
        else:
            return

        def apply(choice):
            self.execute_devs_x(self.tNow, controller, PortValue(event, choice))

        selected = choices[0]
        scores = {}
        if depth:
            snapshot = self.capture_snapshot()
            original_interval = self.TimeIntervalAnimation
            virtual = snapshot
            if original_interval != LOOKAHEAD_ANIMATION_INTERVAL:
                virtual = replace(snapshot, t_next_animation=snapshot.t_now + LOOKAHEAD_ANIMATION_INTERVAL)
            try:
                self.TimeIntervalAnimation = LOOKAHEAD_ANIMATION_INTERVAL
                for choice in choices:
                    self.restore_snapshot(virtual)
                    apply(choice)
                    self.rollouts += 1
                    self.reached_depth = max(self.reached_depth, self.depth - depth + 1)
                    scores[choice] = self._advance(depth - 1)
            finally:
                self.TimeIntervalAnimation = original_interval
                self.restore_snapshot(snapshot)
            selected = min(scores, key=scores.get)
            if not isfinite(scores[selected]):
                raise RuntimeError(f'No complete branch at {event}: {scores}')
        if record:
            trigger = 'Pull' if event == '?downstream_choice' else 'Push'
            entry = (round(self.tNow, 3), trigger, selected, scores)
            self.decisions.append(entry)
            print(entry)
        apply(selected)

    def execute_one_step(self):
        result = super().execute_one_step()
        self._decide(self.depth, record=True)
        return result


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--animate', action='store_true')
    parser.add_argument('--depth', type=int, choices=range(5))
    parser.add_argument('--exit-release', type=float, default=16.)
    parser.add_argument('--downstream-release', type=float, default=37.)
    args, viewer_args = parser.parse_known_args()
    config = loads(DEFAULT_CONFIG.read_text(encoding='utf-8-sig'))
    factory = partial(build, config, exit_release=args.exit_release, downstream_release=args.downstream_release)
    if args.animate:
        viewer = import_module('.01_01_RobotPath_Straight', __package__)
        viewer.main(name='06_03 Nested RMFS lookahead', robot_paths=PATHS, grid=GRID,
                    collision_check=True, model_factory=factory, argv=viewer_args,
                    engine_factory=partial(NestedEngine, depth=2 if args.depth is None else args.depth))
        return
    if viewer_args:
        parser.error('unrecognized arguments: ' + ' '.join(viewer_args))
    for depth in range(3) if args.depth is None else (args.depth,):
        model, robots, _ = factory()
        collector = attach_motion_collector(model, robots)
        engine = NestedEngine(model, depth=depth, TimeIntervalAnimation=.01)
        engine.bVerbose = False
        while isfinite(model.tNext):
            engine.execute_one_step()
        if model.collision_history or not model.controller.traffic_manager.done:
            raise RuntimeError('Simulation did not complete safely.')
        print(f'Depth {depth}: {max(collector.arrival_time(name, route[-1]) for name, route in PATHS.items()):.3f} s, '
              f'{engine.rollouts} rollouts, reached depth {engine.reached_depth}')


if __name__ == '__main__':
    main()
