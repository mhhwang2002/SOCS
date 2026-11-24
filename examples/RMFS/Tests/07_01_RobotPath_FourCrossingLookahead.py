"""Four-crossing RMFS layout for Push–Pull–Push lookahead."""

from argparse import ArgumentParser
from functools import partial
from importlib import import_module
from json import loads
from math import dist, inf, isfinite
from time import perf_counter

from ..MotionEnergyCollector import attach_motion_collector
from .run_crossing_energy_comparison import DEFAULT_CONFIG, fleet_energy


BASE = import_module('.06_03_RobotPath_NestedRMFSLookahead', __package__)


def layout(r7_goal):
    paths = {**BASE.PATHS, 'R1': (*BASE.PATHS['R1'], (16, 0)),
             'R2': (*BASE.PATHS['R2'][:-1], (8, -20)),
             'R7': ((13, -2), (13, r7_goal))}
    grid = dict(BASE.GRID,
                control_points={p: tuple(map(float, p)) for route in paths.values() for p in route},
                segments=tuple((a, b) for route in paths.values() for a, b in zip(route, route[1:])),
                clearance_points={**BASE.GRID['clearance_points'],
                                  'R1': (*BASE.GRID['clearance_points']['R1'], (14.52, 0)),
                                  'R2': paths['R2'],
                                  'R7': ((13, 1.52),)})
    return paths, grid



def remaining_distance(position, route):
    """Project onto the fixed polyline and return its remaining arc length."""
    tail = 0.
    nearest = (inf, 0.)
    for a, b in reversed(tuple(zip(route, route[1:]))):
        length = dist(a, b)
        fraction = sum((q - x) * (y - x) for q, x, y in zip(position, a, b)) / length**2
        fraction = min(1., max(0., fraction))
        projection = tuple(x + fraction * (y - x) for x, y in zip(a, b))
        nearest = min(nearest, (dist(position, projection), tail + (1. - fraction) * length))
        tail += length
    return nearest[1]


class HorizonEngine(BASE.NestedEngine):
    """Apply the experimental horizon only to the first Push and its nested trials."""

    def __init__(self, model, *, fraction, paths, robots, speed, **options):
        super().__init__(model, **options)
        self.fraction = fraction
        self.paths = paths
        self.robots = robots
        self.speed = speed
        self.stop_with_remaining = None
        self.first_push_seconds = None
        self.first_push_rollouts = 0
        self.cutoff_times = []

    def _decide(self, depth, *, record):
        manager = self.model.controller.traffic_manager
        if not record or not manager.pending_push:
            return super()._decide(depth, record=record)
        if self.fraction is not None:
            initial = remaining_distance(self.robots[0].position_at(self.tNow), self.paths['R1'])
            self.stop_with_remaining = initial * (1. - self.fraction)
        started = perf_counter()
        try:
            return super()._decide(depth, record=record)
        finally:
            self.first_push_seconds = perf_counter() - started
            self.first_push_rollouts = self.rollouts
            self.stop_with_remaining = None

    def rollout_cutoff(self):
        if self.model.collision_history:
            return inf
        if self.stop_with_remaining is None:
            return None
        remaining = [remaining_distance(robot.position_at(self.tNow), self.paths[robot.name])
                     for robot in self.robots]
        if remaining[0] > self.stop_with_remaining + 1e-8:
            return None
        self.cutoff_times.append(self.tNow)
        # Optimistic fleet ETA: elapsed time plus the longest remaining free-flow trip.
        remaining = (remaining_distance(robot.position_at(self.tNow), self.paths[robot.name])
                     for robot in self.robots)
        return self.tNow + max(remaining) / self.speed


def horizon_experiment(factory, paths, config):
    for label, fraction in (('1/3', 1 / 3), ('1/2', .5), ('1', 1.), ('fleet', None)):
        model, robots, _ = factory()
        collector = attach_motion_collector(model, robots)
        engine = HorizonEngine(model, depth=3, fraction=fraction, paths=paths, robots=robots,
                               speed=config['with_rack']['speed'], TimeIntervalAnimation=.01)
        engine.bVerbose = False
        while isfinite(model.tNext):
            engine.execute_one_step()
        if not model.controller.traffic_manager.done or model.collision_history:
            raise RuntimeError(f'Horizon {label} did not complete safely.')
        arrivals, energy = fleet_energy(config, collector, paths, loaded=True)
        print(dict(horizon=label, makespan=max(arrivals.values()), fleet_energy_wh=sum(energy.values()),
                   robot_energy_wh=energy, first_push=engine.decisions[0],
                   first_push_seconds=engine.first_push_seconds,
                   first_push_rollouts=engine.first_push_rollouts,
                   cutoff_times=engine.cutoff_times))

def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--animate', action='store_true')
    parser.add_argument('--compare-horizons', action='store_true')
    parser.add_argument('--depth', type=int, choices=range(5))
    parser.add_argument('--r7-goal', type=float, default=8.)
    args, viewer_args = parser.parse_known_args()
    paths, grid = layout(args.r7_goal)
    config = loads(DEFAULT_CONFIG.read_text(encoding='utf-8-sig'))
    factory = partial(BASE.build, config, paths=paths, grid=grid, exit_release=13.)
    if args.compare_horizons:
        if args.animate or args.depth is not None or viewer_args:
            parser.error('--compare-horizons uses headless depth 3.')
        horizon_experiment(factory, paths, config)
        return
    if args.animate:
        viewer = import_module('.01_01_RobotPath_Straight', __package__)
        viewer.main(name='07_01 Four-crossing lookahead', robot_paths=paths, grid=grid,
                    collision_check=True, model_factory=factory, argv=viewer_args,
                    engine_factory=partial(BASE.NestedEngine, depth=3 if args.depth is None else args.depth))
        return
    if viewer_args:
        parser.error('unrecognized arguments: ' + ' '.join(viewer_args))
    for depth in range(4) if args.depth is None else (args.depth,):
        model, _, _ = factory()
        engine = BASE.NestedEngine(model, depth=depth, TimeIntervalAnimation=.01)
        engine.bVerbose = False
        started = perf_counter()
        while isfinite(model.tNext):
            engine.execute_one_step()
        if model.collision_history or not model.controller.traffic_manager.done:
            raise RuntimeError('Simulation did not complete safely.')
        print(f'Depth {depth}: {engine.tNow:.3f} s, {engine.rollouts} rollouts, reached depth {engine.reached_depth}, ' 
              f'{perf_counter() - started:.3f} s wall time')


if __name__ == '__main__':
    main()
