"""Measure fleet energy for the full RMFS order workflow using DEVS events."""
from argparse import ArgumentParser
from bisect import bisect_right
from collections import Counter
from json import loads
from math import isfinite

from ..DEVSModelCreator import add_lookahead_options, create_model, workflow_engine_factory
from ..MotionEnergyCollector import attach_motion_collector
from .run_crossing_energy_comparison import DEFAULT_CONFIG, traction_work


def run(robot_count, order_count, seed, config, *, lookahead_depth=0, lookahead_horizon=1 / 3, **options):
    model = create_model(robot_count=robot_count, order_count=order_count, seed=seed, full_cycle=True, **options)
    collector = attach_motion_collector(model, model.robots, track_load=True)
    engine_factory = workflow_engine_factory(lookahead_depth=lookahead_depth,
                                             lookahead_horizon=lookahead_horizon, **options)
    engine = engine_factory(model)
    engine.bVerbose = False
    controller = model.controller
    reported = 0
    while isfinite(model.tNext):
        engine.execute_one_step()
        completed = controller.completed_orders
        if completed >= reported + 10:
            print(f'Returned {completed}/{order_count} racks at {engine.tNow:.3f} s', flush=True)
            reported = completed
    if (controller.completed_orders != order_count
            or any(status != 'home' for status in controller.robot_status.values())):
        raise RuntimeError('Warehouse run ended with unfinished work.')
    totals = [0., 0.]
    per_robot = {}
    auxiliary = 0.
    rolling = config['rolling_coefficient'] * config['gravity_m_s2']
    for robot in model.robots:
        name = robot.name
        goal = controller.roadmap.paths[name].vertices[-1]
        arrival = collector.arrival_time(name, goal) if controller.robot_counts[name] else 0.
        times, states = zip(*collector.load_events(name))
        if states[-1]:
            raise RuntimeError(f'{name} returned home carrying a rack.')
        work = [0., 0.]
        for start, end, origin, profile in collector.motion_intervals(name):
            end = min(end, origin + profile.duration)
            loaded = states[bisect_right(times, start) - 1]
            if any(start < stamp < end for stamp in times):
                raise RuntimeError(f'{name} changed load while moving.')
            mass = config['robot_mass_kg'] + config['rack_mass_kg'] * loaded
            work[loaded] += (traction_work(profile, mass, rolling, end - origin)
                             - traction_work(profile, mass, rolling, start - origin))
        unloaded_wh, loaded_wh = (joules / config['drive_efficiency'] / 3600 for joules in work)
        auxiliary_wh = config['auxiliary_power_w'] * arrival / 3600
        totals[0] += unloaded_wh
        totals[1] += loaded_wh
        auxiliary += auxiliary_wh
        per_robot[name] = dict(unloaded_wh=unloaded_wh, loaded_wh=loaded_wh, auxiliary_wh=auxiliary_wh)
    if sum(len(collector.load_events(robot.name)) - 1 for robot in model.robots) != 2 * order_count:
        raise RuntimeError('Missing rack lift or lower events.')
    history = controller.roadmap.history
    counts = Counter(event['event'] for event in history)
    crossings = getattr(controller.traffic_manager, 'crossings', ())
    return dict(makespan=engine.tNow, completed_orders=controller.completed_orders,
                push_extensions=counts['push_extension'],
                pull_extensions=counts['extension_reserved'] - counts['push_extension'],
                intermediate_stops=sum(event['event'] == 'waypoint_wait' for event in controller.order_history),
                stops_in_crossing=sum(event['event'] == 'waypoint_wait' and event['position'] in crossings
                                      for event in controller.order_history),
                lookahead_calls=getattr(engine, 'lookahead_calls', 0),
                lookahead_timeouts=getattr(engine, 'lookahead_timeouts', 0),
                lookahead_seconds=getattr(engine, 'lookahead_seconds', 0.),
                deferred_pushes=getattr(engine, 'deferred_pushes', 0),
                last_return=max(a['time'] for a in controller.finished_assignments),
                unloaded_traction_wh=totals[0], loaded_traction_wh=totals[1],
                auxiliary_wh=auxiliary, fleet_wh=sum(totals) + auxiliary, per_robot=per_robot)


if __name__ == '__main__':
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--robots', type=int, default=30)
    parser.add_argument('--orders', type=int, default=100)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--rolling-passage', choices=('allow', 'forbid'))
    parser.add_argument('--crossing-stops', choices=('allow', 'forbid'))
    add_lookahead_options(parser)
    args = parser.parse_args()
    result = run(args.robots, args.orders, args.seed, loads(DEFAULT_CONFIG.read_text(encoding='utf-8-sig')),
                 rolling_passage=args.rolling_passage, crossing_stops=args.crossing_stops,
                 lookahead_depth=args.lookahead_depth, lookahead_horizon=args.lookahead_horizon)
    print(result, flush=True)
