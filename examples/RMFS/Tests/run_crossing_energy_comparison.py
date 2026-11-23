"""Estimate battery energy for the straight blocked-exit fixtures."""

from argparse import ArgumentParser
from importlib import import_module
from json import loads
from math import dist, inf, isclose, isfinite
from pathlib import Path

from socs.devs.TESEngin import TESEnginBase

from ..MotionEnergyCollector import attach_motion_collector
from ..motion import ExtendedProfile, Limits, MovingProfile


DEFAULT_CONFIG = (Path(__file__).resolve().parents[3] / 'docs/RMFS/Papers'
                  / '2026_11_MobileRobotTrafficControl_EnergyAndMakespan/Data/mir600_energy_inputs.json')
CASES = ('03_10_RobotPath_BlockedCrossingExit', '05_02_RobotPath_CrossingBuffer')


def traction_work(profile, mass, rolling_acceleration, end):
    """Integrate positive wheel work over the executed piecewise profile."""
    if isinstance(profile, ExtendedProfile):
        prefix = traction_work(profile.original, mass, rolling_acceleration, min(end, profile.switch_time))
        if end <= profile.switch_time:
            return prefix
        return prefix + traction_work(profile.tail, mass, rolling_acceleration, end - profile.switch_time)

    acceleration_end = (profile.acceleration_time if isinstance(profile, MovingProfile)
                        else profile.peak / profile.acceleration)
    boundaries = (0., acceleration_end, profile.brake_start, profile.duration)
    accelerations = (profile.acceleration, 0., -profile.deceleration)
    return sum(mass * max(0., a + rolling_acceleration)
               * (profile.distance_at(min(right, end)) - profile.distance_at(min(left, end)))
               for left, right, a in zip(boundaries, boundaries[1:], accelerations))


def collect_motion(config, case, loaded, push_pull, allow_stops, interval):
    example = import_module(f'examples.RMFS.Tests.{case}')
    model, robots, _ = example.create_model(
        lift_up=loaded, push_pull=push_pull, allow_crossing_stops=allow_stops,
        collision_check=isfinite(interval),
        motion_limits=Limits(**config['with_rack' if loaded else 'unloaded']))
    collector = attach_motion_collector(model, robots)
    engine = TESEnginBase(model, TimeIntervalAnimation=interval)
    engine.bVerbose = False
    for _ in range(10000):
        if not isfinite(model.tNext):
            break
        engine.execute_one_step()
        if any(robot.phase == 'RotateTo' for robot in robots):
            raise RuntimeError('This energy model only covers the straight crossing fixtures.')
    else:
        raise RuntimeError('Simulation exceeded the event limit.')
    if not model.controller.traffic_manager.done or model.collision_history:
        raise RuntimeError('Simulation failed to complete without detected contact.')
    return collector, example.PATHS


def cumulative_energy(config, mass, intervals, arrival, time):
    """Return battery draw in Wh up to time, capped at final arrival."""
    time = max(0., min(time, arrival))
    rolling = config['rolling_coefficient'] * config['gravity_m_s2']
    wheel_work = sum(
        traction_work(profile, mass, rolling, min(time, end) - origin)
        - traction_work(profile, mass, rolling, start - origin)
        for start, end, origin, profile in intervals if time > start
    )
    return (wheel_work / config['drive_efficiency'] + config['auxiliary_power_w'] * time) / 3600


def evaluate(config, case, loaded, push_pull, allow_stops, interval):
    collector, paths = collect_motion(config, case, loaded, push_pull, allow_stops, interval)
    mass = config['robot_mass_kg'] + config['rack_mass_kg'] * loaded
    arrivals = {name: collector.arrival_time(name, route[-1]) for name, route in paths.items()}
    energy = 0.
    for name, route in paths.items():
        intervals = tuple(collector.motion_intervals(name))
        expected_distance = sum(dist(a, b) for a, b in zip(route, route[1:]))
        distance = sum(p.distance_at(end - origin) - p.distance_at(start - origin)
                       for start, end, origin, p in intervals)
        if not isclose(distance, expected_distance):
            raise RuntimeError(f'{name}: recorded profiles do not cover the assigned route.')
        energy += cumulative_energy(config, mass, intervals, arrivals[name], arrivals[name])
    return max(arrivals.values()), energy


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG,
                        help='JSON file with masses, power assumptions, and load-specific motion limits')
    parser.add_argument('--load', choices=('rack', 'unloaded', 'both'), default='both',
                        help='select which load conditions to evaluate')
    args = parser.parse_args()
    config = loads(args.config.read_text(encoding='utf-8-sig'))
    for field in ('robot_mass_kg', 'rack_mass_kg', 'auxiliary_power_w',
                  'rolling_coefficient', 'gravity_m_s2', 'drive_efficiency'):
        if not isfinite(config[field]) or config[field] < 0:
            parser.error(f'{field} must be finite and nonnegative')
    if config['robot_mass_kg'] == 0 or not 0 < config['drive_efficiency'] <= 1:
        parser.error('robot mass must be positive and drive efficiency must be in (0, 1]')
    for load in ('unloaded', 'with_rack'):
        limits = Limits(**config[load])
        if not all(isfinite(value) and value > 0 for value in vars(limits).values()):
            parser.error(f'{load}: all motion limits must be finite and positive')
    print(f"Energy model: {config['model_id']}")
    print('Example  Load   Admission   Stop policy  Makespan (s)  Fleet Wh')
    load_states = (False, True) if args.load == 'both' else (args.load == 'rack',)
    for case in CASES:
        for loaded in load_states:
            for push_pull in (False, True):
                for allow_stops in (True, False):
                    args = config, case, loaded, push_pull, allow_stops
                    result = evaluate(*args, inf)
                    for interval in (.01, .1):
                        replay = evaluate(*args, interval)
                        if not all(isclose(a, b, rel_tol=1e-10, abs_tol=1e-10)
                                   for a, b in zip(result, replay)):
                            raise RuntimeError(f'{args[1:]}: animation sampling changed the result.')
                    makespan, energy = result
                    print(f"{case[:5]}    {'Rack' if loaded else 'Empty':5}  "
                          f"{'Push/Pull' if push_pull else 'Stop-first':11} "
                          f"{'Allow' if allow_stops else 'Forbid':11}  {makespan:12.3f}  {energy:8.3f}")


if __name__ == '__main__':
    main()
