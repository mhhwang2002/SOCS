"""One crossing: a delayed third robot temporarily blocks one exit."""

from math import dist

from .blocked_exit_fixture import hold_departure, serial_legs_witness
from . import crossing_stop_comparison as comparison


CROSSING = (0, 0)
PATHS = {'R1': ((-6, 0), (-2, 0), CROSSING, (6, 0)),
         'R2': ((0, -10), (0, -2), (0, 5)),
         'R3': ((2, 0), (2, 4))}
GRID = dict(control_points={p: tuple(map(float, p)) for route in PATHS.values() for p in route},
            segments=tuple((a, b) for route in PATHS.values() for a, b in zip(route, route[1:])),
            clearance_points={'R1': ((.76, 0),), 'R2': ((0, .76),), 'R3': ((2, .76),)},
            extent=15.)
BLOCKER_DEPARTURE = 16.


def exit_blockage_witness(solver, allow_crossing_stops):
    # The policy determines whether the available prefix can end inside the crossing.
    # Complete each alternative with a validated, nonblocking reference schedule.
    r1, r2, r3 = range(3)
    finish = [(robot, solver.goal[robot]) for robot in (r2, r3, r1)]
    if allow_crossing_stops:
        crossing_wait = next(i for i, part in enumerate(solver.words[r1], 1)
                             if part.is_point and part.coordinates()[0] == CROSSING)
        legs = [(r1, crossing_wait), finish[1], finish[2], finish[0]]
    else:
        legs = finish
    return serial_legs_witness(solver, legs)


def create_model(*, allow_crossing_stops=True, **options):
    model, robots, sensors = comparison.create_model(
        PATHS, GRID, {CROSSING}, allow_crossing_stops=allow_crossing_stops,
        plan_builder=lambda solver: exit_blockage_witness(solver, allow_crossing_stops), **options)
    hold_departure(model, robots[2], BLOCKER_DEPARTURE)
    return model, robots, sensors


def report(history, allow_crossing_stops):
    def time(event, robot, position):
        return next((e['time'] for e in history if e['event'] == event and e['robot'] == robot
                     and dist(e['position'], position) < 1e-8), None)

    a_stop = time('arrived', 'R1', CROSSING)
    a_clear = time('released', 'R1', (1.51, 0.))
    b_clear = time('released', 'R2', (0., 1.51))
    if a_clear is None or b_clear is None or (a_clear < b_clear) != allow_crossing_stops:
        raise RuntimeError('The fixture did not reproduce the intended crossing order.')
    if (a_stop is not None) != allow_crossing_stops:
        raise RuntimeError('R1 did not follow the crossing-stop policy.')
    parked = 0. if a_stop is None else next(e['time'] for e in history
        if e['event'] == 'issued' and e['robot'] == 'R1' and e['time'] >= a_stop) - a_stop
    print(f'  Crossing cleared: R1 {a_clear:.3f} s, R2 {b_clear:.3f} s; '
          f'R1 parked on crossing: {parked:.3f} s')


if __name__ == '__main__':
    comparison.main('03_10 Blocked Crossing Exit', PATHS, GRID, {CROSSING}, create_model, report=report)
