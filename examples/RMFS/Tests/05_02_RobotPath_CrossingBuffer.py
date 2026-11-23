"""Two intersections: a forward buffer can drain the queue at the rear crossing."""

from math import dist

from . import crossing_stop_comparison as comparison
from .blocked_exit_fixture import hold_departure, serial_legs_witness


REAR, FRONT = (-2, 0), (0, 0)
PATHS = {'R1': (REAR, FRONT, (6, 0)),
         'R2': ((-2, -3), REAR, (-2, 7)),
         'R3': ((-2, -5), (-2, -3), REAR, (-2, 5)),
         'R4': ((2, 0), (2, 4)),
         'R5': ((0, -6), (0, -2), FRONT, (0, 5))}
GRID = dict(control_points={p: tuple(map(float, p)) for route in PATHS.values() for p in route},
            segments=tuple((a, b) for route in PATHS.values() for a, b in zip(route, route[1:])),
            clearance_points={'R1': ((-1.24, 0), (.76, 0)), 'R2': ((-2, .76),),
                              'R3': ((-2, .76),), 'R4': ((2, .76),), 'R5': ((0, .76),)},
            extent=13.)
BLOCKER_DEPARTURE = 32.


def buffer_witness(solver, allow_crossing_stops):
    finish = [(i, count) for i, count in enumerate(solver.goal)]
    if allow_crossing_stops:
        front_wait = next(i for i, part in enumerate(solver.words[0], 1)
                          if part.is_point and part.coordinates()[0] == FRONT)
        legs = [(0, front_wait), finish[1], finish[2], finish[3], finish[0], finish[4]]
    else:
        legs = [finish[i] for i in (4, 3, 0, 1, 2)]
    return serial_legs_witness(solver, legs)


def create_model(*, allow_crossing_stops=True, **options):
    model, robots, sensors = comparison.create_model(
        PATHS, GRID, {FRONT}, allow_crossing_stops=allow_crossing_stops,
        plan_builder=lambda solver: buffer_witness(solver, allow_crossing_stops), **options)
    hold_departure(model, robots[3], BLOCKER_DEPARTURE)
    return model, robots, sensors


def report(history, allow_crossing_stops):
    def cleared(robot, position):
        return next(e['time'] for e in history if e['event'] == 'released' and e['robot'] == robot
                    and dist(e['position'], position) < 1e-8)

    rear = [cleared(robot, (-2, 1.51)) for robot in ('R2', 'R3')]
    front = cleared('R5', (0, 1.51))
    parked = next((e['time'] for e in history if e['event'] == 'arrived'
                   and e['robot'] == 'R1' and tuple(e['position']) == FRONT), None)
    if (parked is not None) != allow_crossing_stops:
        raise RuntimeError('The forward buffer was not used as intended.')
    if (any((time < BLOCKER_DEPARTURE) != allow_crossing_stops for time in rear)
            or (front < BLOCKER_DEPARTURE) == allow_crossing_stops):
        raise RuntimeError('The rear queue did not clear in the intended order.')
    print(f'  Rear cleared: R2 {rear[0]:.3f} s, R3 {rear[1]:.3f} s; front cleared: R5 {front:.3f} s')


if __name__ == '__main__':
    comparison.main('05_02 Crossing Buffer', PATHS, GRID, {FRONT}, create_model, report=report)
