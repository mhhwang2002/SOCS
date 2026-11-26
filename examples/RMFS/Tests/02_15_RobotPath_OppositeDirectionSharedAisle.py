"""Two robots traverse a shared aisle in opposite directions."""

from argparse import ArgumentParser
from importlib import import_module
from math import isfinite

from socs.devs.TESEngin import TESEnginBase

from ..controlled import create_controlled_model
from .blocked_exit_fixture import serial_legs_witness


PATHS = {
    'R1': ((0, -4), (0, -2), (0, 0), (12, 0), (12, 4)),
    'R2': ((12, -4), (12, -2), (12, 0), (0, 0), (0, 4)),
}
GRID = dict(
    control_points={point: point for route in PATHS.values() for point in route},
    segments=tuple((a, b) for route in PATHS.values() for a, b in zip(route, route[1:])),
    clearance_points={'R1': ((12, 1.51),), 'R2': ((0, 1.51),)},
    extent=12.,
)


def create_model(**options):
    command_blocks = []

    def plan(solver):
        approaches = tuple(next(i for i, part in enumerate(word, 1)
                                if not part.is_point) for word in solver.words)
        legs = [(0, approaches[0]), (1, approaches[1]), (0, solver.goal[0]), (1, solver.goal[1])]
        counts = [0, 0]
        stage = 0
        for index, target in legs:
            end = stage + target - counts[index]
            command_blocks.append((stage, end))
            counts[index] = target
            stage = end
        return serial_legs_witness(solver, legs)

    model, robots, sensors = create_controlled_model(
        PATHS, grid=GRID, event_release=True, plan_builder=plan,
        motion_extension=False, **options)
    model.controller.traffic_manager.command_blocks = tuple(command_blocks)
    return model, robots, sensors


def run():
    model, _, _ = create_model()
    engine = TESEnginBase(model, TimeIntervalAnimation=.01)
    engine.bVerbose = False
    while isfinite(model.tNext):
        engine.execute_one_step()
    manager = model.controller.traffic_manager
    if not manager.done or model.collision_history:
        raise RuntimeError('The shared-aisle run did not complete without detected contact.')
    history = manager.roadmap.history
    clear = next(event['time'] for event in history
                 if event['robot'] == 'R1' and event['event'] == 'released'
                 and event['position'][1] >= 1.5)
    blue_entry = [event['time'] for event in history
                  if event['robot'] == 'R2' and event['event'] == 'issued'][1]
    if blue_entry < clear:
        raise RuntimeError('R2 was granted passage before R1 cleared the shared aisle.')
    print(f'R1 clear: {clear:.3f} s; R2 passage issued: {blue_entry:.3f} s; '
          f'makespan: {engine.tNow:.3f} s')


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--animate', action='store_true')
    args, viewer_args = parser.parse_known_args()
    if args.animate:
        viewer = import_module('.01_01_RobotPath_Straight', __package__)
        viewer.main(name='02_15 Opposite-direction shared aisle', grid=GRID, robot_paths=PATHS,
                    model_factory=create_model, no_rack=True, collision_check=True, argv=viewer_args)
    elif viewer_args:
        parser.error('unrecognized arguments: ' + ' '.join(viewer_args))
    else:
        run()


if __name__ == '__main__':
    main()
