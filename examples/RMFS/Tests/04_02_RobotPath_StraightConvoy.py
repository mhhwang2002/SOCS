"""Four robots follow overlapping, three-edge routes on a straight aisle."""

from argparse import ArgumentParser
from importlib import import_module
from math import isfinite

from socs.devs.TESEngin import TESEnginBase

from ..controlled import create_controlled_model


POINTS = tuple((2 * index, 0) for index in range(7))
PATHS = {f'R{index + 1}': POINTS[index:index + 4] for index in range(4)}
GRID = dict(control_points={p: tuple(map(float, p)) for p in POINTS},
            segments=tuple(zip(POINTS, POINTS[1:])), extent=12.)


def create_model(*, push_pull=True, **options):
    return create_controlled_model(
        PATHS, grid=GRID, event_release=True, safe_prefix=True,
        passage_push=push_pull, motion_extension=push_pull, **options)


def run(push_pull):
    model, _, _ = create_model(push_pull=push_pull)
    engine = TESEnginBase(model, TimeIntervalAnimation=.01)
    engine.bVerbose = False
    for _ in range(10000):
        if not isfinite(model.tNext):
            break
        engine.execute_one_step()
    if not model.controller.traffic_manager.done or model.collision_history:
        raise RuntimeError('The convoy did not complete without detected contact.')
    history = model.controller.roadmap.history
    arrivals = [event for event in history if event['event'] == 'arrived']
    stops = sum(tuple(event['position']) != PATHS[event['robot']][-1] for event in arrivals)
    extensions = sum(event['event'] == 'extension_reserved' for event in history)
    pushes = sum(event['event'] == 'push_extension' for event in history)
    print(f"{'Push/Pull' if push_pull else 'Stop-first'}: {engine.tNow:.3f} s, "
          f'{stops} intermediate stops, {pushes} Push and {extensions - pushes} Pull extensions')


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--animate', action='store_true')
    parser.add_argument('--push-pull', choices=('yes', 'no'))
    args, viewer_args = parser.parse_known_args()
    enabled = args.push_pull != 'no'
    if args.animate:
        def toggle_passage():
            nonlocal enabled
            enabled = not enabled

        import_module('examples.RMFS.Tests.01_01_RobotPath_Straight').main(
            name='04_02 Straight Convoy', no_rack=True, robot_paths=PATHS,
            grid=GRID, collision_check=True, argv=viewer_args,
            model_factory=lambda **options: create_model(push_pull=enabled, **options),
            toggle_passage=toggle_passage)
        return
    if viewer_args:
        parser.error('unrecognized arguments: ' + ' '.join(viewer_args))
    for mode in (enabled,) if args.push_pull else (False, True):
        run(mode)


if __name__ == '__main__':
    main()
