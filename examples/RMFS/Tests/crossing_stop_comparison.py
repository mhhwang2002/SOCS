"""Compare two admission modes and two crossing-stop policies on one layout."""

from argparse import ArgumentParser
from importlib import import_module
from math import isfinite

from socs.devs.TESEngin import TESEnginBase

from ..controlled import create_controlled_model
from .incremental_routing import dag_witness


def crossing_layout(lanes, endpoint):
    axis = (0, 4, *lanes, endpoint)
    crossings = {(x, y) for x in lanes for y in lanes}
    paths = {f'R{i}': tuple((x, y) for x in axis)
             for i, y in enumerate(reversed(lanes), 1)}
    paths.update({f'R{i}': tuple((x, y) for y in axis)
                  for i, x in enumerate(lanes, len(lanes) + 1)})
    grid = dict(
        control_points={p: tuple(map(float, p)) for route in paths.values() for p in route},
        segments=tuple((a, b) for route in paths.values() for a, b in zip(route, route[1:])),
        clearance_points={name: tuple((x + .76, y) if route[0][1] == route[-1][1]
                                      else (x, y + .76) for x, y in route if (x, y) in crossings)
                          for name, route in paths.items()},
        extent=float(endpoint),
    )
    return paths, grid, crossings


def horizontal_first_witness(solver):
    # Parallel lanes are disjoint; crossing tasks share a reference precedence.
    horizontal = {name for name, path in zip(solver.names, solver.paths)
                  if path.vertices[0][1] == path.vertices[-1][1]}
    requires = {name: set() if name in horizontal else horizontal for name in solver.names}
    return dag_witness(solver, requires)


def create_model(paths, grid, crossings, *, push_pull=False, allow_crossing_stops=True, **options):
    paths = paths if allow_crossing_stops else {
        name: tuple(p for p in route if p not in crossings) for name, route in paths.items()}
    grid = grid if allow_crossing_stops else dict(
        grid, control_points={p: xy for p, xy in grid['control_points'].items() if p not in crossings})
    model, robots, sensors = create_controlled_model(
        paths, grid=grid, event_release=True, safe_prefix=True,
        passage_push=push_pull, motion_extension=push_pull, **options)
    model.allow_crossing_stops = allow_crossing_stops
    return model, robots, sensors


def run(paths, crossings, create_model, push_pull, allow_crossing_stops, report=None):
    model, robots, _ = create_model(push_pull=push_pull, allow_crossing_stops=allow_crossing_stops)
    engine = TESEnginBase(model, TimeIntervalAnimation=.01)
    engine.bVerbose = False
    for _ in range(10000):
        if not isfinite(model.tNext):
            break
        engine.execute_one_step()
    else:
        raise RuntimeError('The simulation did not reach quiescence.')
    if not model.controller.traffic_manager.done or model.collision_history:
        raise RuntimeError('The crossing routes did not complete without detected contact.')
    history = model.controller.roadmap.history
    stops = [e for e in history if e['event'] == 'arrived'
             and tuple(e['position']) != paths[e['robot']][-1]]
    crossing_stops = sum(tuple(e['position']) in crossings for e in stops)
    if crossing_stops and not allow_crossing_stops:
        raise RuntimeError('A command ended inside a crossing despite the no-stop policy.')
    extensions = sum(sum(r._variable.motion_extension_counts.values()) for r in robots)
    pushes = sum(e['event'] == 'push_extension' for e in history)
    print(f"{'Push/Pull' if push_pull else 'Stop-first':11} "
          f"{'allow' if allow_crossing_stops else 'forbid':6} {engine.tNow:8.3f} "
          f'{len(stops):5} {crossing_stops:8} {pushes:5} {extensions-pushes:5}')
    if report is not None:
        report(history, allow_crossing_stops)


def main(title, paths, grid, crossings, create_model, *, report=None):
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--animate', action='store_true')
    parser.add_argument('--push-pull', choices=('yes', 'no'))
    parser.add_argument('--crossing-stops', choices=('allow', 'forbid'))
    args, viewer_args = parser.parse_known_args()
    enabled = args.push_pull != 'no'
    allowed = args.crossing_stops != 'forbid'
    if args.animate:
        def toggle_passage():
            nonlocal enabled
            enabled = not enabled

        def toggle_crossing_stops():
            nonlocal allowed
            allowed = not allowed

        import_module('examples.RMFS.Tests.01_01_RobotPath_Straight').main(
            name=title,
            no_rack=True, robot_paths=paths, grid=grid, collision_check=True, argv=viewer_args,
            model_factory=lambda **options: create_model(
                push_pull=enabled, allow_crossing_stops=allowed, **options),
            toggle_passage=toggle_passage, toggle_crossing_stops=toggle_crossing_stops)
        return
    if viewer_args:
        parser.error('unrecognized arguments: ' + ' '.join(viewer_args))
    print('Admission   Stops  Makespan  Stops Crossing  Push  Pull')
    for mode in (enabled,) if args.push_pull else (False, True):
        for policy in (allowed,) if args.crossing_stops else (True, False):
            run(paths, crossings, create_model, mode, policy, report)

