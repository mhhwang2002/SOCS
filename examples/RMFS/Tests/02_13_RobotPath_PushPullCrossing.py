"""Compare FIFO and LP under stop-first and Push/Pull at a two-robot crossing."""

from argparse import ArgumentParser
from importlib import import_module
from math import isfinite

from socs.devs.TESEngin import TESEnginBase

from ..controlled import create_controlled_model


# Distances to and from the crossing, in metres, for R and B respectively.
LAYOUTS = {
    'no-approach-braking': ((4.5, 7), (13.2, 8)),
    'farther-longer': ((10, 3), (17, 14)),
    'equal-exits': ((10, 3), (17, 3)),
    'nearer-longer': ((10, 15), (17, 14)),
    'early-follower': ((10, 3), (12, 14)),
}
MODES = {'stop-first': False, 'push-pull': True}
POLICIES = ('fifo', 'lp')
DEFAULT_ANIMATION_LAYOUT = 'no-approach-braking'


def crossing_geometry(layout):
    (r_start, r_goal), (b_start, b_goal) = LAYOUTS[layout]
    paths = {'R': ((-r_start, 0), (-2, 0), (r_goal, 0)),
             'B': ((0, -b_start), (0, -2), (0, b_goal))}
    grid = dict(control_points={p: tuple(map(float, p)) for route in paths.values() for p in route},
                segments=tuple((a, b) for route in paths.values() for a, b in zip(route, route[1:])),
                extent=max(r_start + r_goal, b_start + b_goal))
    if layout == DEFAULT_ANIMATION_LAYOUT:
        # With 1.5 m footprints, the tail clears this sensor when the centre
        # is 1.51 m past the crossing. These are sensing points, not stops.
        grid['clearance_points'] = {'R': ((.76, 0.),), 'B': ((0., .76),)}
    return paths, grid


def build_model(paths, grid, mode, policy, **options):
    push_pull = MODES[mode]
    return create_controlled_model(
        paths, grid=grid, event_release=True, safe_prefix=True,
        passage_push=push_pull, motion_extension=push_pull, flexible_crossing=True,
        crossing_policy=policy, **options)


def run(mode, policy, layout):
    push_pull = MODES[mode]
    paths, grid = crossing_geometry(layout)
    model, robots, _ = build_model(paths, grid, mode, policy)
    engine = TESEnginBase(model, TimeIntervalAnimation=.05)
    engine.bVerbose = False
    for _ in range(5000):
        if not isfinite(model.tNext):
            break
        engine.execute_one_step()
    else:
        raise RuntimeError(f'{mode} did not finish within the event limit')
    if not model.controller.traffic_manager.done or model.collision_history:
        raise RuntimeError(f'{mode} did not complete both routes without collision')
    arrivals = [event for event in model.controller.roadmap.history if event['event'] == 'arrived']
    stops = [event['robot'] for event in arrivals
             if tuple(event['position']) == paths[event['robot']][1]]
    extensions = [robot.name for robot in robots if sum(robot._variable.motion_extension_counts.values())]
    grants = [event for event in model.controller.roadmap.history
              if event['event'] == 'crossing_granted']
    (_, r_goal), (_, b_goal) = LAYOUTS[layout]
    expected_order = ['B', 'R'] if policy == 'lp' and b_goal > r_goal else ['R', 'B']
    if [event['robot'] for event in grants] != expected_order:
        raise RuntimeError(f'{mode}/{policy} did not grant passage in the expected order')
    if not push_pull and (len(stops) != len(robots) or extensions):
        raise RuntimeError('Stop-first mode must stop both robots before granting passage')
    if push_pull and layout in ('farther-longer', 'early-follower'):
        expected_stops = (['R'] if policy == 'lp' else
                          ['B'] if layout == 'early-follower' else [])
        expected_triggers = ['push', 'after_stop' if expected_stops else 'pull']
        if (stops, [event['trigger'] for event in grants]) != (expected_stops, expected_triggers):
            raise RuntimeError(f'{layout}/{policy} did not exercise the intended stop and release behavior')
    profile_changes = [f'{robot.name}:{phase}' for robot in robots
                       for phase in ('accelerating', 'cruising', 'braking')
                       for _ in range(robot._variable.motion_extension_counts[phase])]
    if layout == DEFAULT_ANIMATION_LAYOUT and push_pull and policy == 'fifo':
        if stops or [event['trigger'] for event in grants] != ['push', 'pull'] or any(
                robot._variable.motion_extension_counts['braking'] for robot in robots):
            raise RuntimeError('Both approach profiles must extend before braking.')
    return max(event['time'] for event in arrivals), stops, grants, profile_changes


def main():
    parser = ArgumentParser(description=__doc__)
    mode_options = parser.add_mutually_exclusive_group()
    mode_options.add_argument('--mode', choices=MODES)
    mode_options.add_argument('--push-pull', choices=('yes', 'no'),
                              help='yes enables Push/Pull; no uses stop-first')
    parser.add_argument('--policy', choices=POLICIES)
    parser.add_argument('--layout', choices=LAYOUTS)
    parser.add_argument('--animate', action='store_true',
                        help=f'open the viewer; defaults to {DEFAULT_ANIMATION_LAYOUT}, push-pull, FIFO')
    args, viewer_args = parser.parse_known_args()
    if args.push_pull is not None:
        args.mode = 'push-pull' if args.push_pull == 'yes' else 'stop-first'
    if args.animate:
        layout, mode, policy = (args.layout or DEFAULT_ANIMATION_LAYOUT, args.mode or 'push-pull',
                                args.policy or 'fifo')
        paths, grid = crossing_geometry(layout)

        def model_factory(**options):
            return build_model(paths, grid, mode, policy, **options)

        def toggle_passage():
            nonlocal mode
            mode = 'stop-first' if MODES[mode] else 'push-pull'

        import_module('examples.RMFS.Tests.01_01_RobotPath_Straight').main(
            name=f'02_13 Crossing | {layout} | {policy.upper()}',
            robot_paths=paths, grid=grid, model_factory=model_factory,
            no_rack=True, collision_check=True, argv=viewer_args,
            toggle_passage=toggle_passage)
        return
    if viewer_args:
        parser.error('unrecognized arguments: ' + ' '.join(viewer_args))
    print('Mode        Rule  Makespan (s)  Approach stops  Grants (simulation seconds)  Profile changes')
    for layout in (args.layout,) if args.layout else LAYOUTS:
        print(f'\n{layout} (R and B approach/exit distances: {LAYOUTS[layout]})')
        for mode in (args.mode,) if args.mode else MODES:
            for policy in (args.policy,) if args.policy else POLICIES:
                time, stops, grants, profile_changes = run(mode, policy, layout)
                decisions = ', '.join(f"{e['robot']}:{e['trigger']}@{e['time']:.3f}" for e in grants)
                print(f'{mode:11} {policy.upper():4} {time:12.3f}  {",".join(stops) or "-":14}  '
                      f'{decisions:45}  {", ".join(profile_changes) or "-"}')


if __name__ == '__main__':
    main()
