from math import isfinite

import pytest

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.path_geometry import Path, PathPosition, Subpath, TuplePath
from examples.RMFS.sweep_operations import compact_tuple, merge_parts, split_at_vertex
from examples.RMFS.Tests.incremental_routing import create_experiment_model


def part(path, a, b):
    return Subpath(path, PathPosition(a), PathPosition(b))


def test_split_restores_existing_vertex_and_merge_preserves_bend():
    path = Path('R1', ((0.,0.),(2.,0.),(2.,2.)))
    whole = part(path,0,2)
    halves = split_at_vertex(whole,1)
    assert merge_parts(halves) == whole
    assert merge_parts(halves).coordinates() == whole.coordinates()
    assert (2.,0.) in whole.coordinates()
    with pytest.raises(ValueError):
        split_at_vertex(whole,0)


def test_cross_stage_interference_prevents_unsafe_merge():
    a = Path('A', ((0.,2.),(4.,2.)))
    b = Path('B', ((2.,0.),(2.,4.)))
    plan = TuplePath(('A','B'), ((part(a,0,1),part(b,0,0)),
                                (part(a,1,1),part(b,0,1))))
    compact, blocks = compact_tuple(plan)
    assert compact == plan
    assert blocks == ((0,1),(1,2))


@pytest.mark.parametrize('count', (10,20))
def test_compacted_plan_completes_with_sampled_collision_verification(count):
    times = []
    commands = []
    for merged in (False,True):
        model, robots, sensors = create_experiment_model(
            robot_count=count, merge_sweeps=merged,
            collision_check=True, stop_on_collision=True)
        engine = TESEnginBase(model,TimeIntervalAnimation=.01)
        engine.bVerbose = False
        iterations = 0
        while isfinite(model.tNext):
            engine.execute_one_step()
            iterations += 1
            assert iterations < 200000
        assert model.controller.traffic_manager.done
        assert not model.collision_history
        for robot, sensor in zip(robots,sensors):
            assert robot.phase == 'Wait' and sensor.phase == 'Idle'
            goal = model.routing_experiment['routes'][robot.name][-1]
            assert robot.position_at(engine.tNow) == model.routing_experiment['grid']['control_points'][goal]
        times.append(engine.tNow)
        commands.append(model.controller.traffic_manager.next_command_id)
    assert times[1] < times[0]  # empirical fixture result, not a general timing theorem
    assert commands[1] < commands[0]
