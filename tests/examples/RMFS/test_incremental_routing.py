from math import isfinite

import pytest

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.Tests.incremental_routing import (
    create_experiment_model, dependency_order, generate,
)


def test_dependency_direction_and_cycle():
    assert dependency_order({'R1': {'R2'}, 'R2': set()}) == ('R2', 'R1')
    assert dependency_order({'R1': {'R2'}, 'R2': {'R1'}}) is None


@pytest.mark.parametrize('count', (10, 20))
def test_incremental_candidates_complete_in_real_plant(count):
    data = generate(robot_count=count)
    assert data['routes'] == generate(robot_count=count)['routes']
    assert len(data['routes']) == count
    assert dependency_order(data['requires']) is not None
    for route in data['routes'].values():
        assert len(route) == len(set(route))
        assert (route[0][0] in (0, 9)) != (route[0][1] in (0, 9))
        assert 0 < route[-1][0] < 9 and 0 < route[-1][1] < 9
    model, robots, sensors = create_experiment_model(
        robot_count=count, collision_check=True, stop_on_collision=True,
    )
    engine = TESEnginBase(model, TimeIntervalAnimation=.01)
    engine.bVerbose = False
    iterations = 0
    while isfinite(model.tNext):
        engine.execute_one_step()
        iterations += 1
        assert iterations < 200000
    assert not model.collision_history
    assert model.controller.traffic_manager.done
    for robot, sensor in zip(robots, sensors):
        assert robot.phase == 'Wait' and sensor.phase == 'Idle'
        assert robot.position_at(engine.tNow) == data['grid']['control_points'][data['routes'][robot.name][-1]]
