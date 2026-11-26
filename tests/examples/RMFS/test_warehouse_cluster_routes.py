from functools import partial
from math import isfinite

import pytest

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.Tests.warehouse_cluster_routes import (
    CROSS_RACKS, CROSS_STARTS, cluster_layout, create_model, crossed_witness, plan_routes,
)
from examples.RMFS.TrafficManager import TrafficManager


@pytest.mark.parametrize("allow_crossing_stops", [False, True])
def test_crossed_route_extension_obeys_endpoint_policy(allow_crossing_stops):
    grid = cluster_layout(crossed=True)
    routes = plan_routes(grid, CROSS_STARTS, CROSS_RACKS, sweep_aware=True)
    model, robots, _ = create_model(
        grid, routes, collision_check=True, plan_builder=crossed_witness,
        traffic_manager_type=partial(TrafficManager, allow_crossing_stops=allow_crossing_stops),
    )
    engine = TESEnginBase(model, TimeIntervalAnimation=.005)
    engine.bVerbose = False
    while isfinite(model.tNext):
        engine.execute_one_step()

    assert model.controller.traffic_manager.done
    assert not model.collision_history
    assert all(robot.position_at(engine.tNow) == routes[robot.name][-1] for robot in robots)
    arrivals = [event["position"] for event in model.controller.roadmap.history
                if event["robot"] == "R1" and event["event"] == "arrived"]
    horizontal_stops = [x for x, y in arrivals if y == -16.0 and 0 < x < 15.725]
    expected = [] if allow_crossing_stops else [2.0, 4.8625, 7.725, 9.925, 12.825]
    assert horizontal_stops == expected
