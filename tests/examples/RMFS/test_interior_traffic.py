from math import isfinite

import pytest

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.interior_traffic import create_interior_model, make_interior_traffic


def cyclic(graph):
    state = {}
    def visit(node):
        if state.get(node) == 1:
            return True
        if state.get(node) == 2:
            return False
        state[node] = 1
        if any(visit(other) for other in graph[node]):
            return True
        state[node] = 2
        return False
    return any(visit(node) for node in graph)


@pytest.mark.parametrize("size,count", ((4,8),(10,50),(20,200)))
def test_interior_simple_routes_interfere_have_static_cycles_and_complete(size,count):
    grid,routes,_ = make_interior_traffic(size)
    assert len(routes) == count
    for name,route in routes.items():
        assert len(route) == len(set(route)) == 8
        assert route[-1] != route[0]
        assert any(0<x<size-1 and 0<y<size-1 for x,y in route)
        assert any(set(route)&set(other) for key,other in routes.items() if key!=name)
    graph = {i:[j for j,b in routes.items() if i!=j and
                 (b[0] in a or a[-1] in b)] for i,a in routes.items()}
    assert cyclic(graph)  # static whole-path serial requirements, NOT a current wait-for graph
    model,robots,sensors = create_interior_model(size)
    engine = TESEnginBase(model,TimeIntervalAnimation=.01)
    engine.bVerbose = False
    iterations = 0
    while isfinite(model.tNext):
        engine.execute_one_step()
        iterations += 1
        assert iterations < 100000
    assert not model.collision_history
    assert model.controller.traffic_manager.done
    assert model.controller.traffic_manager.next_command_id == 7*count
    for robot,sensor in zip(robots,sensors):
        assert robot.phase == "Wait" and sensor.phase == "Idle"
        assert robot.position_at(engine.tNow) == grid["control_points"][routes[robot.name][-1]]
