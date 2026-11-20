from math import isfinite
from itertools import combinations

import pytest

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.convoy import create_convoy_model, make_convoy
from examples.RMFS.path_geometry import sweep_2d


@pytest.mark.parametrize("size,count", ((4,6),(10,18),(20,38)))
def test_convoy_simple_interfering_routes_and_actual_plant(size,count):
    grid,routes,headings=make_convoy(size)
    assert len(routes)==count
    for route in routes.values():
        assert len(route)==6 and len(set(route))==6 and route[0]!=route[-1]
    model,robots,sensors=create_convoy_model(size)
    # Every full route overlaps at least one other full route.
    plan=model.plan_result.tuple_path
    from examples.RMFS.path_geometry import Subpath,PathPosition
    paths={name:next(row[i].path for row in plan.stages)
           for i,name in enumerate(plan.object_names)}
    sweeps={name:sweep_2d(Subpath(p,PathPosition(0),PathPosition(5))) for name,p in paths.items()}
    assert all(any(sweeps[a].intersects(sweeps[b]) for b in paths if b!=a) for a in paths)
    engine=TESEnginBase(model,TimeIntervalAnimation=.01)
    engine.bVerbose=False
    iterations=0
    while isfinite(model.tNext):
        engine.execute_one_step();iterations+=1
        assert iterations<100000
    assert not model.collision_history and model.controller.traffic_manager.done
    assert model.controller.traffic_manager.next_command_id==5*count
    for robot,sensor in zip(robots,sensors):
        assert robot.phase=="Wait" and sensor.phase=="Idle"
        assert robot.position_at(engine.tNow)==grid["control_points"][routes[robot.name][-1]]
