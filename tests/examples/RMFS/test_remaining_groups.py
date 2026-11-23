from importlib import import_module
from math import isfinite

import pytest

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.RoadMap import RoadMap
from examples.RMFS.path_geometry import Path, PathPosition, Subpath, RectangularFootprint
from examples.RMFS.Tests.incremental_routing import create_experiment_model


def command(road,name):
    path = road.paths[name]
    road.approve(name,Subpath(path,PathPosition(0),PathPosition(len(path.vertices)-1)),0,0,0.)
    road.issued(name,0,0.)


def test_transitive_group_splits_after_report_and_snapshot_restores():
    road = RoadMap({
        'A':Path('A',((0.,2.),(8.,2.))),
        'B':Path('B',((2.,0.),(2.,4.))),
        'C':Path('C',((6.,0.),(6.,4.))),
    })
    assert road.components == (('A','B','C'),)
    assert road.interference_edges == (('A','B'),('A','C'))
    command(road,'A')
    snapshot = road.get_snapshot()
    road.report('A','leaving',(4.75,2.),5.)
    assert road.interference_groups == (('A','C'),)
    assert road.independent_objects == ('B',)
    road.arrival('A',(8.,2.),10.)
    assert road.interference_groups == ()
    assert road.independent_objects == ('A','B','C')
    road.set_snapshot(snapshot)
    assert road.components == (('A','B','C'),)


def test_arrived_objects_keep_occupied_point_in_graph():
    road = RoadMap({'A':Path('A',((0.,2.),(4.,2.))),
                    'B':Path('B',((2.,0.),(2.,2.)))})
    command(road,'B')
    road.arrival('B',(2.,2.),5.)
    assert road.remaining_subpath('B').is_point
    assert road.interference_groups == (('A','B'),)


def test_graph_uses_configured_footprints_and_viewer_reads_roadmap():
    paths = {'A':Path('A',((0.,0.),(2.,0.))),
             'B':Path('B',((0.,2.),(2.,2.)))}
    assert RoadMap(paths).interference_groups == ()
    road = RoadMap(paths,footprints={name:RectangularFootprint(1.5,3.) for name in paths})
    assert road.interference_groups == (('A','B'),)
    text = import_module('examples.RMFS.Tests.01_01_RobotPath_Straight').remaining_group_text(road)
    assert 'G1 (2): A, B' in text


@pytest.mark.parametrize('count',(10,20))
def test_fixed_route_edges_only_disappear_on_real_plant(count):
    model,robots,sensors = create_experiment_model(robot_count=count,merge_sweeps=True,
        event_release=True,collision_check=True,stop_on_collision=True)
    road = model.controller.roadmap
    initial_components = road.components
    previous = set(road.interference_edges)
    assert previous
    engine = TESEnginBase(model,TimeIntervalAnimation=.01)
    engine.bVerbose = False
    iterations = 0
    while isfinite(model.tNext):
        engine.execute_one_step()
        edges = set(road.interference_edges)
        assert edges <= previous
        previous = edges
        iterations += 1
        assert iterations < 200000
    assert model.controller.traffic_manager.done and not model.collision_history
    assert not road.interference_groups
    assert len(road.independent_objects)==count
    assert road.components != initial_components
    assert all(robot.phase=='Wait' and sensor.phase=='Idle' for robot,sensor in zip(robots,sensors))
