from math import isfinite, inf

import pytest

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.RoadMap import RoadMap
from examples.RMFS.path_geometry import Path, PathPosition, Subpath
from examples.RMFS.Tests.incremental_routing import create_experiment_model
from examples.RMFS.controlled import create_controlled_model
from importlib import import_module


def test_reported_release_preserves_footprint_and_snapshot():
    path = Path('R1',((0.,0.),(2.,0.),(4.,0.)))
    road = RoadMap({'R1':path})
    road.approve('R1',Subpath(path,PathPosition(0),PathPosition(2)),0,0,0.)
    road.issued('R1',0,0.)
    road.report('R1','leaving',(.75,0.),2.)
    before = road.get_snapshot()
    assert road.remaining_subpath('R1').coordinates()[0] == (.75,0.)
    road.report('R1','leaving',(2.75,0.),4.)
    assert road.remaining_subpath('R1').start == PathPosition(1,.375)
    road.set_snapshot(before)
    assert road.remaining_subpath('R1').coordinates()[0] == (.75,0.)
    with pytest.raises(ValueError):
        road.report('R1','leaving',(0.,0.),5.)


def finish(model,engine):
    iterations = 0
    while isfinite(model.tNext):
        engine.execute_one_step()
        iterations += 1
        assert iterations < 200000


@pytest.mark.parametrize('release',(False,True))
@pytest.mark.parametrize('count',(10,20))
def test_merging_keeps_original_control_units_and_first_departure(release,count):
    plain,_,_ = create_experiment_model(robot_count=count,event_release=release)
    merged,_,_ = create_experiment_model(robot_count=count,event_release=release,merge_sweeps=True)
    a,b = plain.controller.traffic_manager,merged.controller.traffic_manager
    assert a.tuple_path == b.tuple_path == plain.plan_result.tuple_path
    assert a.tasks == b.tasks
    assert a.task_sweeps == b.task_sweeps
    assert {c['robot'] for c in a.command_buffer} == {c['robot'] for c in b.command_buffer}
    if release and count == 10:
        first = next(c for c in b.command_buffer if c['robot']=='R1')
        assert len(first['path']) > 2  # merge allowed prefix without waiting for full route
        assert first['path'][-1] != merged.controller.roadmap.paths['R1'].vertices[-1]


def test_event_release_snapshot_and_reset_reproduce():
    model,_,_ = create_experiment_model(robot_count=10,merge_sweeps=True,
                                       event_release=True,collision_check=False)
    engine = TESEnginBase(model)
    engine.bVerbose = False
    while not any(e['event']=='release_dispatch' for e in model.controller.roadmap.history):
        engine.execute_one_step()
    snapshot = engine.capture_snapshot()
    finish(model,engine)
    final_time,final_state = engine.tNow,model.controller.get_snapshot()
    engine.restore_snapshot(snapshot)
    finish(model,engine)
    assert engine.tNow==final_time
    assert model.controller.get_snapshot()==final_state
    engine.reset()
    finish(model,engine)
    assert engine.tNow==final_time
    assert model.controller.get_snapshot()==final_state


@pytest.mark.parametrize('example',(
    '02_03_RobotPath_SameDirection_Collision',
    '02_07_RobotPath_OppositeDirection_EntryWait',
    '02_10_RobotPath_AlternatingProgress',
    '03_04_RobotPath_SweepInterference',
    '03_05_RobotPath_LocalSafe_Deadlock',
))
def test_original_interfering_routes_complete_under_event_release(example):
    source = import_module('examples.RMFS.Tests.'+example)
    model,robots,sensors = create_controlled_model(
        source.PATHS,initial_headings=getattr(source,'INITIAL_HEADINGS',None),
        merge_sweeps=True,event_release=True,collision_check=True,stop_on_collision=True)
    engine = TESEnginBase(model,TimeIntervalAnimation=.01)
    engine.bVerbose = False
    finish(model,engine)
    assert model.controller.traffic_manager.done
    assert not model.collision_history
    assert all(robot.phase=='Wait' for robot in robots)
    assert all(sensor.phase=='Idle' for sensor in sensors)


@pytest.mark.parametrize('count',(10,20))
def test_release_dispatch_before_arrival_and_devs_only_equivalence(count):
    outcomes = []
    for interval in (inf,.01):
        model,robots,sensors = create_experiment_model(
            robot_count=count,merge_sweeps=True,event_release=True,
            collision_check=interval!=inf,stop_on_collision=True)
        engine = TESEnginBase(model,TimeIntervalAnimation=interval)
        engine.bVerbose = False
        finish(model,engine)
        manager = model.controller.traffic_manager
        road = model.controller.roadmap
        assert manager.done and not manager.awaiting and not manager.command_buffer
        assert not model.collision_history
        releases = [e for e in road.history if e['event']=='release_dispatch']
        assert releases
        for release in releases:
            assert any(e['event']=='leaving' and e['robot']==release['robot'] and
                       e['time']==release['time'] for e in road.history)
            assert any(e['event']=='arrived' and e['robot']==release['robot'] and
                       e['time']>release['time'] for e in road.history)
            for command_id in release['command_ids']:
                events = [e for e in road.history if e.get('command_id')==command_id and
                          e['event'] in ('approved','issued','arrived')]
                assert [e['event'] for e in events] == ['approved','issued','arrived']
                assert events[0]['time']==events[1]['time']==release['time']
        for robot,sensor in zip(robots,sensors):
            assert robot.phase=='Wait' and sensor.phase=='Idle'
            remaining = road.remaining_subpath(robot.name)
            assert remaining.is_point
            assert remaining.start==PathPosition(len(remaining.path.vertices)-1)
            assert robot.position_at(engine.tNow)==remaining.path.vertices[-1]
        outcomes.append((engine.tNow,manager.next_command_id))
    assert outcomes[0]==outcomes[1]
