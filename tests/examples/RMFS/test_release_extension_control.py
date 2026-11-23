"""Controller/blackboard/Plant contract for real-release motion extensions."""
from copy import deepcopy
from importlib import import_module
from math import isfinite

import pytest
from socs.devs.TESEngin import TESEnginBase


def experiment():
    example = import_module('examples.RMFS.Tests.03_09_RobotPath_FIFO_ReleaseExtension')
    model,robots,sensors = example.create_model()
    engine = TESEnginBase(model,TimeIntervalAnimation=.1)
    engine.bVerbose = False
    return model,robots,sensors,engine


def test_no_extension_while_actual_conflict_has_not_cleared():
    model,robots,_,engine = experiment()
    road = model.controller.roadmap
    for _ in range(3000):
        if engine.pick_next_execution_time_type()[0] > 8.5:
            break
        engine.execute_one_step()
    else:
        pytest.fail('Could not reach braking interval.')
    assert not any(e['event']=='extension_reserved' for e in road.history)
    robot = next(r for r in robots if r.name=='R2')
    profile = robot._variable.profile
    elapsed = engine.tNow-robot._variable.start_time
    assert profile.brake_start < elapsed < profile.duration
    assert robot._variable.path[-1] == (16.,0.)
    assert not model.collision_history


def test_reservation_precedes_output_and_snapshot_replays_buffered_extension():
    model,robots,sensors,engine = experiment()
    manager = model.controller.traffic_manager
    road = model.controller.roadmap
    robot = next(r for r in robots if r.name=='R2')
    for _ in range(3000):
        if any('extension' in command for command in manager.command_buffer):
            break
        engine.execute_one_step()
    else:
        pytest.fail('No actual-release extension was buffered.')
    snapshot = engine.capture_snapshot()
    command = next(c for c in manager.command_buffer if 'extension' in c)
    record = road.robots[robot.name]
    assert record['approved_path'].coordinates()[-1] == command['extension']['path'][-1]
    assert robot._variable.path[-1] == (16.,0.)  # command not delivered yet
    assert record['reported_position'] != command['extension']['path'][-1]
    extension_index = next(i for i,e in enumerate(road.history) if e['event']=='extension_reserved')
    assert any(e['event'] in ('released','arrived') and e['robot']=='R1'
               and e['time']==engine.tNow for e in road.history[:extension_index])
    def complete():
        for _ in range(4000):
            if not isfinite(model.tNext):
                break
            engine.execute_one_step()
        else:
            pytest.fail('Actual-release execution did not complete.')
        assert manager.done and not model.collision_history
        assert not manager.awaiting and not manager.command_buffer
        for plant in robots:
            assert plant._variable.path[-1] == road.paths[plant.name].vertices[-1]
            assert plant.phase == 'Wait'
            # Completion keeps the endpoint occupied; it is not a new release.
            assert road.remaining_subpath(plant.name).is_point
        return engine.tNow,deepcopy(model.controller.get_snapshot()),deepcopy(
            [(r.get_state(),s.get_state()) for r,s in zip(robots,sensors)])
    first = complete()
    engine.restore_snapshot(snapshot)
    assert complete() == first


def test_each_approval_causes_exactly_one_profile_change_on_moving_robot():
    model,robots,_,engine = experiment()
    robot = next(r for r in robots if r.name=='R2')
    changes = []
    for _ in range(4000):
        if not isfinite(model.tNext):
            break
        before = robot._variable.profile
        engine.execute_one_step()
        after = robot._variable.profile
        if before is not None and after is not before and robot.phase=='MoveTo':
            changes.append(engine.tNow)
    else:
        pytest.fail('Experiment did not finish.')
    approvals = [e['time'] for e in model.controller.roadmap.history
                 if e['event']=='extension_reserved' and e['robot']=='R2']
    assert changes == pytest.approx(approvals)
    assert len(changes) == 2
    assert sum(robot._variable.motion_extension_counts.values()) == len(changes)
    assert not model.collision_history


def test_extension_toggle_changes_only_extension_not_safe_prefix_or_fifo():
    example = import_module('examples.RMFS.Tests.03_09_RobotPath_FIFO_ReleaseExtension')
    from examples.RMFS.controlled import create_controlled_model
    results = []
    for enabled in (False,True):
        model,robots,_ = create_controlled_model(example.PATHS,grid=example.GRID,lift_up=False,
            event_release=True,merge_sweeps=True,safe_prefix=True,motion_extension=enabled,
            collision_check=True,stop_on_collision=True)
        engine = TESEnginBase(model,TimeIntervalAnimation=.1)
        engine.bVerbose = False
        manager = model.controller.traffic_manager
        assert manager.safe_prefix and manager.event_release
        assert {c['robot'] for c in manager.command_buffer} == {'R1','R2','R3'}
        for _ in range(4000):
            if not isfinite(model.tNext):
                break
            engine.execute_one_step()
        else:
            pytest.fail('Paired comparison stalled.')
        assert manager.done and not model.collision_history
        counts = sum(sum(r._variable.motion_extension_counts.values()) for r in robots)
        assert bool(counts) == enabled
        results.append(engine.tNow)
    assert results[1] < results[0]


def test_zero_time_controller_command_precedes_simultaneous_child_event():
    from socs.devs import DEVSph
    from examples.RMFS.model import RMFS
    model = RMFS()
    robot = DEVSph('ARobot')
    controller = DEVSph('ZController')
    for child in (robot,controller):
        child.add_phase('Command',0,initial=True)
        child.add_phase('Wait',float('inf'))
        child.add_internal_transition('Command','Wait')
    controller.traffic_manager = object()
    model.add_components(robot,controller)
    engine = TESEnginBase(model)
    engine.bVerbose = False
    engine.execute_one_step()
    assert controller.phase == 'Wait' and robot.phase == 'Command'
    assert engine.tNow == 0.
    engine.execute_one_step()
    assert robot.phase == 'Wait'
