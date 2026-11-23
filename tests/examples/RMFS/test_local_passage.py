"""Actual-clearance FIFO and controller-commanded profile replacement."""
from importlib import import_module
from math import inf, isfinite

import pytest
from socs.devs.TESEngin import TESEnginBase
from socs.devs.TESBase import PortValue
from examples.RMFS.MRobot import MRobot
from examples.RMFS.PSensor import PSensor
from examples.RMFS.model import RMFS
from examples.RMFS.motion import StraightProfile, ExtendedProfile
from examples.RMFS.path_geometry import include_path_points


@pytest.mark.parametrize('switch',[1.,6.,9.])
def test_controller_extension_updates_sensor_in_accel_cruise_and_braking(switch):
    points = {(x,0):(float(x),0.) for x in range(0,25,2)}
    robot = MRobot('R1',(0,0),lift_up=False,command_coordinates='metres',control_points=points)
    sensor = PSensor('S1')
    model = RMFS()
    model.add_components(robot,sensor)
    model.add_couples(((robot,'!motion_plan'),(sensor,'?motion_plan')))
    engine = TESEnginBase(model)
    engine.bVerbose = False
    path = tuple(points[x,0] for x in range(0,17,2))
    engine.execute_devs_x(0.,robot,PortValue('?command',path))
    while engine.pick_next_execution_time_type()[0] <= switch:
        engine.execute_one_step()
    model.Delta_x(None,switch)
    engine.tNow = switch
    old_profile = robot._variable.profile
    old_position = robot.position_at(switch)
    old_speed = old_profile.velocity_at(switch)
    old_pending = tuple(sensor._variable.pending_events)
    extended = tuple(points[x,0] for x in range(0,25,2))
    engine.execute_devs_x(switch,robot,PortValue('?extend_motion',{'path':extended}))
    assert robot.position_at(switch) == old_position
    assert robot._variable.profile.velocity_at(switch) == pytest.approx(old_speed)
    assert robot._variable.motion_plan_pending
    snapshot = engine.capture_snapshot()
    def finish():
        engine.execute_one_step()  # zero-time replacement !motion_plan
        assert engine.tNow == switch
        assert not robot._variable.motion_plan_pending
        pending = tuple(sensor._variable.pending_events)
        assert pending != old_pending
        assert all(t > switch for t,_,_ in pending)
        assert len({position for _,position,_ in pending}) == len(pending)
        for _ in range(1000):
            if not isfinite(model.tNext):
                break
            engine.execute_one_step()
        else:
            pytest.fail('Extended motion failed to finish.')
        assert robot.phase == 'Wait'
        assert robot.position_at(engine.tNow) == (24.,0.)
        assert sensor.phase == 'Idle'
        return engine.tNow,robot.get_state(),sensor.get_state()
    final = finish()
    engine.restore_snapshot(snapshot)
    assert finish() == final
    assert '!motion_status' not in robot.OP


def test_profile_inverse_and_continuity():
    old = StraightProfile(16.,acceleration=.41,deceleration=.41)
    for switch in (1.,6.,9.):
        profile = ExtendedProfile(old,switch,24.,.41,.41,2.)
        for k in range(101):
            t = profile.duration*k/100
            assert profile.time_at_distance(profile.distance_at(t)) == pytest.approx(t,abs=1e-6)
        assert profile.velocity_at(switch-1e-7) == pytest.approx(profile.velocity_at(switch+1e-7),abs=1e-6)


def test_clearance_only_example_starts_all_three_and_completes():
    example = import_module('examples.RMFS.Tests.03_08_RobotPath_FIFO_LocalPassage')
    model,robots,sensors = example.create_model()
    engine = TESEnginBase(model,TimeIntervalAnimation=.1)
    engine.bVerbose = False
    road = model.controller.roadmap
    assert {e['robot'] for e in road.history if e['event']=='approved'} == {'R1','R2','R3'}
    assert road.robots['R3']['approved_path'].coordinates() == ((6.,0.),(6.,1.))
    for _ in range(3000):
        if model.tNext == inf:
            break
        engine.execute_one_step()
    else:
        pytest.fail('Fleet did not complete.')
    assert all('!motion_status' not in robot.OP for robot in robots)
    assert not any('motion_status' in port for port in model.controller.IP)
    stops = [e for e in road.history if e['event']=='arrived' and e['robot']=='R2']
    assert stops[0]['position'] == (4.,4.)
    for name,expected in [('R3',[.75,1.75,4.75,7.75]),('R2',[.75,4.75,6.75,8.75])]:
        axis = 1 if name=='R3' else 0
        positions = [e['position'][axis] for e in road.history if e['event']=='released' and e['robot']==name]
        assert positions == pytest.approx(expected)
    assert not model.collision_history
    assert model.controller.traffic_manager.done


def test_point_expansion_preserves_geometry():
    assert include_path_points(((0.,0.),(4.,0.)),((0.,0.),(2.,0.),(4.,0.),(2.,1.))) == ((0.,0.),(2.,0.),(4.,0.))


def test_real_release_extends_braking_robot_without_request_or_lookahead():
    example = import_module('examples.RMFS.Tests.03_09_RobotPath_FIFO_ReleaseExtension')
    model,robots,_ = example.create_model()
    engine = TESEnginBase(model,TimeIntervalAnimation=.1)
    engine.bVerbose = False
    robot = next(r for r in robots if r.name=='R2')
    changed_while_braking = False
    for _ in range(4000):
        if not isfinite(model.tNext):
            break
        old = robot._variable.profile
        start = robot._variable.start_time
        engine.execute_one_step()
        new = robot._variable.profile
        if isinstance(new,ExtendedProfile) and new is not old:
            elapsed = engine.tNow-start
            assert new.velocity_at(elapsed) == pytest.approx(old.velocity_at(elapsed))
            assert new.distance_at(elapsed) == pytest.approx(old.distance_at(elapsed))
            changed_while_braking |= elapsed > old.brake_start and old.velocity_at(elapsed)>0
    else:
        pytest.fail('Actual-release example stalled.')
    assert changed_while_braking
    assert not model.collision_history
    assert model.controller.traffic_manager.done
