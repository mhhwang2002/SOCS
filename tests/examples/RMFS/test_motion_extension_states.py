"""State-boundary regressions for command-only motion extension (no lookahead)."""
from copy import deepcopy
from math import atan2, inf, isfinite, radians, sqrt
from types import SimpleNamespace

import pytest
from socs.devs import DEVSph
from socs.devs.TESEngin import TESEnginBase
from socs.devs.TESBase import PortValue
from examples.RMFS.MRobot import MRobot
from examples.RMFS.PSensor import PSensor
from examples.RMFS.model import RMFS


class Reports(DEVSph):
    """Test observer; captures actual coupled outputs, including snapshot replay."""
    def __init__(self):
        super().__init__('Reports',variables={'events':[]})
        self.add_input_ports('?sense','?plan','?arrived','?rotation_started','?rotation_finished')
        self.add_phase('Idle',inf,initial=True)

    def delta_x(self,event):
        if event is not None:
            self._variable.events.append((self.tNow,event.port,deepcopy(event.value)))
        return False


def plant(path, *, loaded=False, heading=None):
    robot = MRobot('R1',path[0],lift_up=loaded,initial_heading_deg=heading,
                   command_coordinates='metres',control_points={path[0]:path[0]})
    sensor,observer = PSensor('S1'),Reports()
    model = RMFS()
    model.add_components(robot,sensor,observer)
    model.add_couples(((robot,'!motion_plan'),(sensor,'?motion_plan')))
    for source,output,input_port in ((robot,'!motion_plan','?plan'),
                                    (sensor,'!sense','?sense'),
                                    (robot,'!arrived','?arrived'),
                                    (robot,'!rotation_started','?rotation_started'),
                                    (robot,'!rotation_finished','?rotation_finished')):
        model.add_couples(((source,output),(observer,input_port)))
    engine = TESEnginBase(model)
    engine.bVerbose = False
    return SimpleNamespace(model=model,robot=robot,sensor=sensor,observer=observer,engine=engine)


def send(p,port,value, target=None):
    return p.engine.execute_devs_x(p.engine.tNow,target or p.robot,PortValue(port,value))


def advance(p,time, *, include_equal=True):
    for _ in range(1000):
        next_time,_ = p.engine.pick_next_execution_time_type()
        if next_time > time or (not include_equal and next_time == time):
            break
        p.engine.execute_one_step()
    else:
        pytest.fail('Too many events while advancing to input time.')
    p.model.Delta_x(None,time)
    p.engine.tNow = time


def finish(p):
    for _ in range(1000):
        if not isfinite(p.model.tNext):
            assert p.robot.phase == 'Wait'
            assert p.sensor.phase == 'Idle'
            return p.engine.tNow,deepcopy(p.observer.get_state())
        p.engine.execute_one_step()
    pytest.fail('Physical motion failed to finish.')


def events(p,port):
    return [(t,value) for t,kind,value in p.observer._variable.events if kind == port]


@pytest.mark.parametrize('loaded',[False,True],ids=['empty','loaded'])
@pytest.mark.parametrize('fraction',[.2,.55,.85],ids=['accelerating','cruising','braking'])
def test_extension_in_each_speed_regime_preserves_motion_and_clearances(loaded,fraction):
    path = tuple((float(x),0.) for x in range(0,17,2))
    p = plant(path,loaded=loaded)
    send(p,'?command',path)
    advance(p,0.)
    old = p.robot._variable.profile
    if fraction == .2:
        time = old.peak/p.robot.motion_limits.acceleration/2
    elif fraction == .55:
        time = (old.peak/p.robot.motion_limits.acceleration+old.brake_start)/2
    else:
        time = (old.brake_start+old.duration)/2
    advance(p,time)
    before = p.robot.position_at(time),old.velocity_at(time)
    extended = (*path,(18.,0.),(20.,0.),(24.,0.))
    send(p,'?extend_motion',{'path':extended})
    new = p.robot._variable.profile
    category = { .2:'accelerating', .55:'cruising', .85:'braking'}[fraction]
    assert p.robot._variable.motion_extension_counts[category] == 1
    assert sum(p.robot._variable.motion_extension_counts.values()) == 1
    assert (p.robot.position_at(time),new.velocity_at(time)) == before
    assert new.velocity_at(time+1e-5) >= before[1]-1e-9
    finish(p)
    assert events(p,'?arrived')[-1][1] == (24.,0.)
    assert len(events(p,'?plan')) == 2
    reports = events(p,'?sense')
    assert [position for _,position in reports] == [(point[0]+.75,0.) for point in extended[:-1]]
    for t,position in reports:
        assert p.robot._variable.profile.distance_at(t) == pytest.approx(position[0])


def test_intermediate_collinear_point_does_not_prevent_extension():
    path = ((0.,0.),(4.,0.),(8.,0.),(16.,0.))
    p = plant(path)
    send(p,'?command',path)
    advance(p,4.6)
    assert p.robot.get_current_segment_destination() == (8.,0.)
    send(p,'?extend_motion',{'path':(*path,(24.,0.))})
    finish(p)
    assert [point for _,point in events(p,'?arrived')] == [(24.,0.)]
    assert not events(p,'?rotation_started')


@pytest.mark.parametrize('during_rotation',[False,True],ids=['before_turn','during_turn'])
def test_destination_extension_is_retained_without_restarting_run_or_turn(during_rotation):
    path = ((0.,0.),(4.,0.),(4.,4.))
    p = plant(path)
    send(p,'?command',path)
    advance(p,0.)
    old = p.robot._variable.profile
    time = old.duration+.5 if during_rotation else 1.
    advance(p,time)
    deadline = p.robot.tNext
    original_profile = p.robot._variable.profile
    send(p,'?extend_motion',{'path':(*path,(4.,8.))})
    assert p.robot.tNext == deadline
    assert p.robot._variable.profile is original_profile
    assert not p.robot._variable.motion_plan_pending
    snapshot = p.engine.capture_snapshot()
    first = finish(p)
    assert [point for _,point in events(p,'?arrived')] == [(4.,8.)]
    turns = events(p,'?rotation_started'),events(p,'?rotation_finished')
    assert len(turns[0]) == len(turns[1]) == 1
    assert turns[1][0][0]-turns[0][0][0] == pytest.approx(2.)
    p.engine.restore_snapshot(snapshot)
    assert finish(p) == first


def test_extension_adding_turn_preserves_stop_at_corner_without_false_arrival():
    path = ((0.,0.),(4.,0.))
    p = plant(path)
    send(p,'?command',path)
    advance(p,1.)
    old = p.robot._variable.profile
    send(p,'?extend_motion',{'path':(*path,(4.,4.))})
    assert p.robot._variable.profile is old
    finish(p)
    assert events(p,'?rotation_started')[0] == (old.duration,(4.,0.))
    assert [position for _,position in events(p,'?arrived')] == [(4.,4.)]


@pytest.mark.parametrize('heading',[None,90.],ids=['aligned_start','entry_rotation'])
def test_extension_before_first_motion_plan_is_not_lost(heading):
    path = ((0.,0.),(4.,0.))
    p = plant(path,heading=heading)
    send(p,'?command',path)
    assert p.robot.phase == 'StartMove'
    send(p,'?extend_motion',{'path':(*path,(8.,0.))})
    finish(p)
    assert [position for _,position in events(p,'?arrived')] == [(8.,0.)]
    assert len(events(p,'?plan')) == 1
    assert p.robot._variable.profile.length == 8.


@pytest.mark.parametrize("loaded", [False, True])
def test_small_entry_rotation_accelerates_and_brakes_without_cruising(loaded):
    path = ((0., 0.), (4., 0.))
    p = plant(path, loaded=loaded, heading=5.)
    send(p, '?command', path)
    advance(p, 0.)
    assert p.robot.phase == 'RotateTo'
    acceleration_time = sqrt(radians(5.) / 2.5)
    started = events(p, '?rotation_started')[0][0]
    direction = p.robot.direction_at(started + acceleration_time / 2)
    assert atan2(direction[1], direction[0]) == pytest.approx(radians(5.) * 7 / 8)
    finish(p)
    assert events(p, '?rotation_finished')[0][0] - started == pytest.approx(2 * acceleration_time)


def test_stopped_robot_receives_new_command_starting_from_rest():
    p = plant(((0.,0.),(4.,0.)))
    assert p.robot.phase == 'Wait'
    send(p,'?command',((0.,0.),(4.,0.)))
    finish(p)
    send(p,'?command',((4.,0.),(8.,0.)))
    assert p.robot._variable.profile.velocity_at(0.) == 0.
    finish(p)
    assert [position for _,position in events(p,'?arrived')] == [(4.,0.),(8.,0.)]


@pytest.mark.parametrize('extension',[
    ((0.,0.),(3.,0.),(8.,0.)),
    ((0.,0.),(4.,0.),(4.,0.)),
    ((0.,0.),(4.,0.),(6.,2.)),
    ((0.,0.),(4.,0.),(float('nan'),0.)),
    ((0.,0.),(4.,0.)),
])
def test_invalid_extension_does_not_change_plant_or_sensor(extension):
    path = ((0.,0.),(4.,0.))
    p = plant(path)
    send(p,'?command',path)
    advance(p,1.)
    before = p.engine.capture_snapshot()
    with pytest.raises(ValueError):
        send(p,'?extend_motion',{'path':extension})
    assert p.robot.get_state() == (before.root_model.children['R1'].model_state)
    assert p.sensor.get_state() == (before.root_model.children['S1'].model_state)


@pytest.mark.parametrize('already_emitted',[False,True],ids=['report_still_due','report_already_emitted'])
def test_sensor_replacement_at_exact_clearance_time_reports_once(already_emitted):
    path = ((0.,0.),(2.,0.),(4.,0.))
    p = plant(path)
    send(p,'?command',path)
    advance(p,0.)
    due = p.sensor._variable.pending_events[0][0]
    advance(p,due,include_equal=already_emitted)
    send(p,'?extend_motion',{'path':(*path,(8.,0.))})
    finish(p)
    positions = [position for _,position in events(p,'?sense')]
    assert positions == [(.75,0.),(2.75,0.),(4.75,0.)]


def test_sensor_keeps_clearance_for_point_behind_center_but_under_footprint():
    path = ((0.,0.),(2.,0.),(4.,0.))
    p = plant(path)
    send(p,'?command',path)
    advance(p,0.)
    time = p.robot._variable.profile.time_at_distance(2.25)
    advance(p,time)
    send(p,'?extend_motion',{'path':(*path,(8.,0.))})
    advance(p,time)
    assert (2.75,0.) in [position for _,position,_ in p.sensor._variable.pending_events]
    finish(p)
    assert [position for _,position in events(p,'?sense')].count((2.75,0.)) == 1


def test_multiple_extensions_preserve_all_reports_without_intermediate_arrival():
    path = ((0.,0.),(4.,0.))
    p = plant(path)
    send(p,'?command',path)
    for time,next_point in ((1.,(8.,0.)),(2.,(12.,0.)),(3.,(16.,0.))):
        advance(p,time)
        path = (*path,next_point)
        send(p,'?extend_motion',{'path':path})
        advance(p,time)
    finish(p)
    assert len(events(p,'?plan')) == 4  # once per initial/extension command
    assert [point for _,point in events(p,'?arrived')] == [(16.,0.)]
    assert [point for _,point in events(p,'?sense')] == [(.75,0.),(4.75,0.),(8.75,0.),(12.75,0.)]


def test_collision_cancels_motion_and_pending_sensor_schedule():
    path = ((0.,0.),(4.,0.))
    p = plant(path)
    send(p,'?command',path)
    advance(p,1.)
    send(p,'?extend_motion',{'path':(*path,(8.,0.))})
    position = p.robot.position_at(p.engine.tNow)
    plan_count = len(events(p,'?plan'))
    send(p,'?collision',('R2',))
    send(p,'?collision',('R2',),target=p.sensor)
    assert p.robot.phase == p.sensor.phase == 'Collided'
    assert p.model.tNext == inf
    assert p.robot.position_at(100.) == position
    assert not p.sensor._variable.pending_events
    assert len(events(p,'?plan')) == plan_count
    with pytest.raises(ValueError):
        send(p,'?extend_motion',{'path':(*path,(8.,0.),(12.,0.))})
