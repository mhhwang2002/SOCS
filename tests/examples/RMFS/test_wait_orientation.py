from math import inf, isfinite

import pytest

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.controlled import create_controlled_model


def test_blocked_robot_rotates_before_release_then_moves_without_entry_turn():
    routes = {'R1': ((2.,-4.),(2.,4.)), 'R2': ((0.,0.),(4.,0.))}
    points = {p:p for path in routes.values() for p in path}
    model, robots, sensors = create_controlled_model(routes, grid={'control_points':points},
        initial_headings={'R1':90.,'R2':90.}, event_release=True,
        collision_check=True, stop_on_collision=True)
    engine = TESEnginBase(model, TimeIntervalAnimation=.1)
    engine.bVerbose = False
    def finish():
        steps = 0
        while isfinite(model.tNext):
            engine.execute_one_step()
            steps += 1
            assert steps < 2000
        return engine.tNow, model.controller.get_snapshot()
    while robots[1].phase != 'RotateTo':
        engine.execute_one_step()
    assert robots[1].position_at(engine.tNow) == (0.,0.)
    assert sensors[1].phase == 'Idle'  # Rotation does not schedule translation clearance.
    snapshot = engine.capture_snapshot()
    final = finish()
    assert not model.collision_history
    history = model.controller.roadmap.history
    turns = [e for e in history if e['robot']=='R2' and e['event']=='rotation_started']
    ends = [e for e in history if e['robot']=='R2' and e['event']=='rotation_finished']
    departures = [e for e in history if e['robot']=='R2' and e['event']=='issued']
    assert len(turns) == len(ends) == len(departures) == 1
    assert turns[0]['time'] == pytest.approx(0.)
    assert ends[0]['time'] == pytest.approx(2.)
    assert ends[0]['time'] < departures[0]['time']
    assert robots[1].direction_at(engine.tNow) == (1.,0.)
    engine.restore_snapshot(snapshot)
    assert finish() == final
    engine.reset()
    assert finish() == final
