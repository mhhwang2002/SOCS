from math import inf, isfinite

import pytest

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.TrafficManager import TrafficManager
from examples.RMFS.RoadMap import RoadMap
from examples.RMFS.path_geometry import Path, TuplePath
from examples.RMFS.DEVSModelCreator import create_model
from examples.RMFS.GridPath import make_grid


@pytest.mark.parametrize('advance', (False, True))
def test_blocked_destination_allows_only_safe_prefix_in_advance_mode(advance):
    starts = {'A': Path('A', ((0., 0.),)), 'B': Path('B', ((6., 0.),))}
    manager = TrafficManager(RoadMap(starts), TuplePath(tuple(starts), ()), event_release=True)
    manager.append_route('B', Path('B', ((6., 0.), (6., 2.), (6., 4.))), 0., advance=advance)
    manager.emit_commands(0.)
    manager.append_route('A', Path('A', ((0., 0.), (2., 0.), (4., 0.), (6., 0.))), 0., advance=advance)
    commands = manager.emit_commands(0.)
    if advance:
        assert len(commands) == 1
        assert commands[0]['path'] == ((0., 0.), (2., 0.), (4., 0.))
        # Approval does not teleport A; it must actually arrive at its safe stop.
        assert manager.roadmap.robots['A']['reported_position'] == (0., 0.)
        manager.arrived('A', (4., 0.), 1.)
        assert not manager.command_buffer
    else:
        assert not commands
    snapshot = manager.get_snapshot()
    manager.leaving('B', (6., 2.75), 2.)
    resumed = manager.emit_commands(2.)
    assert len(resumed) == 1 and resumed[0]['robot'] == 'A'
    assert resumed[0]['path'][-1] == (6., 0.)
    manager.set_snapshot(snapshot)
    manager.leaving('B', (6., 2.75), 2.)
    assert manager.emit_commands(2.) == resumed


def test_advance_full_cycle_collision_stop_and_snapshot_reset():
    model = create_model(robot_count=1, order_count=4, full_cycle=True,
        departure_policy='advance', collision_check=True, layout=make_grid(),
        robot_starts={'R01': (0,0)},
        rack_positions={'A': (6.,0.), 'B': (6.,6.)},
        workstations={'W01': (6.,2.), 'W02': (6.,4.)})
    engine = TESEnginBase(model, TimeIntervalAnimation=.1)
    engine.bVerbose = False
    for _ in range(20):
        engine.execute_one_step()
    snapshot = engine.capture_snapshot()
    def finish():
        steps = 0
        while isfinite(model.tNext):
            engine.execute_one_step()
            steps += 1
            assert steps < 20000
        assert not model.collision_history
        assert model.controller.completed_orders == model.controller.shipped_orders == 4
        assert set(model.controller.robot_status.values()) == {'home'}
        return engine.tNow, model.controller.get_snapshot()
    final = finish()
    engine.restore_snapshot(snapshot)
    assert finish() == final
    engine.reset()
    assert finish() == final
