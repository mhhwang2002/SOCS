from math import inf, isfinite

import pytest

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.DEVSModelCreator import create_model
from examples.RMFS.GridPath import make_grid
from examples.RMFS.MRobot import MRobot
from examples.RMFS.PSensor import PSensor


def build(orders=100, collision=False, racks=None):
    model = create_model(robot_count=2, order_count=orders, seed=0,
        layout=make_grid(), robot_starts={"R01": (0, 0), "R02": (0, 3)},
        rack_positions=racks or {"A": (6., 0.), "B": (6., 6.)},
        collision_check=collision)
    engine = TESEnginBase(model, TimeIntervalAnimation=.1 if collision else inf)
    engine.bVerbose = False
    return model, engine


def finish(model, engine):
    steps = 0
    while isfinite(model.tNext):
        engine.execute_one_step()
        steps += 1
        assert steps < 10000


def test_initial_batch_assigns_unique_racks_and_uses_same_blackboard():
    model, engine = build()
    c = model.controller
    assert c.path_planner.roadmap is c.traffic_manager.roadmap is c.roadmap
    assert len([m for m in model.M if isinstance(m, MRobot)]) == 2
    assert len([m for m in model.M if isinstance(m, PSensor)]) == 2
    assert not c.assignments
    engine.execute_one_step()  # Generator output and controller receipt at t=0.
    assert engine.tNow == 0
    assert c.received_orders == 100 and len(c.pending_orders) == 98
    assert [a["order"] for a in c.assignments.values()] == [1, 2]
    assert len(c.rack_reservations) == 2
    for robot in model.robots:
        assert robot.position_at(0) == c.initial_paths[robot.name].vertices[0]
    finish(model, engine)
    assert set(c.robot_status.values()) == {"at_rack"}
    assert c.completed_orders == 0 and len(c.pending_orders) == 98
    for robot in model.robots:
        assert robot.position_at(engine.tNow) == c.assignments[robot.name]["rack_position"]
    history = c.roadmap.history
    assert any(e["event"] == "released" for e in history)
    for command in [e for e in history if e["event"] == "issued"]:
        assert any(e["event"] == "approved" and e["command_id"] == command["command_id"]
                   for e in history[:history.index(command)])
    # Noninterfering routes should not regress to a one-robot-at-a-time warehouse.
    assert len([e for e in history if e["event"] == "issued" and e["time"] == 0]) == 2


def test_snapshot_before_orders_and_during_motion_and_reset_reproduce():
    model, engine = build()
    before_orders = engine.capture_snapshot()
    while not any(e["event"] == "released" for e in model.controller.roadmap.history):
        engine.execute_one_step()
    moving = engine.capture_snapshot()
    finish(model, engine)
    expected, end = model.controller.get_snapshot(), engine.tNow
    for snapshot in (moving, before_orders):
        engine.restore_snapshot(snapshot)
        finish(model, engine)
        assert engine.tNow == end
        assert model.controller.get_snapshot() == expected
    engine.reset()
    finish(model, engine)
    assert engine.tNow == end
    assert model.controller.get_snapshot() == expected


@pytest.mark.parametrize("orders", (0, 1))
def test_no_orders_or_fewer_orders_than_robots(orders):
    model, engine = build(orders)
    finish(model, engine)
    c = model.controller
    assert c.received_orders == orders and len(c.assignments) == orders
    assert sum(s == "idle" for s in c.robot_status.values()) == 2-orders


def test_occupied_rack_is_not_given_to_another_robot():
    model, engine = build(racks={"occupied": (0., 6.)})
    finish(model, engine)
    assert "R01" not in model.controller.assignments
    assert model.controller.assignments["R02"]["rack"] == "occupied"
    assert model.controller.robot_status["R02"] == "at_rack"
    assert len(model.controller.pending_orders) == 99


def test_collision_stop_replay():
    model, engine = build(collision=True)
    finish(model, engine)
    assert not model.collision_history
    assert all(r.phase == "Wait" for r in model.robots)
    assert all(s.phase == "Idle" for s in model.sensors)


def test_planned_route_uses_existing_arcs_and_expands_pass_through_points():
    model, engine = build(orders=1)
    engine.execute_one_step()
    path = model.controller.roadmap.paths["R01"].vertices
    assert path == ((0., 0.), (2., 0.), (4., 0.), (6., 0.))
    assert all(a[0] == b[0] or a[1] == b[1] for a, b in zip(path, path[1:]))
