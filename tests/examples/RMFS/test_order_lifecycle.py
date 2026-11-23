from math import inf, isfinite

import pytest

from socs.devs.TESEngin import TESEnginBase
from examples.RMFS.DEVSModelCreator import create_model
from examples.RMFS.GridPath import make_grid


def test_thirty_dedicated_outer_homes_have_connected_nonoverlapping_bays():
    from math import dist
    from examples.RMFS.WarehouseLayout import load_layout
    original = load_layout()
    expanded = load_layout(home_capacity=30)
    homes = expanded['warehouse_homes']
    assert len(homes) == len(set(homes)) == 30
    assert homes[:10] == original['warehouse_homes']
    assert all(len(expanded['adjacency'][p]) == 1 for p in homes)
    assert all(dist(a,b) > 1.5 for i,a in enumerate(homes) for b in homes[i+1:])
    assert expanded['warehouse_racks'] == original['warehouse_racks']
    assert expanded['warehouse_stations'] == original['warehouse_stations']
    visited = {homes[0]}
    pending = [homes[0]]
    while pending:
        for point in expanded['adjacency'][pending.pop()]:
            if point not in visited:
                visited.add(point)
                pending.append(point)
    assert visited == set(expanded['control_points'])


def build(orders=4, collision=False):
    model = create_model(robot_count=2, order_count=orders, seed=0, full_cycle=True,
        layout=make_grid(), robot_starts={"R01": (0, 0), "R02": (0, 3)},
        rack_positions={"A": (6., 0.), "B": (6., 6.)},
        workstations={"W01": (6., 2.), "W02": (6., 4.)}, collision_check=collision)
    engine = TESEnginBase(model, TimeIntervalAnimation=.1 if collision else inf)
    engine.bVerbose = False
    return model, engine


def finish(model, engine):
    steps = 0
    while isfinite(model.tNext):
        engine.execute_one_step()
        steps += 1
        assert steps < 20000


def check_finished(model, engine, count):
    c = model.controller
    assert c.completed_orders == c.shipped_orders == c.received_orders == count
    assert not c.pending_orders and not c.assignments and not c.rack_reservations
    assert not c.station_reservations and not c.work_events and not c.service_commands
    assert set(c.robot_status.values()) == {"home"}
    assert sum(c.robot_counts.values()) == count
    for robot in model.robots:
        assert robot.phase == "Wait" and not robot._variable.lift_up
        assert robot._variable.rack is None
        assert robot.position_at(engine.tNow) == c.initial_paths[robot.name].vertices[0]
    assert all(sensor.phase == "Idle" for sensor in model.sensors)


def test_shipping_return_home_and_real_handling_durations():
    model, engine = build()
    finish(model, engine)
    check_finished(model, engine, 4)
    h = model.controller.order_history
    assert [e["order"] for e in h if e["event"] == "assigned"] == [1, 2, 3, 4]
    for order in range(1, 5):
        events = {e["event"]: e for e in h if e.get("order") == order}
        assert events["lifted"]["time"]-events["station_reserved"]["time"] == pytest.approx(1.4)
        assert events["shipped"]["time"]-events["service_started"]["time"] == pytest.approx(3.)
        assert events["returned"]["time"]-events["rack_return_arrival"]["time"] == pytest.approx(1.4)
        assert events["shipped"]["time"] < events["returned"]["time"]
    # Every movement is planned at its own stage event, not at initial dispatch.
    station_plans = [e for e in h if e["event"] == "leg_planned" and e["leg"] == "to_station"]
    for e in station_plans:
        lifted = next(x for x in h if x["event"] == "lifted" and x["order"] == e["order"])
        assert e["time"] >= lifted["time"] and e["loaded"]
    for e in [x for x in h if x["event"] == "leg_planned" and x["leg"] == "returning"]:
        shipped = next(x for x in h if x["event"] == "shipped" and x["order"] == e["order"])
        assert e["time"] >= shipped["time"] and e["loaded"]
    assert model.controller.path_planner.roadmap is model.controller.traffic_manager.roadmap


def test_lifecycle_snapshot_during_lift_and_service_and_reset():
    model, engine = build(orders=3)
    snapshots = {}
    while isfinite(model.tNext):
        engine.execute_one_step()
        statuses = model.controller.robot_status.values()
        for status in ("lifting", "processing", "lowering"):
            if status in statuses and status not in snapshots:
                snapshots[status] = engine.capture_snapshot()
    check_finished(model, engine, 3)
    assert set(snapshots) == {"lifting", "processing", "lowering"}
    end, state = engine.tNow, model.controller.get_snapshot()
    for snapshot in snapshots.values():
        engine.restore_snapshot(snapshot)
        finish(model, engine)
        assert engine.tNow == end and model.controller.get_snapshot() == state
    engine.reset()
    finish(model, engine)
    assert engine.tNow == end and model.controller.get_snapshot() == state


def test_collision_stop_validation_replay():
    model, engine = build(collision=True)
    finish(model, engine)
    check_finished(model, engine, 4)
    assert not model.collision_history


def test_empty_workload_finishes_at_home():
    model, engine = build(orders=0)
    finish(model, engine)
    check_finished(model, engine, 0)
