"""Real DEVS controller/plant execution, not geometric planner replay."""

from importlib import import_module
from math import inf, isfinite

import pytest

from socs.devs.TESEngin import TESEnginBase

from examples.RMFS.GridPath import CONTROL_POINTS
from examples.RMFS.MRobot import MRobot
from examples.RMFS.controlled import create_controlled_model


FEASIBLE = (
    "02_01_RobotPath_Crossing_NoControl", "02_02_RobotPath_LeftEdge_NoControl",
    "02_03_RobotPath_SameDirection_Collision", "02_04_RobotPath_SameDirection_Clear",
    "02_06_RobotPath_OppositeDirection_Clear", "02_07_RobotPath_OppositeDirection_EntryWait",
    "02_08_RobotPath_DisjointSweeps", "02_09_RobotPath_FirstMoverMatters",
    "02_10_RobotPath_AlternatingProgress", "02_11_RobotPath_SharedCorner",
    "03_02_RobotPath_ThreeFollowing_Collision", "03_04_RobotPath_SweepInterference",
    "03_05_RobotPath_LocalSafe_Deadlock",
)


def build(example, interval=inf, *, lift_up=False):
    source = import_module("examples.RMFS.Tests." + example)
    model, robots, sensors = create_controlled_model(
        source.PATHS, initial_headings=getattr(source, "INITIAL_HEADINGS", None),
        lift_up=lift_up, collision_check=interval != inf)
    engine = TESEnginBase(model, TimeIntervalAnimation=interval)
    engine.bVerbose = False
    return source.PATHS, model, robots, sensors, engine


def finish(model, engine):
    count = 0
    while isfinite(model.tNext):
        engine.execute_one_step()
        count += 1
        assert count < 100000, "Unexpected non-termination in finite tuple execution."
    return count


def check_trace(model):
    manager = model.controller.traffic_manager
    assert manager.done and not manager.awaiting and not manager.command_buffer
    assert manager.completed_stages == list(range(len(manager.tuple_path.stages)))
    records = model.controller.roadmap.history
    commands = {}
    for index, event in enumerate(records):
        if event["event"] in ("approved", "issued", "arrived"):
            commands.setdefault(event["command_id"], []).append((index, event))
    for events in commands.values():
        assert [event["event"] for _, event in events] == ["approved", "issued", "arrived"]
        assert events[2][1]["time"] > events[1][1]["time"]
    completed_at = {x["stage"]: x["time"] for x in manager.stage_history
                    if x["event"] == "completed"}
    for event in records:
        if event["event"] == "approved" and event["stage"]:
            assert event["time"] >= completed_at[event["stage"] - 1]


@pytest.mark.parametrize("example", FEASIBLE)
@pytest.mark.parametrize("interval", (inf, .01), ids=("DEVS-only", "collision-sampling"))
def test_all_feasible_original_routes_execute_on_real_plant(example, interval):
    routes, model, robots, sensors, engine = build(example, interval)
    finish(model, engine)
    assert not model.collision_history
    check_trace(model)
    for robot, sensor in zip(robots, sensors):
        assert robot.phase == "Wait" and sensor.phase == "Idle"
        expected = CONTROL_POINTS[routes[robot.name][-1]]
        assert robot.position_at(engine.tNow) == expected
        record = model.controller.roadmap.robots[robot.name]
        assert record["reported_position"] == expected
        assert record["approved_path"] is None
        assert not sensor._variable.pending_events


def test_approval_and_emission_do_not_teleport_reported_or_physical_position():
    routes, model, robots, _, engine = build("02_10_RobotPath_AlternatingProgress")
    manager = model.controller.traffic_manager
    assert manager.command_buffer
    for command in manager.command_buffer:
        record = manager.roadmap.robots[command["robot"]]
        assert record["command_state"] == "approved"
        assert record["reported_position"] == CONTROL_POINTS[routes[command["robot"]][0]]
    engine.execute_one_step()  # controller emits commands at t=0
    assert engine.tNow == 0
    for robot in robots:
        assert robot.position_at(0) == CONTROL_POINTS[routes[robot.name][0]]
    finish(model, engine)
    check_trace(model)


def test_simultaneous_commands_wait_for_last_arrival_and_preserve_rotation():
    routes = {"R1": ((0, 0), (1, 0)), "R2": ((0, 3), (3, 3))}
    model, _, _ = create_controlled_model(routes, initial_headings={"R1": 0, "R2": 180})
    engine = TESEnginBase(model)
    engine.bVerbose = False
    finish(model, engine)
    check_trace(model)
    history = model.controller.roadmap.history
    issued = [x for x in history if x["event"] == "issued"]
    arrivals = [x for x in history if x["event"] == "arrived"]
    assert issued[0]["time"] == issued[1]["time"] == 0
    assert arrivals[0]["time"] < arrivals[1]["time"]
    assert any(x["event"] == "rotation_started" for x in history)
    assert any(x["event"] == "rotation_finished" for x in history)
    # Early completion cannot advance the tuple while the second is rotating/moving.
    stage = issued[0]["stage"]
    completed = next(x for x in model.controller.traffic_manager.stage_history
                     if x["stage"] == stage and x["event"] == "completed")
    assert completed["time"] == arrivals[-1]["time"]


def test_loaded_plant_and_reset_are_deterministic():
    _, model, _, _, engine = build("03_04_RobotPath_SweepInterference", .01, lift_up=True)
    finish(model, engine)
    first_time = engine.tNow
    first_history = model.controller.roadmap.get_snapshot()
    engine.reset()
    finish(model, engine)
    assert engine.tNow == first_time
    assert model.controller.roadmap.get_snapshot() == first_history
    assert not model.collision_history


def test_midrun_system_snapshot_restores_controller_blackboard_buffer_and_plant():
    _, model, _, _, engine = build("03_04_RobotPath_SweepInterference")
    while engine.tNow < 3:
        engine.execute_one_step()
    snapshot = engine.capture_snapshot()
    finish(model, engine)
    final_time = engine.tNow
    final_state = model.controller.get_snapshot()
    engine.restore_snapshot(snapshot)
    finish(model, engine)
    assert engine.tNow == final_time
    assert model.controller.get_snapshot() == final_state


@pytest.mark.parametrize("example", (
    "02_05_RobotPath_OppositeDirection_Collision", "03_01_RobotPath_ThreeWay_Collision",
    "03_03_RobotPath_MixedDirection_Collision", "03_06_RobotPath_ThreeWay_CyclicWait"))
def test_no_plan_is_rejected_before_plant_creation(example):
    routes = import_module("examples.RMFS.Tests." + example).PATHS
    with pytest.raises(ValueError, match="No wait-duplication"):
        create_controlled_model(routes)


def test_metric_commands_do_not_depend_on_roadmap_ids():
    from socs.devs.TESBase import PortValue
    robot = MRobot("R", (0, 0), lift_up=False, command_coordinates="metres")
    robot.set_init_total_state(0)
    robot.Delta_x(PortValue("?command", ((0., 0.), (1.25, 0.))), 0)
    while isfinite(robot.tNext):
        robot.Delta_y(robot.tNext)
    assert robot.phase == "Wait"
    assert robot.position_at(robot.tLast) == (1.25, 0.)


def test_five_frogs_all_interfere_satisfy_theorem_and_complete():
    from examples.RMFS.path_geometry import (Path, PathPosition, RectangularFootprint,
                                           Subpath, interference_2d, sweep_2d)
    example = "05_01_RobotPath_TheoremSafeEndpoints"
    source = import_module("examples.RMFS.Tests." + example)
    paths = {name: Path(name, tuple(CONTROL_POINTS[p] for p in route))
             for name, route in source.PATHS.items()}
    sweeps = {name: sweep_2d(Subpath(path, PathPosition(0),
                                   PathPosition(len(path.vertices) - 1)))
              for name, path in paths.items()}
    expected = {"R1": {"R2", "R3"}, "R2": {"R1", "R3"}, "R3": {"R1", "R2"},
                "R4": {"R5"}, "R5": {"R4"}}
    for name, path in paths.items():
        assert len(set(path.vertices)) == len(path.vertices)
        data = interference_2d(path, RectangularFootprint(),
                               {k: (p, RectangularFootprint())
                                for k, p in paths.items() if k != name})
        assert data.intervals
        assert not data.contains(PathPosition(0))
        assert not data.contains(PathPosition(len(path.vertices) - 1))
        neighbors = {other for other in paths if other != name and
                     sweeps[name].intersects(sweeps[other])}
        assert neighbors == expected[name]
    routes, model, robots, sensors, engine = build(example, .01)
    finish(model, engine)
    check_trace(model)
    assert not model.collision_history
    assert engine.tNow == pytest.approx(58.589871045)
    for robot, sensor in zip(robots, sensors):
        assert robot.phase == "Wait" and sensor.phase == "Idle"
        assert robot.position_at(engine.tNow) == CONTROL_POINTS[routes[robot.name][-1]]


def test_eight_frogs_reach_capacity_with_safe_endpoints():
    from examples.RMFS.path_geometry import (Path, PathPosition, RectangularFootprint,
                                           interference_2d)
    example = "08_01_RobotPath_TheoremCapacity"
    source = import_module("examples.RMFS.Tests." + example)
    paths = {name: Path(name, tuple(CONTROL_POINTS[p] for p in route))
             for name, route in source.PATHS.items()}
    endpoints = [point for route in source.PATHS.values() for point in route]
    assert len(paths) == 8 and len(set(endpoints)) == 16
    assert set(endpoints) == set(CONTROL_POINTS)
    for name, path in paths.items():
        data = interference_2d(path, RectangularFootprint(),
                               {k: (p, RectangularFootprint())
                                for k, p in paths.items() if k != name})
        assert not data.intervals
    routes, model, robots, sensors, engine = build(example, .01)
    finish(model, engine)
    check_trace(model)
    assert not model.collision_history
    issued = [e for e in model.controller.roadmap.history if e["event"] == "issued"]
    assert len(issued) == 8 and all(e["time"] == 0 for e in issued)
    for robot, sensor in zip(robots, sensors):
        assert robot.phase == "Wait" and sensor.phase == "Idle"
        assert robot.position_at(engine.tNow) == CONTROL_POINTS[routes[robot.name][-1]]
