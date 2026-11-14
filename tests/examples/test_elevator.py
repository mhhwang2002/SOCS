import pytest

from examples.DEVS_Simple.cDEVSphElevator import Elevator, request_floor
from socs.devs.TESEngin import ExecutionResult, TESEnginBase


@pytest.mark.parametrize("animation_interval", [float("inf"), 0.05])
def test_elevator_moves_both_directions_with_or_without_animation(animation_interval):
    model = Elevator()
    engine = TESEnginBase(model, TimeIntervalAnimation=animation_interval)
    engine.bVerbose = False
    for floor, expected in (
        (2, [("Cruising", 0.375, 1.5), ("Braking", 2.625, 1.5), ("Idle", 3, 0)]),
        (1, [("Cruising", 2.625, -1.5), ("Braking", 0.375, -1.5), ("Idle", 0, 0)]),
    ):
        request_floor(engine, floor)
        for phase, position, velocity in expected:
            for _ in range(100):
                if engine.execute_one_step() == ExecutionResult.ScheduledEventOccurred:
                    break
            assert model.phase == phase
            assert model._variable.position == pytest.approx(position)
            assert model._variable.velocity == pytest.approx(velocity)
        assert model._variable.floor == floor
    assert engine.tNow == pytest.approx(5)


def test_elevator_advances_to_finite_bound_and_restores_continuous_state():
    model = Elevator()
    engine = TESEnginBase(model, TimeSimulationEnd=0.25)
    engine.bVerbose = False
    request_floor(engine, 2)
    assert engine.execute_one_step() == ExecutionResult.TimeBoundReached
    assert model._variable.position == pytest.approx(0.09375)
    assert model._variable.velocity == pytest.approx(0.75)
    snapshot = engine.capture_snapshot()
    engine.set_simulation_time_bound(3)
    engine.execute_one_step()
    engine.restore_snapshot(snapshot)
    assert model.phase == "Accelerating"
    assert model._variable.position == pytest.approx(0.09375)
    assert engine.execute_one_step() == ExecutionResult.ScheduledEventOccurred
    assert model._variable.position == pytest.approx(0.375)


def test_elevator_evaluates_elapsed_from_schedule_origin_without_double_counting():
    model = Elevator()
    engine = TESEnginBase(model, TimeIntervalAnimation=0.1)
    engine.bVerbose = False
    request_floor(engine, 2)
    engine.execute_one_step()
    assert model._variable.position == pytest.approx(0.015)
    engine.execute_one_step()
    assert model.tLast == 0
    assert model._variable.position == pytest.approx(0.06)
    assert model._variable.velocity == pytest.approx(0.6)
