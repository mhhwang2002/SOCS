"""Step-level integration checks for the simple FDDEVS examples."""

from pathlib import Path

import pytest

from socs.devs.DEVSInterfaces import PortValue
from socs.devs.SimEngine import ExecutionResult, SimEngineBase


def make_engine(model):
    """Create a synchronous engine for deterministic one-step checks."""
    engine = SimEngineBase(
        model,
        TimeIntervalContinuous=float("inf"),
        TimeIntervalAnimation=float("inf"),
    )
    engine.bVerbose = False
    return engine


def queue_input(engine, target, port, value=None):
    """Queue an input event for the next synchronous engine step."""
    engine.XBuffer.append((target, PortValue(port, value)))


def test_counter_advances_through_expected_internal_states():
    from examples.FDDEVS_Simple.FDDEVSCounter import Counter

    model = Counter("Counter", count_no=3)
    engine = make_engine(model)

    assert model.get_state() == ("C", 1, 0, 3)
    assert model.timeNext() == 5.0

    assert engine.executeOneStep() is ExecutionResult.ScheduledEventOccurred
    assert model.get_state() == ("C", 2, 1, 3)
    assert model.timeLast() == 5.0
    assert model.timeNext() == 10.0

    assert engine.executeOneStep() is ExecutionResult.ScheduledEventOccurred
    assert model.get_state() == ("C", 3, 3, 3)
    assert model.timeLast() == 10.0
    assert model.timeNext() == 15.0

    assert engine.executeOneStep() is ExecutionResult.ScheduledEventOccurred
    assert model.get_state() == ("D", 4, 6, 3)
    assert model.timeLast() == 15.0
    assert model.timeNext() == float("inf")


def test_toaster_external_and_internal_transitions():
    from examples.FDDEVS_Simple.FDDEVSToaster import Toaster

    model = Toaster("T1", tt=2.5)
    engine = make_engine(model)

    queue_input(engine, model, model.IP["?push"])
    assert engine.executeOneStep() is ExecutionResult.ExternalEventOccurred
    assert model.get_state() == "T"
    assert model.timeLast() == 0.0
    assert model.timeNext() == 2.5

    assert engine.executeOneStep() is ExecutionResult.ScheduledEventOccurred
    assert model.get_state() == "I"
    assert model.timeLast() == 2.5
    assert model.timeNext() == float("inf")


def test_two_slot_toaster_routes_inputs_and_completion_events(monkeypatch):
    example_dir = Path(__file__).resolve().parents[2] / "examples" / "FDDEVS_Simple"
    monkeypatch.syspath_prepend(str(example_dir))

    from FDDEVSTwoSlotToaster import makeModel

    model = makeModel(tt1=2.0, tt2=4.0)
    toaster1, toaster2 = model.children
    engine = make_engine(model)

    queue_input(engine, model, model.IP["?push1"])
    assert engine.executeOneStep() is ExecutionResult.ExternalEventOccurred
    assert (toaster1.get_state(), toaster2.get_state()) == ("T", "I")
    assert model.timeNext() == 2.0

    queue_input(engine, model, model.IP["?push2"])
    assert engine.executeOneStep() is ExecutionResult.ExternalEventOccurred
    assert (toaster1.get_state(), toaster2.get_state()) == ("T", "T")
    assert model.timeNext() == 2.0

    assert engine.executeOneStep() is ExecutionResult.ScheduledEventOccurred
    assert (toaster1.get_state(), toaster2.get_state()) == ("I", "T")
    assert model.timeLast() == 2.0
    assert model.timeNext() == 4.0

    assert engine.executeOneStep() is ExecutionResult.ScheduledEventOccurred
    assert (toaster1.get_state(), toaster2.get_state()) == ("I", "I")
    assert model.timeLast() == 4.0
    assert model.timeNext() == float("inf")


def test_ping_pong_routes_each_internal_output_to_the_other_player():
    from examples.FDDEVS_Simple.FDDEVSPingPong import Player, makeModel

    model = makeModel()
    player_a, player_b = model.children
    engine = make_engine(model)

    assert isinstance(player_a, Player)
    assert isinstance(player_b, Player)
    assert (player_a.get_state(), player_b.get_state()) == ("S", "W")

    assert engine.executeOneStep() is ExecutionResult.ScheduledEventOccurred
    assert (player_a.get_state(), player_b.get_state()) == ("W", "S")
    assert model.timeLast() == pytest.approx(0.1)
    assert model.timeNext() == pytest.approx(0.3)

    assert engine.executeOneStep() is ExecutionResult.ScheduledEventOccurred
    assert (player_a.get_state(), player_b.get_state()) == ("S", "W")
    assert model.timeLast() == pytest.approx(0.3)
    assert model.timeNext() == pytest.approx(0.4)
