import pytest

from socs.decision import (
    ControlDecision,
    ControlRuntimeExecutor,
    NoControl,
)
from socs.devs.DEVSInterfaces import PortValue
from socs.devs.SimEngine import SimEngineThread

from examples.LogBridge.LoadPassive_LogBridge import makeModel


def make_engine():
    engine = SimEngineThread(makeModel())
    engine.bVerbose = False
    return engine


def test_runtime_executor_resolves_target_path_and_input_port():
    engine = make_engine()
    target = next(child for child in engine.model.children if child.name == "L11")

    result = ControlRuntimeExecutor().execute(
        engine,
        ControlDecision("Load_LogBridge.L11", "?go"),
    )

    assert result is engine
    assert len(engine.XBuffer) == 1
    queued_target, queued_event = engine.XBuffer[0]
    assert queued_target is target
    assert queued_event == PortValue(target.IP["?go"])


def test_runtime_executor_resolves_port_value_pair():
    engine = make_engine()
    target = next(child for child in engine.model.children if child.name == "L11")

    ControlRuntimeExecutor().execute(
        engine,
        ControlDecision("Load_LogBridge.L11", ("?in", 7.5)),
    )

    queued_target, queued_event = engine.XBuffer[0]
    assert queued_target is target
    assert queued_event == PortValue(target.IP["?in"], 7.5)


def test_no_control_does_not_queue_an_external_input():
    engine = make_engine()

    result = ControlRuntimeExecutor().execute(engine, NoControl())

    assert result is engine
    assert not engine.XBuffer


def test_runtime_executor_rejects_unknown_target_model():
    engine = make_engine()

    with pytest.raises(ValueError, match="does not exist"):
        ControlRuntimeExecutor().execute(
            engine,
            ControlDecision("Load_LogBridge.Missing", "?go"),
        )

