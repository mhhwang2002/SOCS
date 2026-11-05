import pytest

from socs.decision.control import apply_control, make_port_value, resolve_model
from socs.decision.decision import ControlDecision
from socs.devs.DEVSInterfaces import PortValue
from socs.devs.SimEngine import SimEngineThread

from examples.LogBridge.LoadPassive_LogBridge import makeModel


def make_engine():
    engine = SimEngineThread(makeModel())
    engine.bVerbose = False
    return engine


def test_resolves_target_path_and_input_port():
    engine = make_engine()
    target = next(child for child in engine.model.children if child.name == "L11")

    assert resolve_model(engine.model, "Load_LogBridge.L11") is target
    event = make_port_value(target, "?go")
    assert event == PortValue(target.IP["?go"])


def test_resolves_port_value_pair():
    engine = make_engine()
    target = next(child for child in engine.model.children if child.name == "L11")

    event = make_port_value(target, ("?in", 7.5))
    assert event == PortValue(target.IP["?in"], 7.5)


def test_apply_control_queues_external_input():
    engine = make_engine()
    target = next(child for child in engine.model.children if child.name == "L11")

    apply_control(engine, ControlDecision("Load_LogBridge.L11", "?go"))

    assert len(engine.XBuffer) == 1
    queued_target, queued_event = engine.XBuffer[0]
    assert queued_target is target
    assert queued_event == PortValue(target.IP["?go"])


def test_no_control_does_not_queue_an_external_input():
    engine = make_engine()

    apply_control(engine, ControlDecision(None))

    assert not engine.XBuffer


def test_rejects_unknown_target_model():
    engine = make_engine()

    with pytest.raises(ValueError, match="does not exist"):
        apply_control(engine, ControlDecision("Load_LogBridge.Missing", "?go"))
