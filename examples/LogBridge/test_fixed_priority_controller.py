"""Tests for the fixed-priority LogBridge controller."""

import pytest

from socs.devs.TESEngin import TESEnginThread
from socs.devs.TESBase import PortValue
from .LoadPassive_LogBridge import make_model
from .FixedPriorityController import FixedPriorityController
from .LoadPassive import LoadPassive
from .LogBridge import LogBridge


def get_components(model):
    loads = {item.name: item for item in model.children if isinstance(item, LoadPassive)}
    bridge = next(item for item in model.children if isinstance(item, LogBridge))
    return loads, bridge


def make_engine(model):
    engine = TESEnginThread(model)
    engine.bVerbose = False
    engine.set_simulation_time_bound(200)
    return engine

def test_fixed_priority_selects_l11_first():
    model = make_model()
    loads, bridge = get_components(model)
    engine = make_engine(model)
    controller = FixedPriorityController()

    loads["L11"].phase = "W"
    loads["L21"].phase = "W"

    selected = controller.dispatch(engine, model)

    assert selected is loads["L11"]
    assert len(engine.XBuffer) == 1
    assert controller.dispatch_history == [(0.0, "L11")]


def test_selects_l21_when_l11_is_not_waiting():
    model = make_model()
    loads, _ = get_components(model)
    engine = make_engine(model)
    controller = FixedPriorityController()

    loads["L21"].phase = "W"

    assert controller.dispatch(engine, model) is loads["L21"]


def test_does_not_dispatch_when_bridge_is_busy():
    model = make_model()
    loads, bridge = get_components(model)
    engine = make_engine(model)
    controller = FixedPriorityController()

    loads["L11"].phase = "W"
    bridge.Phase = "S1"

    assert controller.dispatch(engine, model) is None
    assert not engine.XBuffer


def test_does_not_dispatch_when_a_load_is_already_in_service_phase():
    model = make_model()
    loads, _ = get_components(model)
    engine = make_engine(model)
    controller = FixedPriorityController()

    loads["L11"].phase = "S"
    loads["L21"].phase = "W"

    assert controller.dispatch(engine, model) is None
    assert not engine.XBuffer


def test_does_not_dispatch_when_external_input_is_pending():
    model = make_model()
    loads, _ = get_components(model)
    engine = make_engine(model)
    controller = FixedPriorityController()

    loads["L11"].phase = "W"
    engine.inject(loads["L21"], PortValue(loads["L21"].IP["?go"]))

    assert controller.dispatch(engine, model) is None
    assert len(engine.XBuffer) == 1


def test_rejects_duplicate_priority_names():
    with pytest.raises(ValueError):
        FixedPriorityController(("L11", "L11"))


def test_automatic_run_dispatches_in_priority_order_without_collision():
    from .LoadPassive_LogBridge import run_automatic_simulation

    model = make_model()
    engine, controller = run_automatic_simulation(model, end_time=100.0)
    _, bridge = get_components(model)

    assert controller.dispatch_history == [(20.0, "L11"), (40.0, "L21")]
    assert bridge.Phase != "C"
    assert engine.tNow == 100.0
