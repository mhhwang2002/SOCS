"""Tests for the FIFO LogBridge controller."""

from socs.devs.SimEngine import SimEngineThread
from socs.devs.DEVSInterfaces import PortValue

from examples.LogBridge.LoadPassive_LogBridge import makeModel
from examples.LogBridge.FIFOController import FIFOController
from examples.LogBridge.LoadPassive import LoadPassive
from examples.LogBridge.LogBridge import LogBridge


def get_components(model):
    loads = {
        item.name: item
        for item in model.children
        if isinstance(item, LoadPassive)
    }
    bridge = next(
        item for item in model.children if isinstance(item, LogBridge)
    )
    return loads, bridge


def make_engine(model):
    engine = SimEngineThread(model)
    engine.bVerbose = False
    engine.set_simulation_time_bound(200)
    return engine


def test_fifo_selects_load_that_entered_waiting_first():
    model = makeModel()
    loads, _ = get_components(model)
    engine = make_engine(model)
    controller = FIFOController()

    # The child order is L11 then L21, but L21 entered W earlier.
    loads["L11"].phase = "W"
    loads["L11"].set_timeLast(15.0)
    loads["L21"].phase = "W"
    loads["L21"].set_timeLast(10.0)

    selected = controller.dispatch(engine, model)

    assert selected is loads["L21"]
    assert len(engine.XBuffer) == 1
    assert controller.dispatch_history == [(0.0, "L21")]


def test_fifo_selects_only_waiting_load():
    model = makeModel()
    loads, _ = get_components(model)
    engine = make_engine(model)
    controller = FIFOController()

    loads["L21"].phase = "W"
    loads["L21"].set_timeLast(12.0)

    assert controller.dispatch(engine, model) is loads["L21"]


def test_fifo_does_not_dispatch_when_bridge_is_busy():
    model = makeModel()
    loads, bridge = get_components(model)
    engine = make_engine(model)
    controller = FIFOController()

    loads["L11"].phase = "W"
    bridge.Phase = "S1"

    assert controller.dispatch(engine, model) is None
    assert not engine.XBuffer


def test_fifo_does_not_dispatch_when_load_is_in_service_phase():
    model = makeModel()
    loads, _ = get_components(model)
    engine = make_engine(model)
    controller = FIFOController()

    loads["L11"].phase = "S"
    loads["L21"].phase = "W"

    assert controller.dispatch(engine, model) is None
    assert not engine.XBuffer


def test_fifo_does_not_dispatch_when_external_input_is_pending():
    model = makeModel()
    loads, _ = get_components(model)
    engine = make_engine(model)
    controller = FIFOController()

    loads["L11"].phase = "W"
    engine.inject(loads["L21"], PortValue(loads["L21"].IP["?go"]))

    assert controller.dispatch(engine, model) is None
    assert len(engine.XBuffer) == 1


def test_fifo_ties_preserve_model_child_order():
    model = makeModel()
    loads, _ = get_components(model)
    engine = make_engine(model)
    controller = FIFOController()

    loads["L11"].phase = "W"
    loads["L11"].set_timeLast(10.0)
    loads["L21"].phase = "W"
    loads["L21"].set_timeLast(10.0)

    assert controller.dispatch(engine, model) is loads["L11"]


def test_automatic_fifo_run_completes_without_collision():
    from examples.LogBridge.LoadPassive_LogBridge import runAutomaticFIFOSimulation

    model = makeModel()
    engine, controller = runAutomaticFIFOSimulation(model, end_time=100.0)
    _, bridge = get_components(model)

    assert controller.dispatch_history == [
        (20.0, "L11"),
        (40.0, "L21"),
        (62.22222222222222, "L12"),
        (84.44444444444444, "L22"),
    ]
    assert bridge.Phase != "C"
    assert engine.tNow == 100.0
