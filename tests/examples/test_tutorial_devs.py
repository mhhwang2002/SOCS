import pytest

from examples.DEVS_Simple.DEVSCounter import Counter
from examples.DEVS_Simple.DEVSphCounter import DEVSphCounter
from examples.DEVS_Simple.DEVSphCounterBasic import DEVSphCounterBasic
from examples.DEVS_Simple.DEVSPingPong import make_model as make_devs_model
from examples.DEVS_Simple.DEVSphPingPong import make_model
from examples.DEVS_Simple.DEVSToaster import Toaster
from examples.DEVS_Simple.DEVSphToaster import DEVSphToaster, ToasterEngine
from socs.devs.TESBase import PortValue


@pytest.mark.parametrize("counter_type", [Counter, DEVSphCounter, DEVSphCounterBasic])
def test_tutorial_counters_produce_same_trace(counter_type):
    counter = counter_type("counter", 3)
    counter.set_init_total_state(0)
    for time, count, total, output in [(5, 1, 1, 0), (10, 2, 3, 1), (15, 3, 6, 3)]:
        assert counter.tNext == time
        counter.Delta_t(time)
        assert counter.Delta_y(time) == PortValue("!c", output)
        assert counter.display_state() == ("Done" if count == 3 else "Count", count, total)
    assert counter.tNext == float("inf")


@pytest.mark.parametrize("toaster_type", [Toaster, DEVSphToaster])
def test_tutorial_toasters_ignore_repeat_push_and_pop_at_default_time(toaster_type):
    toaster = toaster_type("toaster")
    engine = ToasterEngine(toaster, animation_interval=1)
    engine.push()
    assert toaster.phase == "Toasting"
    for _ in range(3):
        engine.execute_one_step()
    assert engine.tNow == 3
    engine.push()
    assert toaster.tNext == 10
    for _ in range(8):
        engine.execute_one_step()
    assert engine.tNow == 10
    assert toaster.phase == "Idle"
    assert engine.last_output == PortValue("!pop")


def test_phase_toaster_does_not_draw_ignored_push_as_self_loop():
    toaster = DEVSphToaster("toaster")
    engine = ToasterEngine(toaster)
    engine.push()
    engine.push()
    assert toaster.phase == "Toasting"
    assert toaster.tNext == 10
    assert toaster.graph.edges("Toasting", "Toasting") == []
    assert "p1 -> p1" not in toaster.to_dot()


@pytest.mark.parametrize("factory", [make_devs_model, make_model])
def test_tutorial_ping_pong_couplings_produce_same_boundary_outputs(factory):
    model = factory()
    model.set_init_total_state(0)
    for time, port in [(0.1, "!ping"), (0.3, "!pong"), (0.4, "!ping"), (0.6, "!pong")]:
        next_time = model.tNext
        assert next_time == pytest.approx(time)
        model.Delta_t(next_time)
        assert model.Delta_y(next_time) == [PortValue(port)]
