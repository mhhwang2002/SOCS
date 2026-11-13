import math

import pytest

from socs.devs import CoupledDEVS, DEVS
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import ExecutionResult, TESEnginBase, TESEnginThread


@pytest.mark.parametrize(
    "devs_time, animation_interval, continuous_interval, expected",
    [
        (5, 5, math.inf, [("null", 5), ("animation", 5), ("devs", 5)]),
        (5, math.inf, math.inf, [("null", 5), ("devs", 5)]),
        (math.inf, math.inf, 5, [("null", 5)]),
        (5, 5, 5, [("null", 5), ("animation", 5), ("devs", 5)]),
    ],
)
def test_engine_updates_time_once_before_due_events(
    devs_time, animation_interval, continuous_interval, expected,
):
    calls = []

    class Model(PeriodicModel):
        def Delta_null(self, t):
            calls.append(("null", t))
            return super().Delta_null(t)

    class Engine(TESEnginBase):
        def doAnimation(self):
            calls.append(("animation", self.tNow))

        def executeDEVS_star(self, t):
            calls.append(("devs", t))
            return super().executeDEVS_star(t)

    engine = Engine(
        Model("model", devs_time),
        TimeIntervalAnimation=animation_interval,
        TimeIntervalContinuous=continuous_interval,
    )
    engine.bVerbose = False
    calls.clear()
    while engine.PickNextExecutionTime_Type()[0] == 5:
        engine.executeOneStep()
    assert calls == expected


def test_same_time_inputs_and_zero_delay_event_share_update_after_restore():
    calls = []

    class Model(SinkModel):
        def Delta_null(self, t):
            calls.append(t)
            return super().Delta_null(t)

        def delta_y(self):
            self.interval = math.inf

    model = Model("model", reschedule=True, interval=0)
    engine = TESEnginBase(model)
    engine.bVerbose = False
    engine.XBuffer.append((model, PortValue("in", 1)))
    engine.XBuffer.append((model, PortValue("in", 2)))
    assert engine.executeOneStep() == ExecutionResult.ExternalEventOccurred
    snapshot = engine.capture_snapshot()
    engine.executeOneStep()
    engine.restore_snapshot(snapshot)
    assert engine.executeOneStep() == ExecutionResult.ExternalEventOccurred
    assert engine.executeOneStep() == ExecutionResult.ScheduledEventOccurred
    assert model.received == [1, 2]
    assert calls == []


def test_coupled_null_reschedule_invalidates_previously_selected_event():
    class Model(PeriodicModel):
        def Delta_null(self, t):
            if t == 5:
                self.tNext = 7
                return True
            return False

    root = CoupledDEVS("root")
    nested = CoupledDEVS("nested")
    child = Model("child", 5)
    root.add_child(nested)
    nested.add_child(child)
    engine = TESEnginBase(root)
    engine.bVerbose = False

    assert engine.executeOneStep() == ExecutionResult.UnscheduledEventOccurred
    assert child.state == 0
    assert root.tNext == 7
    assert engine.executeOneStep() == ExecutionResult.ScheduledEventOccurred
    assert engine.tNow == 7
    assert child.state == 1


def test_finite_time_bound_updates_model_once_without_executing_due_transition():
    calls = []

    class Model(PeriodicModel):
        def Delta_null(self, t):
            calls.append(t)
            return super().Delta_null(t)

    model = Model("model", 5)
    engine = TESEnginBase(model, TimeSimulationEnd=5)
    assert engine.executeOneStep() == ExecutionResult.TimeBoundReached
    assert engine.executeOneStep() == ExecutionResult.TimeBoundReached
    assert calls == [5]
    assert model.state == 0
    assert sum(model.StatePercentage.values()) == 5


def test_coupled_statistics_include_passive_child_and_same_time_events_once():
    root = CoupledDEVS("root")
    first = PeriodicModel("first", 5)
    second = PeriodicModel("second", 5)
    passive = PeriodicModel("passive", math.inf)
    root.addComponents(first, second, passive)
    engine = TESEnginBase(root, TimeIntervalAnimation=5, TimeSimulationEnd=7)
    engine.bVerbose = False
    assert engine.executeOneStep() == ExecutionResult.NoEventOccurred
    assert all(child.StatePercentage == {} for child in root.M)
    assert engine.executeOneStep() == ExecutionResult.ScheduledEventOccurred
    assert engine.executeOneStep() == ExecutionResult.ScheduledEventOccurred
    assert first.StatePercentage == second.StatePercentage == {"0": 5}
    assert passive.StatePercentage == {}
    assert engine.executeOneStep() == ExecutionResult.TimeBoundReached
    assert first.StatePercentage == second.StatePercentage == {"0": 5, "1": 2}
    assert passive.StatePercentage == {"0": 7}


def test_external_event_without_rescheduling_closes_statistics_interval():
    class Model(PeriodicModel):
        def delta_x(self, event):
            self.state = event.value
            return False

    model = Model("model", 10)
    engine = TESEnginBase(model, TimeIntervalAnimation=3, TimeSimulationEnd=5)
    engine.bVerbose = False
    engine.executeOneStep()
    assert model.StatePercentage == {}
    engine.XBuffer.append((model, PortValue("change", 1)))
    assert engine.executeOneStep() == ExecutionResult.ExternalEventOccurred
    assert (model.tLast, model.tNext) == (0, 10)
    assert model.StatePercentage == {"0": 3}
    assert engine.executeOneStep() == ExecutionResult.TimeBoundReached
    assert model.StatePercentage == {"0": 3, "1": 2}


def test_nested_output_routes_by_model_even_when_port_names_match():
    root = CoupledDEVS("root")
    nested = CoupledDEVS("nested")
    source = SourceModel("source")
    unrelated = SourceModel("unrelated")
    sink = SinkModel("sink")
    root.addComponents(nested, unrelated, sink)
    nested.add_child(source)
    nested.addInputPorts("out")
    nested.addOutputPorts("out")
    root.addOutputPorts("out")
    nested.addCouple((source, "out"), (nested, "out"))
    root.addCouple((nested, "out"), (root, "out"))
    root.addCouple((nested, "out"), (sink, "in"))
    root.set_init_total_state(0)

    outputs = root.Delta_y(1) + root.Delta_y(1)

    assert outputs == [PortValue("out", 42)]
    assert sink.received == [42]


class PeriodicModel(DEVS):
    def __init__(self, name, interval):
        super().__init__(name)
        self.interval = interval
        self.state = 0

    def set_initial_state(self):
        self.state = 0

    def tau(self):
        return self.interval if self.state == 0 else math.inf

    def delta_x(self, x):
        return False

    def delta_y(self):
        self.state += 1
        return None

    def set_state(self, state):
        self.state = state

    def get_state(self):
        return self.state


class SourceModel(DEVS):
    def __init__(self, name):
        super().__init__(name)
        self.addOutputPorts("out")
        self.state = 0

    def set_initial_state(self):
        self.state = 0

    def tau(self):
        return 1.0 if self.state == 0 else math.inf

    def delta_x(self, x):
        return False

    def delta_y(self):
        self.state = 1
        return PortValue(self.OP["out"], 42)

    def set_state(self, state):
        self.state = state

    def get_state(self):
        return self.state


class SinkModel(DEVS):
    def __init__(self, name, reschedule=False, interval=math.inf):
        super().__init__(name)
        self.addInputPorts("in")
        self.reschedule = reschedule
        self.interval = interval
        self.received = []

    def set_initial_state(self):
        self.received = []

    def tau(self):
        return self.interval

    def delta_x(self, event):
        self.received.append(event.value)
        return self.reschedule

    def delta_y(self):
        return None

    def set_state(self, state):
        self.received = state

    def get_state(self):
        return self.received


def make_engine(root):
    engine = TESEnginThread(root)
    engine.sfactor = math.inf
    engine.bVerbose = False
    return engine


def test_engine_constructor_initializes_atomic_models():
    root = CoupledDEVS("System")
    first = PeriodicModel("First", 2.0)
    second = PeriodicModel("Second", 5.0)
    root.add_child(first)
    root.add_child(second)

    engine = make_engine(root)

    assert first.state == 0
    assert second.state == 0
    assert first.tNext == 2.0
    assert second.tNext == 5.0
    engine.terminate()


def test_infinite_time_advance_is_not_a_finite_event():
    root = CoupledDEVS("System")
    passive = PeriodicModel("Passive", math.inf)
    root.add_child(passive)

    engine = make_engine(root)

    assert math.isinf(passive.tNext)
    assert root.next_event() is None
    engine.terminate()


def test_execute_one_step_routes_output_to_connected_model():
    root = CoupledDEVS("System")
    source = SourceModel("Source")
    sink = SinkModel("Sink")
    root.add_child(source)
    root.add_child(sink)
    root.addCouple((source, "out"), (sink, "in"))

    engine = make_engine(root)
    try:
        executionResult = engine.executeOneStep()
 
        assert executionResult == ExecutionResult.ScheduledEventOccurred
        assert sink.received == [42]
    finally:
        engine.terminate()


def test_external_input_without_rescheduling_preserves_schedule():
    root = CoupledDEVS("System")
    source = SourceModel("Source")
    sink = SinkModel("Sink", reschedule=False, interval=4.0)
    root.add_child(source)
    root.add_child(sink)
    root.addCouple((source, "out"), (sink, "in"))
    engine = make_engine(root)

    try:
        original_time = sink.tNext
        engine.executeOneStep()

        assert sink.received == [42]
        assert sink.tNext == original_time
    finally:
        engine.terminate()


def test_external_input_with_rescheduling_uses_current_time():
    root = CoupledDEVS("System")
    source = SourceModel("Source")
    sink = SinkModel("Sink", reschedule=True, interval=4.0)
    root.add_child(source)
    root.add_child(sink)
    root.addCouple((source, "out"), (sink, "in"))
    engine = make_engine(root)

    try:
        engine.executeOneStep()

        assert sink.received == [42]
        assert sink.tNext == 5.0
        assert sink.tLast == 1.0
    finally:
        engine.terminate()


def test_simultaneous_internal_events_are_processed_on_successive_steps():
    root = CoupledDEVS("System")
    first = PeriodicModel("First", 2.0)
    second = PeriodicModel("Second", 2.0)
    root.add_child(first)
    root.add_child(second)
    engine = make_engine(root)

    try:
        engine.executeOneStep()
        assert first.state + second.state == 1

        engine.executeOneStep()
        assert first.state == 1
        assert second.state == 1
        assert math.isinf(first.tNext)
        assert math.isinf(second.tNext)
    finally:
        engine.terminate()
