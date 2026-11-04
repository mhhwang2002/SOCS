import math

from socs.decision import DecisionGenerator, DecisionRuntimeController
from socs.devs.DEVSInterfaces import PortValue
from socs.devs.SimEngine import ExecutionResult, SimEngineThread

from examples.LogBridge.LoadPassive import LoadPassive
from examples.LogBridge.LoadPassive_LogBridge import makeModel
from examples.LogBridge.LogBridge import LogBridge
from examples.LogBridge.decision_generator import LogBridgeDecisionGenerator
from examples.LogBridge.decisions import EnterBridgeDecision


class RecordingGenerator(DecisionGenerator):
    def __init__(self):
        self.times = []

    def generate(self, state):
        self.times.append(state.t_now)
        return iter(())


def make_engine(model, *, end_time=30.0, continuous_interval=math.inf):
    engine = SimEngineThread(
        model,
        TimeIntervalContinuous=continuous_interval,
    )
    engine.bVerbose = False
    engine.set_simulation_time_bound(end_time)
    return engine


def get_loads_and_bridge(model):
    loads = {
        component.name: component
        for component in model.children
        if isinstance(component, LoadPassive)
    }
    bridge = next(
        component for component in model.children
        if isinstance(component, LogBridge)
    )
    return loads, bridge


def test_generator_is_invoked_after_each_internal_devs_execution():
    model = makeModel()
    engine = make_engine(model)
    generator = LogBridgeDecisionGenerator()
    controller = DecisionRuntimeController(engine, generator)

    first = controller.execute_one_step()
    second = controller.execute_one_step()

    assert first.execution_result is ExecutionResult.ScheduledEventOccurred
    assert first.snapshot.t_now == 20.0
    assert first.decisions == (EnterBridgeDecision("L11"),)

    assert second.execution_result is ExecutionResult.ScheduledEventOccurred
    assert second.snapshot.t_now == 100 / 4.5
    assert second.decisions == (
        EnterBridgeDecision("L11"),
        EnterBridgeDecision("L21"),
    )
    assert controller.devs_execution_count == 2


def test_generator_is_invoked_after_external_devs_execution():
    model = makeModel()
    loads, _ = get_loads_and_bridge(model)
    engine = make_engine(model)
    generator = RecordingGenerator()
    controller = DecisionRuntimeController(engine, generator)

    loads["L11"].phase = "W"
    engine.inject(loads["L11"], PortValue(loads["L11"].IP["?go"]))

    step = controller.execute_one_step()

    assert step.execution_result is ExecutionResult.ExternalEventOccurred
    assert step.snapshot.t_now == 0.0
    assert generator.times == [0.0]
    assert controller.devs_execution_count == 1


def test_generator_is_not_invoked_after_continuous_execution():
    model = makeModel()
    engine = make_engine(model, continuous_interval=1.0)
    generator = RecordingGenerator()
    controller = DecisionRuntimeController(engine, generator)

    step = controller.execute_one_step()

    assert step.execution_result is ExecutionResult.NoEventOccurred
    assert step.snapshot is None
    assert step.decisions == ()
    assert generator.times == []
    assert controller.devs_execution_count == 0


def test_generator_is_not_invoked_at_time_bound():
    model = makeModel()
    engine = make_engine(model, end_time=1.0)
    generator = RecordingGenerator()
    controller = DecisionRuntimeController(engine, generator)

    step = controller.execute_one_step()

    assert step.execution_result is ExecutionResult.TimeBoundReached
    assert step.snapshot is None
    assert generator.times == []
