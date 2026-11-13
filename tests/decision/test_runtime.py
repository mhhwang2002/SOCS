import math

from socs.decision import (
    DecisionGenerator,
    DecisionRuntimeController,
    DecisionStatus,
    FixedPrioritySelector,
)
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import ExecutionResult, TESEnginThread

from examples.LogBridge.LoadPassive import LoadPassive
from examples.LogBridge.LoadPassive_LogBridge import makeModel
from examples.LogBridge.LogBridge import LogBridge
from examples.LogBridge.decision_generator import LogBridgeDecisionGenerator
from socs.decision.decision import ControlDecision, NoControl
from examples.LogBridge.runtime_executor import LogBridgeRuntimeExecutor


class RecordingGenerator(DecisionGenerator):
    def __init__(self):
        self.times = []

    def generate(self, state):
        self.times.append(state.t_now)
        return iter(())


def make_engine(model, *, end_time=30.0, continuous_interval=math.inf):
    engine = TESEnginThread(
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
    assert first.decisions == (ControlDecision("Load_LogBridge.L11", "?go"), NoControl())

    assert second.execution_result is ExecutionResult.ScheduledEventOccurred
    assert second.snapshot.t_now == 100 / 4.5
    assert second.decisions == (
        ControlDecision("Load_LogBridge.L11", "?go"),
        ControlDecision("Load_LogBridge.L21", "?go"),
        NoControl(),
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



def test_runtime_selects_and_executes_enter_bridge_decision():
    model = makeModel()
    loads, _ = get_loads_and_bridge(model)
    engine = make_engine(model)
    selector = FixedPrioritySelector(
        priority=lambda decision: (
            10 if isinstance(decision, ControlDecision) and decision.target_model is not None else 0
        )
    )
    controller = DecisionRuntimeController(
        engine,
        LogBridgeDecisionGenerator(),
        selector=selector,
        executor=LogBridgeRuntimeExecutor(),
    )

    step = controller.execute_one_step()

    assert step.execution_result is ExecutionResult.ScheduledEventOccurred
    assert step.snapshot.t_now == 20.0
    assert step.decisions == (
        ControlDecision("Load_LogBridge.L11", "?go"),
        NoControl(),
    )
    assert step.selected_record.decision == ControlDecision("Load_LogBridge.L11", "?go")
    assert step.selected_record.status is DecisionStatus.EXECUTED
    assert len(engine.XBuffer) == 1
    assert engine.XBuffer[0][0] is loads["L11"]

    external_step = controller.execute_one_step()

    assert external_step.execution_result is ExecutionResult.ExternalEventOccurred
    assert external_step.snapshot.t_now == 20.0
    assert external_step.decisions == ()
    assert loads["L11"].phase == "S"


def test_wait_decision_leaves_engine_unchanged_until_next_step():
    model = makeModel()
    loads, _ = get_loads_and_bridge(model)
    engine = make_engine(model)
    selector = FixedPrioritySelector(
        priority=lambda decision: (
            100 if isinstance(decision, NoControl) else 0
        )
    )
    controller = DecisionRuntimeController(
        engine,
        LogBridgeDecisionGenerator(),
        selector=selector,
        executor=LogBridgeRuntimeExecutor(),
    )

    wait_step = controller.execute_one_step()

    assert wait_step.snapshot.t_now == 20.0
    assert wait_step.selected_record.decision == NoControl()
    assert wait_step.selected_record.status is DecisionStatus.EXECUTED
    assert not engine.XBuffer
    assert loads["L11"].phase == "W"

    next_step = controller.execute_one_step()

    assert next_step.execution_result is ExecutionResult.ScheduledEventOccurred
    assert next_step.snapshot.t_now == 100 / 4.5
    assert loads["L11"].phase == "W"
    assert loads["L21"].phase == "W"
