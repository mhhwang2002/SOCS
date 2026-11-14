import math

import pytest

from socs.devs.DEVS import DEVS
from socs.devs.TESEngin import ExecutionResult, TESEnginBase, TESEnginThread
from socs.decision.decision import Decision
from socs.decision.simulation_executor import SimulationDecisionExecutor


class CounterModel(DEVS):
    """Deterministic model used to test simulation-time branch evaluation."""

    def __init__(self, name="Counter"):
        super().__init__(name)
        self.value = 0
        self.events = 0

    def set_initial_state(self):
        self.value = 0
        self.events = 0

    def delta_x(self, event):
        self.value += event.value
        return True

    def delta_y(self):
        self.events += 1
        self.value += 1

    def tau(self):
        return 2.0

    def get_state(self):
        return {"value": self.value, "events": self.events}

    def set_state(self, state):
        self.value = state["value"]
        self.events = state["events"]

    def state_to_s(self):
        return str(self.get_state())


class AddDecision(Decision):
    """Decision containing a value increment."""

    def __init__(self, amount):
        self.amount = amount


class CounterEvaluator(SimulationDecisionExecutor):
    """Evaluator that applies an additive decision to the counter model."""

    def apply_decision(self, model, decision):
        model.value += decision.amount


@pytest.fixture
def engine_and_model():
    model = CounterModel()
    engine = TESEnginThread(
        model,
        TimeIntervalContinuous=math.inf,
        TimeIntervalAnimation=math.inf,
    )
    engine.bVerbose = False
    return engine, model


def test_set_simulation_time_bound_accepts_valid_absolute_time(engine_and_model):
    engine, _ = engine_and_model
    engine.tNow = 1.0

    engine.set_simulation_time_bound(5)

    assert engine._tSimBound == 5.0


@pytest.mark.parametrize("value", [True, "5", None])
def test_set_simulation_time_bound_rejects_invalid_types(engine_and_model, value):
    engine, _ = engine_and_model

    with pytest.raises(TypeError):
        engine.set_simulation_time_bound(value)


@pytest.mark.parametrize("value", [math.inf, -math.inf, math.nan])
def test_set_simulation_time_bound_rejects_non_finite_values(
    engine_and_model, value
):
    engine, _ = engine_and_model

    with pytest.raises(ValueError):
        engine.set_simulation_time_bound(value)


def test_set_simulation_time_bound_rejects_time_before_current_time(
    engine_and_model,
):
    engine, _ = engine_and_model
    engine.tNow = 3.0

    with pytest.raises(ValueError, match="earlier"):
        engine.set_simulation_time_bound(2.0)


def test_engine_stops_at_absolute_simulation_time_horizon(engine_and_model):
    engine, model = engine_and_model
    engine.set_simulation_time_bound(5.0)

    while True:
        result = engine.execute_one_step()
        if result is ExecutionResult.TimeBoundReached:
            break

    assert engine.tNow == 5.0
    assert model.events == 2


def test_evaluator_runs_independent_branches_from_same_snapshot(engine_and_model):
    engine, model = engine_and_model
    initial_snapshot = engine.capture_snapshot()
    evaluator = CounterEvaluator(engine, end_time=5.0)

    branch_a = evaluator.execute(initial_snapshot, AddDecision(10))
    branch_b = evaluator.execute(initial_snapshot, AddDecision(20))

    assert branch_a.root_model.model_state == {"value": 12, "events": 2}
    assert branch_b.root_model.model_state == {"value": 22, "events": 2}
    assert initial_snapshot.root_model.model_state == {"value": 0, "events": 0}

    engine.restore_snapshot(initial_snapshot)
    assert model.value == 0
    assert model.events == 0
