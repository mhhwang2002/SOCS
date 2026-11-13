import math

import pytest

from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.DEVS import DEVS
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import TESEnginThread
from socs.decision.decision import Decision
from socs.decision.simulation_executor import SimulationDecisionExecutor


class NestedCounter(DEVS):
    def __init__(self, name="atomic", interval=3.0):
        super().__init__(name)
        self.interval = interval
        self.value = 0
        self.events = 0

    def set_initial_state(self):
        self.value = 0
        self.events = 0

    def tau(self):
        return self.interval

    def delta_x(self, event):
        self.value += event.value
        return True

    def delta_y(self):
        self.events += 1
        self.value += 1
        return None

    def get_state(self):
        return {"value": self.value, "events": self.events}

    def set_state(self, state):
        self.value = state["value"]
        self.events = state["events"]


class AddInputDecision(Decision):
    def __init__(self, amount):
        self.amount = amount


class NestedCounterEvaluator(SimulationDecisionExecutor):
    def apply_decision(self, model, decision):
        model.Delta_x(
            PortValue(model.IP["decision"], decision.amount),
            self.engine.tNow,
        )


@pytest.fixture
def nested_system():
    root = CoupledDEVS("root")
    nested = CoupledDEVS("nested")
    atomic = NestedCounter()

    root.add_child(nested)
    nested.add_child(atomic)

    root.addInputPorts("decision")
    nested.addInputPorts("external")
    atomic.addInputPorts("in")

    root.addCouple((root, "decision"), (nested, "external"))
    nested.addCouple((nested, "external"), (atomic, "in"))

    engine = TESEnginThread(
        root,
        TimeIntervalContinuous=math.inf,
        TimeIntervalAnimation=math.inf,
    )
    engine.bVerbose = False

    return engine, root, nested, atomic


def test_nested_snapshot_restores_atomic_state_and_schedules(nested_system):
    engine, root, nested, atomic = nested_system
    snapshot = engine.capture_snapshot()

    atomic.value = 99
    atomic.events = 8
    atomic.tNext = 42.0
    nested.update_tN(update_parent=True)

    engine.restore_snapshot(snapshot)

    assert atomic.value == 0
    assert atomic.events == 0
    assert atomic.tNext == 3.0
    assert nested.tNext == 3.0
    assert root.tNext == 3.0
    assert atomic in nested.IMM
    assert nested in root.IMM


def test_nested_branches_are_independent_and_keep_schedule_consistent(
    nested_system,
):
    engine, root, nested, atomic = nested_system
    initial = engine.capture_snapshot()
    evaluator = NestedCounterEvaluator(engine, end_time=7.0)

    branch_a = evaluator.execute(initial, AddInputDecision(10))

    assert branch_a.t_now == 7.0
    assert branch_a.root_model.children["nested"].children["atomic"].model_state == {
        "value": 12,
        "events": 2,
    }

    branch_b = evaluator.execute(initial, AddInputDecision(20))

    assert branch_b.t_now == 7.0
    assert branch_b.root_model.children["nested"].children["atomic"].model_state == {
        "value": 22,
        "events": 2,
    }

    assert initial.root_model.children["nested"].children["atomic"].model_state == {
        "value": 0,
        "events": 0,
    }

    assert atomic.value == 22
    assert atomic.tNext == 9.0
    assert nested.tNext == 9.0
    assert root.tNext == 9.0
    assert atomic in nested.IMM
    assert nested in root.IMM


def test_nested_branch_can_be_repeated_deterministically(nested_system):
    engine, _, _, atomic = nested_system
    initial = engine.capture_snapshot()
    evaluator = NestedCounterEvaluator(engine, end_time=7.0)

    first = evaluator.execute(initial, AddInputDecision(5))
    second = evaluator.execute(initial, AddInputDecision(5))

    assert first.root_model.children["nested"].children["atomic"].model_state == (
        second.root_model.children["nested"].children["atomic"].model_state
    )
    assert atomic.value == 7
    assert atomic.events == 2
