import math

import pytest

from socs.decision.decision import Decision
from socs.optimization.simulation_executor import SimulationDecisionExecutor
from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.DEVSInterfaces import PortValue
from socs.devs.FDDEVS import FDDEVS
from socs.devs.SimEngine import SimEngineThread
from socs.tree import DecisionTree


class AddInputDecision(Decision):
    def __init__(self, amount):
        self.amount = amount


class NestedCounter(FDDEVS):
    def __init__(self, name="atomic", interval=3.0):
        super().__init__(name)
        self.interval = interval
        self.value = 0
        self.events = 0

    def delta_0(self):
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


class NestedCounterEvaluator(SimulationDecisionExecutor):
    def apply_decision(self, model, decision):
        model.Delta_x(
            PortValue(model.IP["decision"], decision.amount),
            self.engine.tNow,
        )


@pytest.fixture
def simulation_system():
    root = CoupledDEVS("root")
    nested = CoupledDEVS("nested")
    atomic = NestedCounter()

    root.add_child(nested)
    nested.add_child(atomic)

    root.addInputPorts("decision")
    nested.addInputPorts("external")
    atomic.addInputPorts("in")

    root.addCouple(root.IP["decision"], nested.IP["external"])
    nested.addCouple(nested.IP["external"], atomic.IP["in"])

    engine = SimEngineThread(
        root,
        TimeIntervalContinuous=math.inf,
        TimeIntervalAnimation=math.inf,
    )
    engine.bVerbose = False

    return engine, root, nested, atomic


def atomic_state(snapshot):
    return snapshot.root_model.children["nested"].children["atomic"].model_state


def test_decision_tree_branches_simulation_executor_results(simulation_system):
    engine, root, nested, atomic = simulation_system
    initial_snapshot = engine.capture_snapshot()

    tree = DecisionTree()
    root_id = tree.add_root(initial_snapshot)

    first_evaluator = NestedCounterEvaluator(engine, end_time=7.0)
    first_decision = AddInputDecision(10)
    first_snapshot = first_evaluator.execute(
        tree.get_snapshot(root_id), first_decision
    )
    first_id = tree.add_child(root_id, first_decision, first_snapshot)

    branch_evaluator = NestedCounterEvaluator(engine, end_time=14.0)

    decision_a = AddInputDecision(100)
    snapshot_a = branch_evaluator.execute(
        tree.get_snapshot(first_id), decision_a
    )
    node_a = tree.add_child(first_id, decision_a, snapshot_a)

    decision_b = AddInputDecision(200)
    snapshot_b = branch_evaluator.execute(
        tree.get_snapshot(first_id), decision_b
    )
    node_b = tree.add_child(first_id, decision_b, snapshot_b)

    assert (root_id, first_id, node_a, node_b) == (1, 2, 3, 4)
    assert tree.get_children(root_id) == [first_id]
    assert tree.get_children(first_id) == [node_a, node_b]
    assert tree.get_parent(node_a) == first_id
    assert tree.get_parent(node_b) == first_id

    assert tree.get_path(root_id) == []
    assert tree.get_path(first_id) == [first_decision]
    assert tree.get_path(node_a) == [first_decision, decision_a]
    assert tree.get_path(node_b) == [first_decision, decision_b]

    assert atomic_state(initial_snapshot) == {"value": 0, "events": 0}
    assert atomic_state(first_snapshot) == {"value": 12, "events": 2}
    assert atomic_state(snapshot_a) == {"value": 114, "events": 4}
    assert atomic_state(snapshot_b) == {"value": 214, "events": 4}

    assert tree.get_snapshot(first_id) is first_snapshot
    assert tree.get_snapshot(node_a) is snapshot_a
    assert tree.get_snapshot(node_b) is snapshot_b

    assert atomic.value == 214
    assert atomic.timeNext() == 16.0
    assert nested.timeNext() == 16.0
    assert root.timeNext() == 16.0
    assert atomic in nested.IMM
    assert nested in root.IMM


def test_decision_tree_branch_snapshot_can_be_re_evaluated(simulation_system):
    engine, _, _, atomic = simulation_system
    initial_snapshot = engine.capture_snapshot()

    tree = DecisionTree()
    root_id = tree.add_root(initial_snapshot)

    first_evaluator = NestedCounterEvaluator(engine, end_time=7.0)
    first_decision = AddInputDecision(10)
    first_snapshot = first_evaluator.execute(initial_snapshot, first_decision)
    first_id = tree.add_child(root_id, first_decision, first_snapshot)

    evaluator = NestedCounterEvaluator(engine, end_time=14.0)
    decision = AddInputDecision(50)

    result_1 = evaluator.execute(tree.get_snapshot(first_id), decision)
    result_2 = evaluator.execute(tree.get_snapshot(first_id), decision)

    assert atomic_state(result_1) == atomic_state(result_2)
    assert result_1.t_now == result_2.t_now == 14.0
    assert atomic_state(tree.get_snapshot(first_id)) == {"value": 12, "events": 2}
    assert atomic.value == 64
