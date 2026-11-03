import math

import pytest

from socs.decision import (
    Decision,
    DecisionExpansionController,
    DecisionGenerator,
    DecisionLifeCycleManager,
    ExpansionStatus,
    DecisionStatus,
)
from socs.decision.simulation_executor import SimulationDecisionExecutor
from socs.devs.Simulatable import Simulatable
from socs.devs.SimEngine import SimEngineThread
from socs.tree import DecisionTree


class CounterModel(Simulatable):
    def __init__(self):
        super().__init__("counter")
        self.value = 0
        self.events = 0

    def delta_0(self):
        self.value = 0
        self.events = 0

    def delta_x(self, event):
        self.value += event

    def delta_y(self):
        self.events += 1
        self.value += 1

    def timeAdvance(self):
        return 2.0

    def get_state(self):
        return {"value": self.value, "events": self.events}

    def set_state(self, state):
        self.value = state["value"]
        self.events = state["events"]

    def state_to_s(self):
        return str(self.get_state())


class AddDecision(Decision):
    def __init__(self, amount):
        self.amount = amount


class AddDecisionGenerator(DecisionGenerator):
    def __init__(self, amounts):
        self.amounts = amounts

    def generate(self, state):
        for amount in self.amounts:
            yield AddDecision(amount)


class CounterEvaluator(SimulationDecisionExecutor):
    def apply_decision(self, model, decision):
        model.value += decision.amount


class FailingEvaluator(SimulationDecisionExecutor):
    def apply_decision(self, model, decision):
        raise RuntimeError("decision application failed")


@pytest.fixture
def controller_context():
    model = CounterModel()
    engine = SimEngineThread(
        model,
        TimeIntervalContinuous=math.inf,
        TimeIntervalAnimation=math.inf,
    )
    engine.bVerbose = False

    tree = DecisionTree()
    root_id = tree.add_root(engine.capture_snapshot())
    manager = DecisionLifeCycleManager()
    evaluator = CounterEvaluator(engine, end_time=5.0)
    controller = DecisionExpansionController(tree, manager, evaluator)
    return engine, model, tree, manager, controller, root_id


def test_expand_executes_one_decision_and_adds_child(controller_context):
    _, _, tree, manager, controller, root_id = controller_context
    controller.register_node(root_id, AddDecisionGenerator([10, 20]))

    result = controller.expand(root_id)

    assert result.status is ExpansionStatus.EXECUTED
    assert result.parent_node_id == root_id
    assert result.child_node_id == 2
    assert result.decision_id == 1
    assert tree.get_children(root_id) == [2]
    assert tree.get_decision(tree.out_edges(root_id)[0]).amount == 10

    record = manager.get_record(result.decision_id)
    assert record.status is DecisionStatus.EXECUTED
    assert record.child_node_id == result.child_node_id


def test_repeated_expansion_processes_decisions_one_at_a_time(controller_context):
    _, _, tree, manager, controller, root_id = controller_context
    controller.register_node(root_id, AddDecisionGenerator([10, 20]))

    first = controller.expand(root_id)
    second = controller.expand(root_id)
    third = controller.expand(root_id)

    assert first.status is ExpansionStatus.EXECUTED
    assert second.status is ExpansionStatus.EXECUTED
    assert third.status is ExpansionStatus.NO_DECISION
    assert tree.get_children(root_id) == [2, 3]
    assert manager.is_exhausted(root_id)
    assert manager.is_complete(root_id)

    first_state = tree.get_snapshot(first.child_node_id).root_model.model_state
    second_state = tree.get_snapshot(second.child_node_id).root_model.model_state
    assert first_state == {"value": 12, "events": 2}
    assert second_state == {"value": 22, "events": 2}


def test_child_node_is_registered_for_later_expansion(controller_context):
    _, _, tree, _, controller, root_id = controller_context
    controller.register_node(root_id, AddDecisionGenerator([10]))

    first = controller.expand(root_id)
    second = controller.expand(first.child_node_id)

    assert first.status is ExpansionStatus.EXECUTED
    assert second.status is ExpansionStatus.EXECUTED
    assert tree.get_parent(second.child_node_id) == first.child_node_id
    assert tree.get_path(second.child_node_id)[0].amount == 10
    assert tree.get_path(second.child_node_id)[1].amount == 10


def test_empty_generator_returns_no_decision(controller_context):
    _, _, tree, manager, controller, root_id = controller_context
    controller.register_node(root_id, AddDecisionGenerator([]))

    result = controller.expand(root_id)

    assert result.status is ExpansionStatus.NO_DECISION
    assert result.decision_id is None
    assert result.child_node_id is None
    assert manager.is_exhausted(root_id)
    assert tree.get_children(root_id) == []


def test_evaluation_failure_marks_decision_failed_without_child(controller_context):
    engine, _, tree, manager, _, root_id = controller_context
    failing_controller = DecisionExpansionController(
        tree,
        manager,
        FailingEvaluator(engine, end_time=5.0),
    )
    failing_controller.register_node(root_id, AddDecisionGenerator([10]))

    result = failing_controller.expand(root_id)

    assert result.status is ExpansionStatus.FAILED
    assert isinstance(result.error, RuntimeError)
    assert str(result.error) == "decision application failed"
    assert tree.get_children(root_id) == []
    record = manager.get_record(result.decision_id)
    assert record.status is DecisionStatus.FAILED
    assert record.error == "decision application failed"


def test_unknown_or_duplicate_node_registration_is_rejected(controller_context):
    _, _, _, _, controller, root_id = controller_context

    with pytest.raises(KeyError):
        controller.register_node(999, AddDecisionGenerator([1]))

    controller.register_node(root_id, AddDecisionGenerator([1]))
    with pytest.raises(ValueError):
        controller.register_node(root_id, AddDecisionGenerator([2]))

    with pytest.raises(KeyError):
        controller.expand(999)
