from dataclasses import dataclass

import pytest

from examples.FDDEVS_Simple.FDDEVSCounter import Counter

from socs.devs.SimEngine import SimEngineBase
from socs.decision.decision import Decision
from socs.decision.generator import DecisionGenerator
from socs.decision.manager import DecisionManager
from socs.decision.simulation_evaluator import SimulationDecisionEvaluator
from socs.decision.expansion import (
    DecisionExpansionController,
    ExpansionStatus,
)
from socs.tree.decision_tree import DecisionTree
 
 
@dataclass(frozen=True)
class SetCountLimitDecision(Decision):
    count_no: int


class CounterDecisionGenerator(DecisionGenerator):
    def generate(self, state):
        yield SetCountLimitDecision(2)
        yield SetCountLimitDecision(4)


class CounterDecisionEvaluator(SimulationDecisionEvaluator):
    def apply_decision(self, model, decision):
        model.count_no = decision.count_no


@pytest.fixture
def expansion_system():
    model = Counter(name="counter", count_no=2)
    engine = SimEngineBase(model)
    engine.bVerbose = False

    evaluator = CounterDecisionEvaluator(
        engine=engine,
        end_time=25,
    )

    tree = DecisionTree()
    root_snapshot = engine.capture_snapshot()
    root_id = tree.add_root(root_snapshot)

    manager = DecisionManager()
    controller = DecisionExpansionController(
        tree=tree,
        decision_manager=manager,
        evaluator=evaluator,
    )

    generator = CounterDecisionGenerator()
    controller.register_node(root_id, generator)

    return {
        "model": model,
        "engine": engine,
        "tree": tree,
        "manager": manager,
        "controller": controller,
        "root_id": root_id,
    }


def test_root_expands_into_independent_branches(expansion_system):
    system = expansion_system

    tree = system["tree"]
    controller = system["controller"]
    root_id = system["root_id"]

    root_snapshot = tree.get_snapshot(root_id)

    result_a = controller.expand(root_id)
    result_b = controller.expand(root_id)

    assert result_a.status is ExpansionStatus.EXECUTED
    assert result_b.status is ExpansionStatus.EXECUTED

    node_a = result_a.child_node_id
    node_b = result_b.child_node_id

    snapshot_a = tree.get_snapshot(node_a)
    snapshot_b = tree.get_snapshot(node_b)

    state_a = snapshot_a.root_model.model_state
    state_b = snapshot_b.root_model.model_state

    assert state_a == ("D", 3, 3, 2)
    assert state_b == ("D", 5, 10, 4)

    assert root_snapshot.root_model.model_state == ("C", 1, 0, 2)

    assert tree.get_children(root_id) == [node_a, node_b]

    assert tree.get_parent(node_a) == root_id
    assert tree.get_parent(node_b) == root_id


def test_child_node_can_be_expanded(expansion_system):
    system = expansion_system

    tree = system["tree"]
    controller = system["controller"]
    root_id = system["root_id"]

    result = controller.expand(root_id)

    assert result.status is ExpansionStatus.EXECUTED

    child_id = result.child_node_id

    child_result = controller.expand(child_id)

    assert child_result.status is ExpansionStatus.EXECUTED

    grandchild_id = child_result.child_node_id

    assert tree.get_parent(grandchild_id) == child_id
    assert tree.get_parent(child_id) == root_id
    assert len(tree.get_path(grandchild_id)) == 2