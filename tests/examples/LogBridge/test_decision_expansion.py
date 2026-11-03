from socs.decision.expansion import ExpansionStatus
from socs.decision.lifecycle import DecisionLifeCycleManager

from examples.LogBridge.decision_generator import (
    LogBridgeDecisionGenerator,
)
from examples.LogBridge.decision_executor import (
    LogBridgeDecisionExecutor,
)
from socs.decision.expansion import DecisionExpansionController
from socs.tree.decision_tree import DecisionTree

from tests.examples.LogBridge.test_decision_generator import (
    make_system_snapshot,
)
from tests.examples.LogBridge.test_decision_executor import (
    make_root_snapshot,
)

def test_expand_root_into_two_branches():
    engine, root_snapshot = make_root_snapshot()

    tree = DecisionTree()
    root_id = tree.add_root(root_snapshot)

    generator = LogBridgeDecisionGenerator()

    evaluator = LogBridgeDecisionExecutor(
        engine=engine,
        end_time=70.0,
    )

    controller = DecisionExpansionController(
        tree=tree,
        lifecycle_manager=DecisionLifeCycleManager(),
        executor=evaluator,
    )

    controller.register_node(root_id, generator)

    # Expand the first decision.
    result1 = controller.expand(root_id)

    assert result1.status == ExpansionStatus.EXECUTED
    assert result1.child_node_id is not None

    # Expand the second decision.
    result2 = controller.expand(root_id)

    assert result2.status == ExpansionStatus.EXECUTED
    assert result2.child_node_id is not None

    # Both decisions must create separate child nodes.
    children = tree.get_children(root_id)

    assert len(children) == 2

    # Verify the decisions stored in the tree.
    path1 = tree.get_path(children[0])
    path2 = tree.get_path(children[1])

    assert path1[0].load_name == "L11"
    assert path2[0].load_name == "L21"

    # Verify the resulting snapshots.
    states1 = {
        name: child.model_state
        for name, child in tree.get_snapshot(children[0]).root_model.children.items()
    }

    states2 = {
        name: child.model_state
        for name, child in tree.get_snapshot(children[1]).root_model.children.items()
    }

    assert states1["L11"][0] == "I"
    assert states1["L21"][0] == "W"

    assert states2["L11"][0] == "W"
    assert states2["L21"][0] == "I"

    # The root must remain unchanged.
    root_states = {
        name: child.model_state
        for name, child in tree.get_snapshot(root_id).root_model.children.items()
    }

    assert root_states["L11"][0] == "W"
    assert root_states["L21"][0] == "W"


def test_expand_returns_no_decision_when_no_candidate_exists():
    # Neither source Load is waiting.
    root_snapshot = make_system_snapshot(
        l11_state=("B", 5),
        l21_state=("B", 4.5),
    )

    tree = DecisionTree()
    root_id = tree.add_root(root_snapshot)

    manager = DecisionLifeCycleManager()
    engine, _ = make_root_snapshot()
    controller = DecisionExpansionController(
        tree=tree,
        lifecycle_manager=manager,
        executor=LogBridgeDecisionExecutor(
            engine=engine,
            end_time=70.0,
        ),
    )

    controller.register_node(
        root_id,
        LogBridgeDecisionGenerator(),
    )

    result = controller.expand(root_id)

    assert result.status == ExpansionStatus.NO_DECISION
    assert result.decision_id is None
    assert result.child_node_id is None

    assert tree.get_children(root_id) == []
    assert manager.is_exhausted(root_id)
    assert manager.is_complete(root_id)

from socs.decision.lifecycle import DecisionLifeCycleManager, DecisionStatus

def test_expand_marks_decision_failed_on_evaluator_error():
    engine, root_snapshot = make_root_snapshot()

    tree = DecisionTree()
    root_id = tree.add_root(root_snapshot)

    manager = DecisionLifeCycleManager()

    class FailingEvaluator(LogBridgeDecisionExecutor):
        def execute(self, state, decision):
            raise RuntimeError("Intentional evaluation failure")

    evaluator = FailingEvaluator(
        engine=engine,
        end_time=70.0,
    )

    controller = DecisionExpansionController(
        tree=tree,
        lifecycle_manager=manager,
        executor=evaluator,
    )

    controller.register_node(
        root_id,
        LogBridgeDecisionGenerator(),
    )

    result = controller.expand(root_id)

    assert result.status == ExpansionStatus.FAILED
    assert result.decision_id is not None
    assert result.child_node_id is None

    record = manager.get_record(result.decision_id)

    assert record.status == DecisionStatus.FAILED
    assert "Intentional evaluation failure" in str(record.error)

    assert tree.get_children(root_id) == []

    assert manager.get_pending_decisions(root_id) == []
    assert manager.is_complete(root_id) is False