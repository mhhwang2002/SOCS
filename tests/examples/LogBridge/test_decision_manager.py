
from socs.decision import DecisionLifeCycleManager, DecisionStatus

from examples.LogBridge.decisions import EnterBridgeDecision
from examples.LogBridge.decision_generator import LogBridgeDecisionGenerator

from tests.examples.LogBridge.test_decision_generator import (
    make_system_snapshot,
)


def test_manager_registers_and_generates_logbridge_decisions():
    snapshot = make_system_snapshot()

    manager = DecisionLifeCycleManager()
    manager.register_node(1, snapshot, LogBridgeDecisionGenerator())

    records = manager.generate_decisions(1)

    assert len(records) == 2
    assert [r.decision_id for r in records] == [1, 2]
    assert [r.decision.load_name for r in records] == ["L11", "L21"]
    assert all(
        isinstance(r.decision, EnterBridgeDecision)
        for r in records
    )


def test_manager_selects_logbridge_decisions_in_fifo_order():
    snapshot = make_system_snapshot()

    manager = DecisionLifeCycleManager()
    manager.register_node(1, snapshot, LogBridgeDecisionGenerator())
    manager.generate_decisions(1)

    first = manager.select_decision(1)
    second = manager.select_decision(1)

    assert first.decision.load_name == "L11"
    assert second.decision.load_name == "L21"

    assert first.status is DecisionStatus.EXECUTING
    assert second.status is DecisionStatus.EXECUTING

    assert manager.select_decision(1) is None


def test_manager_tracks_logbridge_decision_execution():
    snapshot = make_system_snapshot()

    manager = DecisionLifeCycleManager()
    manager.register_node(1, snapshot, LogBridgeDecisionGenerator())
    manager.generate_decisions(1)

    selected = manager.select_decision(1)

    manager.mark_executed(
        selected.decision_id,
        child_node_id=2,
    )

    record = manager.get_record(selected.decision_id)

    assert record.status is DecisionStatus.EXECUTED
    assert record.child_node_id == 2
    assert record.decision == EnterBridgeDecision("L11")
