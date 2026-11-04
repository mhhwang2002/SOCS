
from socs.decision import DecisionLifeCycleManager, DecisionStatus

from socs.decision.decision import ControlDecision, NoControl
from examples.LogBridge.decision_generator import LogBridgeDecisionGenerator

from tests.examples.LogBridge.test_decision_generator import (
    make_system_snapshot,
)


def test_manager_registers_and_generates_logbridge_decisions():
    snapshot = make_system_snapshot()

    manager = DecisionLifeCycleManager()
    manager.register_node(1, snapshot, LogBridgeDecisionGenerator())

    records = manager.generate_decisions(1)

    assert len(records) == 3
    assert [r.decision_id for r in records] == [1, 2, 3]
    assert [r.decision for r in records] == [
        ControlDecision("Load_LogBridge.L11", "?go"),
        ControlDecision("Load_LogBridge.L21", "?go"),
        NoControl(),
    ]


def test_manager_selects_logbridge_decisions_in_fifo_order():
    snapshot = make_system_snapshot()

    manager = DecisionLifeCycleManager()
    manager.register_node(1, snapshot, LogBridgeDecisionGenerator())
    manager.generate_decisions(1)

    first = manager.select_decision(1)
    second = manager.select_decision(1)
    third = manager.select_decision(1)

    assert first.decision.target_model.endswith(".L11")
    assert second.decision.target_model.endswith(".L21")
    assert third.decision == NoControl()

    assert first.status is DecisionStatus.EXECUTING
    assert second.status is DecisionStatus.EXECUTING
    assert third.status is DecisionStatus.EXECUTING

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
    assert record.decision == ControlDecision("Load_LogBridge.L11", "?go")
