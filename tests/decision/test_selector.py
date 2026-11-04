from dataclasses import dataclass

import pytest

from socs.decision import (
    Decision,
    DecisionRecord,
    DecisionSelector,
    DecisionStatus,
    FIFOSelector,
    FixedPrioritySelector,
)


@dataclass(frozen=True)
class SampleDecision(Decision):
    value: int


def make_record(decision_id, value):
    return DecisionRecord(
        decision_id=decision_id,
        node_id=1,
        decision=SampleDecision(value),
    )


def test_fifo_selector_returns_first_candidate():
    records = [make_record(1, 10), make_record(2, 20)]

    selected = FIFOSelector().select(records)

    assert selected is records[0]


def test_fifo_selector_returns_none_for_empty_candidates():
    assert FIFOSelector().select([]) is None


def test_decision_selector_is_abstract():
    with pytest.raises(TypeError):
        DecisionSelector()


def test_fifo_selector_does_not_change_record_status():
    record = make_record(1, 10)

    FIFOSelector().select([record])

    assert record.status is DecisionStatus.PENDING



def test_fixed_priority_selector_returns_highest_priority():
    records = [make_record(1, 10), make_record(2, 20), make_record(3, 30)]
    selector = FixedPrioritySelector(
        priority=lambda decision: decision.value
    )

    selected = selector.select(records)

    assert selected is records[2]


def test_fixed_priority_selector_preserves_fifo_order_for_ties():
    records = [make_record(1, 10), make_record(2, 10), make_record(3, 5)]
    selector = FixedPrioritySelector(
        priority=lambda decision: decision.value
    )

    selected = selector.select(records)

    assert selected is records[0]


def test_fixed_priority_selector_returns_none_for_empty_candidates():
    selector = FixedPrioritySelector(priority=lambda decision: decision.value)

    assert selector.select([]) is None


def test_fixed_priority_selector_does_not_change_record_status():
    record = make_record(1, 10)
    selector = FixedPrioritySelector(priority=lambda decision: decision.value)

    selector.select([record])

    assert record.status is DecisionStatus.PENDING


def test_fixed_priority_selector_requires_callable():
    with pytest.raises(TypeError, match="priority must be callable"):
        FixedPrioritySelector(priority=10)


def test_fixed_priority_selector_requires_integer_priority():
    selector = FixedPrioritySelector(priority=lambda decision: "high")

    with pytest.raises(TypeError, match="priority must be an integer"):
        selector.select([make_record(1, 10)])


def test_fixed_priority_selector_rejects_boolean_priority():
    selector = FixedPrioritySelector(priority=lambda decision: True)

    with pytest.raises(TypeError, match="priority must be an integer"):
        selector.select([make_record(1, 10)])
