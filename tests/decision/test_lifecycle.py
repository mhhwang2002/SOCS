from dataclasses import dataclass

import pytest

from socs.decision import (
    Decision,
    DecisionGenerator,
    DecisionLifeCycleManager,
    DecisionStatus,
)


@dataclass(frozen=True)
class SampleDecision(Decision):
    value: int


class SampleGenerator(DecisionGenerator[int]):
    def __init__(self, count: int):
        self.count = count
        self.generated_count = 0

    def generate(self, state: int):
        for value in range(state, state + self.count):
            self.generated_count += 1
            yield SampleDecision(value)


def test_register_node_and_generate_decisions_lazily():
    generator = SampleGenerator(5)
    manager = DecisionLifeCycleManager()

    manager.register_node(1, 10, generator)

    assert generator.generated_count == 0
    records = manager.generate_decisions(1, max_count=2)
    assert [record.decision.value for record in records] == [10, 11]
    assert generator.generated_count == 2


def test_generate_decisions_in_batches_and_keep_iterator_position():
    generator = SampleGenerator(5)
    manager = DecisionLifeCycleManager()
    manager.register_node(1, 10, generator)

    first = manager.generate_decisions(1, max_count=2)
    second = manager.generate_decisions(1, max_count=2)

    assert [record.decision.value for record in first] == [10, 11]
    assert [record.decision.value for record in second] == [12, 13]
    assert [record.decision_id for record in first + second] == [1, 2, 3, 4]
    assert not manager.is_exhausted(1)


def test_generate_all_marks_generator_exhausted():
    manager = DecisionLifeCycleManager()
    manager.register_node(1, 3, SampleGenerator(2))

    records = manager.generate_decisions(1)

    assert [record.decision.value for record in records] == [3, 4]
    assert manager.is_exhausted(1)
    assert not manager.is_complete(1)


def test_select_decision_is_fifo_and_marks_executing():
    manager = DecisionLifeCycleManager()
    manager.register_node(1, 0, SampleGenerator(3))
    manager.generate_decisions(1)

    first = manager.select_decision(1)
    second = manager.select_decision(1)

    assert first.decision.value == 0
    assert second.decision.value == 1
    assert first.status is DecisionStatus.EXECUTING
    assert second.status is DecisionStatus.EXECUTING
    assert [r.decision.value for r in manager.get_pending_decisions(1)] == [2]


def test_execution_completion_records_child_node():
    manager = DecisionLifeCycleManager()
    manager.register_node(1, 0, SampleGenerator(1))
    record = manager.generate_decisions(1)[0]

    selected = manager.select_decision(1)
    assert selected is record

    manager.mark_executed(record.decision_id, child_node_id=2)

    assert record.status is DecisionStatus.EXECUTED
    assert record.child_node_id == 2
    assert manager.is_complete(1)


def test_execution_failure_records_error():
    manager = DecisionLifeCycleManager()
    manager.register_node(1, 0, SampleGenerator(1))
    record = manager.generate_decisions(1)[0]
    manager.select_decision(1)

    manager.mark_failed(record.decision_id, RuntimeError("branch failed"))

    assert record.status is DecisionStatus.FAILED
    assert record.error == "branch failed"
    assert manager.is_complete(1)


def test_empty_generator_is_exhausted_and_complete():
    manager = DecisionLifeCycleManager()
    manager.register_node(1, 0, SampleGenerator(0))

    assert manager.generate_decisions(1) == []
    assert manager.is_exhausted(1)
    assert manager.is_complete(1)


def test_unknown_node_and_decision_raise_key_error():
    manager = DecisionLifeCycleManager()

    with pytest.raises(KeyError):
        manager.generate_decisions(99)

    manager.register_node(1, 0, SampleGenerator(1))

    with pytest.raises(KeyError):
        manager.get_record(99)


def test_duplicate_node_registration_is_rejected():
    manager = DecisionLifeCycleManager()
    generator = SampleGenerator(1)
    manager.register_node(1, 0, generator)

    with pytest.raises(ValueError):
        manager.register_node(1, 0, generator)


def test_invalid_generator_and_max_count_are_rejected():
    manager = DecisionLifeCycleManager()

    with pytest.raises(TypeError):
        manager.register_node(1, 0, object())

    manager.register_node(1, 0, SampleGenerator(1))

    with pytest.raises(ValueError):
        manager.generate_decisions(1, max_count=-1)


def test_decision_status_transition_requires_execution():
    manager = DecisionLifeCycleManager()
    manager.register_node(1, 0, SampleGenerator(1))
    record = manager.generate_decisions(1)[0]

    with pytest.raises(ValueError):
        manager.mark_executed(record.decision_id, 2)

    with pytest.raises(ValueError):
        manager.mark_failed(record.decision_id, "not executing")


def test_generator_must_yield_decision_instances():
    class InvalidGenerator(DecisionGenerator[int]):
        def generate(self, state: int):
            yield "not a decision"

    manager = DecisionLifeCycleManager()
    manager.register_node(1, 0, InvalidGenerator())

    with pytest.raises(TypeError):
        manager.generate_decisions(1)
