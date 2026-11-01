
from dataclasses import dataclass

import pytest

from socs.decision import Decision
from socs.decision.evaluator import DecisionEvaluator


@dataclass(frozen=True)
class SampleState:
    value: int


@dataclass(frozen=True)
class IncrementDecision(Decision):
    amount: int


class IncrementEvaluator(
    DecisionEvaluator[SampleState, IncrementDecision]
):

    def evaluate(
        self,
        state: SampleState,
        decision: IncrementDecision,
    ) -> SampleState:
        return SampleState(
            value=state.value + decision.amount
        )


def test_evaluator_returns_new_state():
    state = SampleState(value=10)
    decision = IncrementDecision(amount=5)

    evaluator = IncrementEvaluator()
    next_state = evaluator.evaluate(state, decision)

    assert next_state.value == 15


def test_evaluator_does_not_modify_input_state():
    state = SampleState(value=10)
    decision = IncrementDecision(amount=5)

    evaluator = IncrementEvaluator()
    next_state = evaluator.evaluate(state, decision)

    assert state.value == 10
    assert next_state.value == 15
    assert next_state is not state


def test_evaluator_is_abstract():
    with pytest.raises(TypeError):
        DecisionEvaluator()
