from dataclasses import dataclass

from socs.decision.decision import Decision


@dataclass(frozen=True)
class SampleDecision(Decision):
    value: int


def test_decision_creation():
    decision = SampleDecision(value=10)

    assert isinstance(decision, Decision)
    assert decision.value == 10


def test_decision_is_immutable():
    decision = SampleDecision(value=10)

    try:
        decision.value = 20
        assert False, "Decision should be immutable."
    except AttributeError:
        pass