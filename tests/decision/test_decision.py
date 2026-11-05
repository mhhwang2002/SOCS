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
from socs.decision.decision import ControlDecision


def test_control_decision_represents_target_and_external_event():
    decision = ControlDecision("Root.LoadA", ("?go", 3))

    assert isinstance(decision, Decision)
    assert decision.target_model == "Root.LoadA"
    assert decision.xevent == ("?go", 3)


def test_none_target_is_an_explicit_no_control_decision():
    decision = ControlDecision(None)

    assert isinstance(decision, ControlDecision)
    assert decision.target_model is None
    assert decision.xevent is None


def test_control_decision_validates_target_and_event_pair():
    import pytest

    with pytest.raises(ValueError):
        ControlDecision(None, "?go")
    with pytest.raises(ValueError):
        ControlDecision("Root.LoadA", None)
