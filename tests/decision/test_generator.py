
import pytest
from dataclasses import dataclass

from socs.decision.decision import Decision
from socs.decision.generator import DecisionGenerator


@dataclass(frozen=True)
class SampleDecision(Decision):
    value: int


class CountingGenerator(DecisionGenerator[int]):

    def __init__(self):
        self.generated_count = 0

    def generate(self, state: int):
        for value in range(state):
            self.generated_count += 1
            yield SampleDecision(value=value)


def test_generator_is_lazy():
    generator = CountingGenerator()

    decisions = generator.generate(3)

    assert generator.generated_count == 0

    first = next(decisions)

    assert first.value == 0
    assert generator.generated_count == 1


def test_generator_produces_decisions_one_at_a_time():
    generator = CountingGenerator()

    decisions = generator.generate(3)

    assert next(decisions).value == 0
    assert generator.generated_count == 1

    assert next(decisions).value == 1
    assert generator.generated_count == 2

    assert next(decisions).value == 2
    assert generator.generated_count == 3


def test_generator_exhaustion():
    generator = CountingGenerator()

    decisions = generator.generate(2)

    assert next(decisions).value == 0
    assert next(decisions).value == 1

    with pytest.raises(StopIteration):
        next(decisions)


def test_generator_with_no_decisions():
    generator = CountingGenerator()

    decisions = generator.generate(0)

    with pytest.raises(StopIteration):
        next(decisions)

    assert generator.generated_count == 0


def test_generator_creates_independent_iterators():
    generator = CountingGenerator()

    first = generator.generate(2)
    second = generator.generate(2)

    assert next(first).value == 0
    assert next(second).value == 0

    assert generator.generated_count == 2
