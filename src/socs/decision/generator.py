
from abc import ABC, abstractmethod
from collections.abc import Iterator
from typing import Generic, TypeVar

from .decision import Decision


StateT = TypeVar("StateT")


class DecisionGenerator(ABC, Generic[StateT]):
    """Base class for lazy decision generation."""

    @abstractmethod
    def generate(self, state: StateT) -> Iterator[Decision]:
        """Generate feasible decisions for the given state."""
        raise NotImplementedError
