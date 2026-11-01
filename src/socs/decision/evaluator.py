from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from .decision import Decision


StateT = TypeVar("StateT")
DecisionT = TypeVar("DecisionT", bound=Decision)


class DecisionEvaluator(ABC, Generic[StateT, DecisionT]):
    """Base class for evaluating decisions against system states."""

    @abstractmethod
    def evaluate(
        self,
        state: StateT,
        decision: DecisionT,
    ) -> StateT:
        """Evaluate a decision and return the resulting state."""
        raise NotImplementedError
