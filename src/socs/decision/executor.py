from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from .decision import Decision


StateT = TypeVar("StateT")
DecisionT = TypeVar("DecisionT", bound=Decision)


class DecisionExecutor(ABC, Generic[StateT, DecisionT]):
    """Base class for executing decisions against system states."""

    @abstractmethod
    def execute(
        self,
        state: StateT,
        decision: DecisionT,
    ) -> StateT:
        """Execute a decision and return the resulting state."""
        raise NotImplementedError
