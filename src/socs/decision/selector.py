"""Decision selection policies."""

from abc import ABC, abstractmethod
from collections.abc import Callable, Sequence

from .decision import Decision
from .lifecycle import DecisionRecord


class DecisionSelector(ABC):
    """Select one pending decision from the supplied candidates."""

    @abstractmethod
    def select(
        self,
        records: Sequence[DecisionRecord],
    ) -> DecisionRecord | None:
        """Return one candidate record, or None when no decision is selected."""
        raise NotImplementedError


class FIFOSelector(DecisionSelector):
    """Select the earliest generated pending decision."""

    def select(
        self,
        records: Sequence[DecisionRecord],
    ) -> DecisionRecord | None:
        """Return the first candidate, preserving the supplied order."""
        if not records:
            return None
        return records[0]


class FixedPrioritySelector(DecisionSelector):
    """Select the candidate with the highest decision priority.

    The priority function receives a Decision and returns an integer.
    Higher values represent higher priority. Candidates with equal priority
    retain their original order.
    """

    def __init__(
        self,
        priority: Callable[[Decision], int],
    ) -> None:
        if not callable(priority):
            raise TypeError("priority must be callable.")
        self._priority = priority

    def select(
        self,
        records: Sequence[DecisionRecord],
    ) -> DecisionRecord | None:
        """Return the highest-priority candidate, preserving FIFO ties."""
        if not records:
            return None

        selected = records[0]
        selected_priority = self._get_priority(selected.decision)

        for record in records[1:]:
            priority = self._get_priority(record.decision)
            if priority > selected_priority:
                selected = record
                selected_priority = priority

        return selected

    def _get_priority(self, decision: Decision) -> int:
        priority = self._priority(decision)
        if isinstance(priority, bool) or not isinstance(priority, int):
            raise TypeError("Decision priority must be an integer.")
        return priority
