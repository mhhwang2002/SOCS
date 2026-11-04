from collections.abc import Iterator, Sequence

from socs.decision.decision import Decision
from socs.decision.generator import DecisionGenerator
from socs.devs.Snapshot import SystemSnapshot

from socs.decision.decision import ControlDecision, NoControl


class LogBridgeDecisionGenerator(DecisionGenerator[SystemSnapshot]):
    """Generate feasible bridge-entry and wait decisions from a snapshot."""

    def __init__(
        self,
        bridge_name: str = "LB",
        candidate_load_names: Sequence[str] = ("L11", "L21"),
    ) -> None:
        self.bridge_name = bridge_name
        self.candidate_load_names = tuple(candidate_load_names)

        if not self.candidate_load_names:
            raise ValueError("At least one candidate Load must be configured.")

        if len(set(self.candidate_load_names)) != len(self.candidate_load_names):
            raise ValueError("Candidate Load names must be unique.")

    def has_decisions(self, state: SystemSnapshot) -> bool:
        """Return whether the snapshot is a decision point without generating decisions."""
        return bool(self._waiting_loads(state))

    def generate(self, state: SystemSnapshot) -> Iterator[Decision]:
        """Yield entry alternatives and an explicit wait alternative."""
        waiting_loads = self._waiting_loads(state)
        for name in waiting_loads:
            yield ControlDecision(f"{state.root_model.model_path}.{name}", "?go")

        if waiting_loads:
            yield NoControl()

    def _waiting_loads(self, state: SystemSnapshot) -> list[str]:
        """Return eligible waiting loads for a decision-point snapshot."""
        if not isinstance(state, SystemSnapshot):
            raise TypeError("state must be a SystemSnapshot instance.")

        children = state.root_model.children

        if self.bridge_name not in children:
            raise ValueError(
                f"Bridge model {self.bridge_name!r} is missing from the snapshot."
            )

        missing_loads = [
            name for name in self.candidate_load_names if name not in children
        ]
        if missing_loads:
            raise ValueError(
                f"Candidate Load models are missing from the snapshot: {missing_loads!r}"
            )

        if state.x_buffer:
            return []

        bridge_state = children[self.bridge_name]
        if not bridge_state.has_model_state or not bridge_state.model_state:
            raise ValueError("Bridge snapshot does not contain a valid model state.")

        if bridge_state.model_state[0] != "I":
            return []

        load_states = {}
        for name in self.candidate_load_names:
            child = children[name]
            if not child.has_model_state or not child.model_state:
                raise ValueError(
                    f"Load {name!r} snapshot does not contain a valid model state."
                )
            load_states[name] = child.model_state

        if any(load_state[0] == "S" for load_state in load_states.values()):
            return []

        return [
            name
            for name, load_state in load_states.items()
            if load_state[0] == "W"
        ]
