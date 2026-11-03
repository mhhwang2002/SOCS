from collections.abc import Iterator, Sequence

from socs.decision.generator import DecisionGenerator
from socs.devs.Snapshot import SystemSnapshot

from .decisions import EnterBridgeDecision


class LogBridgeDecisionGenerator(DecisionGenerator[SystemSnapshot]):
    """Generate feasible bridge-entry decisions from a simulation snapshot."""

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

    def generate(
        self,
        state: SystemSnapshot,
    ) -> Iterator[EnterBridgeDecision]:
        """Yield one decision for each eligible source Load."""

        if not isinstance(state, SystemSnapshot):
            raise TypeError("state must be a SystemSnapshot instance.")

        root = state.root_model
        children = root.children

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

        # Do not authorize another action while an external input is queued.
        if state.x_buffer:
            return

        bridge_state = children[self.bridge_name]
        if not bridge_state.has_model_state or not bridge_state.model_state:
            raise ValueError("Bridge snapshot does not contain a valid model state.")

        bridge_phase = bridge_state.model_state[0]
        if bridge_phase != "I":
            return

        load_states = {
            name: children[name].model_state
            for name in self.candidate_load_names
        }

        for name, load_state in load_states.items():
            if not children[name].has_model_state or not load_state:
                raise ValueError(
                    f"Load {name!r} snapshot does not contain a valid model state."
                )

        # A Load in phase S has already received authorization and is about
        # to transfer its walker to the Bridge.
        if any(load_state[0] == "S" for load_state in load_states.values()):
            return

        for name, load_state in load_states.items():
            if load_state[0] == "W":
                yield EnterBridgeDecision(load_name=name)
