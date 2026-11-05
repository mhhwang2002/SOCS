"""Decision generation functions for the LogBridge optimization example."""

from collections.abc import Iterator, Sequence

from socs.decision.decision import ControlDecision, Decision
from socs.devs.Snapshot import SystemSnapshot


def _waiting_loads(
    state: SystemSnapshot,
    bridge_name: str = "LB",
    candidate_load_names: Sequence[str] = ("L11", "L21"),
) -> list[str]:
    """Return eligible waiting loads for a decision-point snapshot."""
    if not isinstance(state, SystemSnapshot):
        raise TypeError("state must be a SystemSnapshot instance.")
    if not candidate_load_names:
        raise ValueError("At least one candidate Load must be configured.")
    if len(set(candidate_load_names)) != len(candidate_load_names):
        raise ValueError("Candidate Load names must be unique.")

    children = state.root_model.children
    if bridge_name not in children:
        raise ValueError(f"Bridge model {bridge_name!r} is missing from the snapshot.")

    missing_loads = [name for name in candidate_load_names if name not in children]
    if missing_loads:
        raise ValueError(
            f"Candidate Load models are missing from the snapshot: {missing_loads!r}"
        )

    if state.x_buffer:
        return []

    bridge_state = children[bridge_name]
    if not bridge_state.has_model_state or not bridge_state.model_state:
        raise ValueError("Bridge snapshot does not contain a valid model state.")
    if bridge_state.model_state[0] != "I":
        return []

    load_states = {}
    for name in candidate_load_names:
        child = children[name]
        if not child.has_model_state or not child.model_state:
            raise ValueError(
                f"Load {name!r} snapshot does not contain a valid model state."
            )
        load_states[name] = child.model_state

    if any(load_state[0] == "S" for load_state in load_states.values()):
        return []

    return [
        name for name, load_state in load_states.items() if load_state[0] == "W"
    ]


def has_decisions(
    state: SystemSnapshot,
    bridge_name: str = "LB",
    candidate_load_names: Sequence[str] = ("L11", "L21"),
) -> bool:
    """Return whether the snapshot is a decision point."""
    return bool(_waiting_loads(state, bridge_name, candidate_load_names))


def generate_decisions(
    state: SystemSnapshot,
    bridge_name: str = "LB",
    candidate_load_names: Sequence[str] = ("L11", "L21"),
) -> Iterator[Decision]:
    """Yield bridge-entry alternatives and an explicit wait alternative."""
    waiting_loads = _waiting_loads(state, bridge_name, candidate_load_names)
    for name in waiting_loads:
        yield ControlDecision(f"{state.root_model.model_path}.{name}", "?go")
    if waiting_loads:
        yield ControlDecision(None)
