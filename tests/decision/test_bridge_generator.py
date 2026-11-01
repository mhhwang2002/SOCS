from dataclasses import dataclass
from collections.abc import Iterator

from socs.decision.decision import Decision
from socs.decision.generator import DecisionGenerator


@dataclass(frozen=True)
class BridgeState:
    bridge_occupied: bool
    waiting_vehicles: tuple[str, ...]


@dataclass(frozen=True)
class EnterBridge(Decision):
    vehicle: str


class BridgeDecisionGenerator(DecisionGenerator[BridgeState]):

    def generate(self, state: BridgeState) -> Iterator[Decision]:
        if state.bridge_occupied:
            return

        for vehicle in state.waiting_vehicles:
            yield EnterBridge(vehicle=vehicle)


# Unit Tests

def test_generate_decisions_for_waiting_vehicles():
    state = BridgeState(
        bridge_occupied=False,
        waiting_vehicles=("A", "B"),
    )

    generator = BridgeDecisionGenerator()
    decisions = generator.generate(state)

    first = next(decisions)
    assert first == EnterBridge(vehicle="A")

    second = next(decisions)
    assert second == EnterBridge(vehicle="B")


def test_no_decisions_when_bridge_is_occupied():
    state = BridgeState(
        bridge_occupied=True,
        waiting_vehicles=("A", "B"),
    )

    generator = BridgeDecisionGenerator()
    decisions = generator.generate(state)

    assert list(decisions) == []


def test_no_decisions_when_no_vehicle_is_waiting():
    state = BridgeState(
        bridge_occupied=False,
        waiting_vehicles=(),
    )

    generator = BridgeDecisionGenerator()

    assert list(generator.generate(state)) == []