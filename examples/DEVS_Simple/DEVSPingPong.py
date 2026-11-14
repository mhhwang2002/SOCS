"""Ping-pong example using the deterministic DEVS model base."""

from socs.devs.DEVS import DEVS
from socs.devs.CoupledDEVS import CoupledDEVS
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import TESEnginThread


class Player(DEVS):
    """Ping-pong player that sends after a delay and then waits for a reply."""

    def __init__(self, name, initial_phase, send_time, parent=None):
        super().__init__(name, parent)
        self.initial_phase = initial_phase
        self.send_time = send_time
        self.add_input_ports("?r")
        self.add_output_ports("!s")
        self.phase = initial_phase

    def set_initial_state(self):
        """Initialize the player's phase."""
        self.phase = self.initial_phase

    def tau(self):
        """Return the time until the next internal event."""
        if self.phase == "S":
            return self.send_time
        return float("inf")

    def delta_x(self, x):
        """Handle a received ball and request rescheduling when needed."""
        if x.port == "?r" and self.phase == "W":
            self.phase = "S"
            return True
        return False

    def delta_y(self):
        """Send the ball and transition to the waiting phase."""
        if self.phase == "S":
            self.phase = "W"
            return PortValue(self.OP["!s"])
        return None

    def get_state(self):
        """Return the current phase."""
        return self.phase

    def set_state(self, state):
        """Set the current phase."""
        self.phase = state


def make_model():
    """Build the coupled ping-pong model."""
    player_a = Player("A", "S", 0.1)
    player_b = Player("B", "W", 0.2)

    model = CoupledDEVS("PingPong")
    model.add_components(player_a, player_b)
    model.add_output_ports("!ping", "!pong")
    model.add_couples(
        ((player_a, "!s"), (player_b, "?r")),
        ((player_b, "!s"), (player_a, "?r")),
        ((player_a, "!s"), (model, "!ping")),
        ((player_b, "!s"), (model, "!pong")),
    )
    return model


def run_simulation(model):
    """Run the model in the interactive simulation engine."""
    engine = TESEnginThread(model)
    engine.StepByStep = True
    engine.sfactor = float("inf")
    engine.run_interactive_menu()


def check_liveness(KDAG):
    """Check whether a kernel DAG contains a state where players meet."""
    for key in KDAG.V:
        cluster = KDAG.kv[key]
        if len(cluster) <= 1:
            continue
        model_e = {}
        model_s = {}
        for vertex in cluster:
            for name in vertex.s:
                model_e.setdefault(name, False)
                model_s.setdefault(name, False)
                phase = vertex.s[name][0][0]
                if phase == "W":
                    model_e[name] = True
                elif phase == "S":
                    model_s[name] = True
        if model_e and all(model_e[name] and model_s[name] for name in model_e):
            return True
    return False


def run_verification(model):
    """Placeholder for the legacy verification workflow."""
    inputs = []
    # The legacy VeriEngine integration is intentionally left disabled.


if __name__ == "__main__":
    model = make_model()
    print("is_network_of(Player) =", model.is_network_of(Player))

    running = True
    while running:
        command = input("(s)im, (v)eri, e(x)it> ").strip().lower()
        if command in ("s", "sim"):
            run_simulation(model)
        elif command in ("v", "veri"):
            run_verification(model)
        elif command in ("x", "exit"):
            running = False
        else:
            print(f"{command!r} is an invalid input.")
    print("good bye~!")
