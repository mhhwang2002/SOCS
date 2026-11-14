"""A two-floor elevator with acceleration, cruising, and braking."""

from argparse import ArgumentParser
from math import inf, isfinite

from socs.devs import cDEVSph
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import TESEnginBase


class Elevator(cDEVSph):
    """Travel 3 m between floors, reaching 1.5 m/s in half a second."""

    FLOOR_HEIGHT = 3
    TOP_SPEED = 1.5
    RAMP_TIME = 0.5

    def __init__(self, name="Elevator"):
        super().__init__(name, variables=dict(position=0.0, velocity=0.0, floor=1, target=1))
        self.add_input_ports("?floor")
        self.add_output_ports("!arrived")
        self.add_phase("Idle", tau=inf, initial=True)
        self.add_phase("Accelerating", tau=self.RAMP_TIME)
        self.add_phase("Cruising", tau=self.FLOOR_HEIGHT / self.TOP_SPEED - self.RAMP_TIME)
        self.add_phase("Braking", tau=self.RAMP_TIME)
        self.add_external_transition(
            "Idle", "Accelerating", input_port="?floor", reschedule=True,
            match=lambda floor: type(floor) is int and floor in (1, 2),
            guard=self.is_different_floor,
            action=self.set_target,
        )
        self.add_internal_transition("Accelerating", "Cruising")
        self.add_internal_transition("Cruising", "Braking")
        self.add_internal_transition(
            "Braking", "Idle", action=self.finish_trip,
            output=lambda model: PortValue("!arrived", model._variable.target),
        )

    @staticmethod
    def is_different_floor(model, event):
        return event.value != model._variable.floor

    @staticmethod
    def set_target(model, event):
        model._variable.target = event.value

    @staticmethod
    def finish_trip(model, event):
        target = model._variable.target
        model._variable.update(floor=target, position=(target - 1) * model.FLOOR_HEIGHT, velocity=0.0)

    def delta_cont(self, elapsed):
        """Evaluate motion at elapsed seconds after the current phase began."""
        if self.phase == "Idle":
            return
        state = self._variable
        direction = state.target - state.floor
        start = (state.floor - 1) * self.FLOOR_HEIGHT
        ramp_distance = self.TOP_SPEED * self.RAMP_TIME / 2
        acceleration = self.TOP_SPEED / self.RAMP_TIME
        if self.phase == "Accelerating":
            distance = acceleration * elapsed**2 / 2
            speed = acceleration * elapsed
        elif self.phase == "Cruising":
            distance = ramp_distance + self.TOP_SPEED * elapsed
            speed = self.TOP_SPEED
        else:
            distance = self.FLOOR_HEIGHT - ramp_distance + self.TOP_SPEED * elapsed
            distance -= acceleration * elapsed**2 / 2
            speed = self.TOP_SPEED - acceleration * elapsed
        state.position = start + direction * distance
        state.velocity = direction * speed


def request_floor(engine, floor):
    engine.XBuffer.append((engine.model, PortValue("?floor", floor)))
    engine.execute_one_step()


def run_console(engine):
    for floor in (2, 1):
        request_floor(engine, floor)
        while isfinite(engine.model.tNext):
            engine.execute_one_step()
            state = engine.model._variable
            print(
                f"t={engine.tNow:g} {engine.model.phase:12} "
                f"position={state.position:.2f} m velocity={state.velocity:.2f} m/s"
            )


def run_live(engine):
    """Open the optional graphical view for this engine."""
    if __package__:
        from ._elevator_view import run_live as show_elevator
    else:
        from _elevator_view import run_live as show_elevator

    show_elevator(engine)


def main():
    parser = ArgumentParser(description=__doc__)
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--live", action="store_true", help="Run in a live 2D window (default).")
    display.add_argument("--console", dest="live", action="store_false", help="Print a round trip in the console.")
    parser.set_defaults(live=True)
    args = parser.parse_args()
    engine = TESEnginBase(Elevator())
    engine.bVerbose = False
    if args.live:
        run_live(engine)
    else:
        run_console(engine)


if __name__ == "__main__":
    main()
