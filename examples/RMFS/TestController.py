"""Send fixed robot paths at time zero and print reports without traffic control."""

from math import inf

from socs.devs import DEVSph
from socs.devs.TESBase import PortValue


class TestController(DEVSph):
    def __init__(self, path=None, parent=None, *, robot_paths=None):
        super().__init__("TestController", parent)
        if robot_paths is None:
            robot_paths = {"R1": path}
        self.command_paths = {
            name: tuple(tuple(point) for point in route)
            for name, route in robot_paths.items()
        }
        if not self.command_paths or any(not route for route in self.command_paths.values()):
            raise ValueError("Each robot needs a nonempty path.")
        self.command_path = next(iter(self.command_paths.values()))
        self.add_phase("Command", tau=0, initial=True)
        self.add_phase("Wait", tau=inf)
        self.add_internal_transition("Command", "Wait")
        for name in self.command_paths:
            self.add_output_ports(f"!{name}")
            self.add_input_ports(
                f"?{name}", f"?arrived_{name}",
                f"?rotation_started_{name}", f"?rotation_finished_{name}",
                f"?collision_{name}",
            )
            for phase in ("Command", "Wait"):
                for port in (
                    f"?{name}", f"?arrived_{name}",
                    f"?rotation_started_{name}", f"?rotation_finished_{name}",
                    f"?collision_{name}",
                ):
                    self.add_external_transition(
                        phase, phase, input_port=port, reschedule=True,
                        action=TestController.report_event,
                    )

    def delta_y(self):
        outputs = [
            PortValue(f"!{name}", route)
            for name, route in self.command_paths.items()
        ]
        super().delta_y()
        return outputs[0] if len(outputs) == 1 else outputs
    def delta_x(self, event):
        super().delta_x(event)
        return True

    def report_event(self, event):
        print(f"({event.port}, {event.value}) at={self.tNow}")
