"""One DEVSph controller containing TrafficManager and the shared RoadMap."""

from math import inf

from socs.devs import DEVSph
from socs.devs.TESBase import PortValue

from .RoadMap import RoadMap
from .TrafficManager import TrafficManager


class RMFSController(DEVSph):
    def __init__(self, paths, tuple_path, *, planning_error=None,
                 event_release=False, footprints=None, command_blocks=None):
        super().__init__("RMFSController")
        self.planning_error = planning_error
        self.roadmap = RoadMap(paths)
        self.traffic_manager = TrafficManager(self.roadmap, tuple_path,
                                             event_release=event_release,footprints=footprints,
                                             command_blocks=command_blocks)
        self.add_phase("Command", tau=0, initial=True)
        self.add_phase("Wait", tau=inf)
        self.add_internal_transition("Command", "Wait")
        for name in paths:
            self.add_output_ports(f"!{name}")
            self.add_input_ports(f"?{name}", f"?arrived_{name}",
                                 f"?rotation_started_{name}", f"?rotation_finished_{name}",
                                 f"?collision_{name}")

    def set_initial_state(self):
        super().set_initial_state()
        self.traffic_manager.reset(self.tNow)
        if self.planning_error:
            self.traffic_manager.done = False
            self.traffic_manager.halted = True
        self.phase = "Command" if self.traffic_manager.command_buffer else "Wait"

    def delta_x(self, event):
        # A report can fill the command buffer. The target phase depends on
        # the post-report buffer, so this hook implements the external graph.
        if event is None:
            return False
        manager = self.traffic_manager
        for name in self.roadmap.paths:
            if event.port == f"?arrived_{name}":
                manager.arrived(name, event.value, self.tNow)
                break
            if event.port == f"?{name}":
                manager.leaving(name, event.value, self.tNow)
                break
            if event.port in (f"?rotation_started_{name}", f"?rotation_finished_{name}"):
                kind = "rotation_started" if "started" in event.port else "rotation_finished"
                self.roadmap.report(name, kind, event.value, self.tNow)
                break
            if event.port == f"?collision_{name}":
                manager.halt(name, event.value, self.tNow)
                break
        else:
            raise ValueError(f"Unknown controller report {event.port}.")
        old_phase = self.phase
        self.phase = "Command" if manager.command_buffer else "Wait"
        return self.phase != old_phase or self.phase == "Command"

    def delta_y(self):
        outputs = [PortValue(f"!{command['robot']}", command["path"])
                   for command in self.traffic_manager.emit_commands(self.tNow)]
        super().delta_y()
        return outputs or None

    def get_snapshot(self):
        return dict(phase=self.phase, manager=self.traffic_manager.get_snapshot())

    def set_snapshot(self, state):
        self.phase = state["phase"]
        self.traffic_manager.set_snapshot(state["manager"])

    def get_state(self):
        return self.get_snapshot()

    def set_state(self, state):
        self.set_snapshot(state)
