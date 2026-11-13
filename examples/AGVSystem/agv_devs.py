"""Fixed-guidepath AGV example using the SOCS FD-DEVS engine."""
from copy import deepcopy
from math import inf

from socs.devs import CoupledDEVS, DEVS
from socs.devs.TESBase import PortValue
from socs.devs.TESEngin import ExecutionResult, TESEnginBase

from .layout import DWELL, POINTS, SEGMENTS, VEHICLES, VEHICLE_BY_NAME

from socs.graph.agraph import AGraph


def make_network():
    graph = AGraph()
    for name, (x, y) in POINTS.items():
        graph.add_vertex(name, {"x_m": x, "y_m": y, "dwell_s": DWELL.get(name, 0)})
    for source, segment in SEGMENTS.items():
        graph.add_edge(source, segment.target, {
            "resource": f"SEG_{source}", "length_m": segment.length,
        })
    return graph


class Atomic(DEVS):
    def get_state(self):
        return deepcopy(self.state)

    def set_state(self, state):
        self.state = deepcopy(state)


class ControlPoint(Atomic):
    """Report arrival immediately and request departure after service."""

    def __init__(self, name):
        super().__init__(name)
        self.addInputPorts("?arrive", "?grant")
        self.addOutputPorts("!traffic", "!start")
        self.initial = next((v.name for v in VEHICLES if v.start == name), None)

    def set_initial_state(self):
        self.state = ("ready", self.initial) if self.initial else ("idle", None)

    def tau(self):
        phase, _ = self.state
        return {"idle": inf, "service": DWELL.get(self.name, 0)}.get(phase, 0)

    def delta_x(self, event):
        phase, _ = self.state
        if phase != "idle":
            raise RuntimeError(f"{self.name}: unexpected input during {phase}")
        next_phase = "arrived" if event.port == "?arrive" else "depart"
        self.state = (next_phase, event.value)
        return True

    def delta_y(self):
        phase, vehicle = self.state
        self.state = ("service", vehicle) if phase == "arrived" else ("idle", None)
        if phase == "depart":
            return PortValue(self.OP["!start"], vehicle)
        kind = "arrive" if phase == "arrived" else "ready"
        return PortValue(self.OP["!traffic"], (kind, self.name, vehicle))


class PathSegment(Atomic):
    """Advance the vehicle front and emit clearance when the rear follows."""

    def __init__(self, source):
        super().__init__(f"SEG_{source}")
        self.segment = SEGMENTS[source]
        self.addInputPorts("?start")
        self.addOutputPorts("!traffic", "!arrive")

    def set_initial_state(self):
        self.state = ("idle", None)

    def delta_x(self, event):
        if self.state[0] != "idle":
            raise RuntimeError(f"{self.name} already has a vehicle")
        self.state = ("enter", event.value)
        return True

    def tau(self):
        phase, name = self.state
        if phase == "idle":
            return inf
        vehicle = VEHICLE_BY_NAME[name]
        if phase == "clear":
            return vehicle.length / vehicle.speed
        if phase == "arrive":
            return (self.segment.length - vehicle.length) / vehicle.speed
        return 0

    def delta_y(self):
        phase, name = self.state
        if phase == "arrive":
            self.state = ("idle", None)
            return PortValue(self.OP["!arrive"], name)
        self.state = ("clear" if phase == "enter" else "arrive", name)
        return PortValue(self.OP["!traffic"], (phase, self.segment.source, name))


class Controller(Atomic):
    """Reserve an exclusive segment and downstream stopping point together."""

    def __init__(self):
        super().__init__("Controller")
        self.network = make_network()
        self.addInputPorts("?traffic")
        self.addOutputPorts(*(f"!{point}" for point in POINTS))

    def set_initial_state(self):
        occupied = {
            resource: vehicle.name
            for vehicle in VEHICLES
            for resource in (vehicle.start, self.network.in_edges(vehicle.start)[0].attrs.resource)
        }
        self.state = {
            "occupied": occupied, "reserved": dict(occupied),
            "waiting": {}, "events": [],
        }

    def _available(self, source):
        reserved = self.state["reserved"]
        edge = self.network.out_edges(source)[0]
        return edge.attrs.resource not in reserved and edge.dst not in reserved

    def tau(self):
        return 0 if any(map(self._available, self.state["waiting"])) else inf

    def delta_x(self, event):
        was_active = self.tau() == 0
        kind, point, vehicle = event.value
        self.state["events"].append(event.value)
        occupied, reserved = self.state["occupied"], self.state["reserved"]
        if kind == "ready":
            self.state["waiting"][point] = vehicle
        elif kind in ("enter", "arrive"):
            resource = f"SEG_{point}" if kind == "enter" else point
            if reserved.get(resource) != vehicle or resource in occupied:
                raise RuntimeError(f"Invalid {kind} on {resource} by {vehicle}")
            occupied[resource] = vehicle
        elif kind == "clear":
            for resource in (point, self.network.in_edges(point)[0].attrs.resource):
                if occupied.pop(resource) != vehicle or reserved.pop(resource) != vehicle:
                    raise RuntimeError(f"Invalid clearance by {vehicle} on {resource}")
        return was_active != (self.tau() == 0)

    def delta_y(self):
        waiting = self.state["waiting"]
        source = next(point for point in waiting if self._available(point))
        vehicle = waiting.pop(source)
        reserved = self.state["reserved"]
        edge = self.network.out_edges(source)[0]
        reserved[edge.attrs.resource] = reserved[edge.dst] = vehicle
        self.state["events"].append(("grant", source, vehicle))
        return PortValue(self.OP[f"!{source}"], vehicle)


def make_model():
    model = CoupledDEVS("AGVSystem")
    controller = Controller()
    points = {name: ControlPoint(name) for name in POINTS}
    paths = {name: PathSegment(name) for name in POINTS}
    model.addComponents(controller, *points.values(), *paths.values())
    for source, segment in SEGMENTS.items():
        point, path = points[source], paths[source]
        model.addCouple((controller, f"!{source}"), (point, "?grant"))
        model.addCouple((point, "!start"), (path, "?start"))
        model.addCouple((path, "!arrive"), (points[segment.target], "?arrive"))
        model.addCouple((point, "!traffic"), (controller, "?traffic"))
        model.addCouple((path, "!traffic"), (controller, "?traffic"))
    return model, controller


def simulate(seconds=90):
    model, controller = make_model()
    engine = TESEnginBase(model, TimeSimulationEnd=seconds)
    engine.bVerbose = False
    engine.sfactor = inf
    trace = []
    read = 0
    while engine.executeOneStep() != ExecutionResult.TimeBoundReached:
        events = controller.state["events"]
        trace.extend((engine.tNow, *event) for event in events[read:])
        read = len(events)
    return trace


if __name__ == "__main__":
    trace = simulate()
    for time, kind, resource, vehicle in trace:
        print(f"{time:6.2f}  {kind:7}  {resource:4}  {vehicle}")
