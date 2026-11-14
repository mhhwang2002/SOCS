"""AGV system with colored tokens for exclusive guidepath reservations."""

from math import inf

from socs.devs import CoupledDEVS, DEVSphPetriNet


from .agv_devs import ControlPoint, PathSegment
from .layout import POINTS, SEGMENTS, VEHICLES


class PetriController(DEVSphPetriNet):
    def __init__(self):
        self.incoming = {segment.target: f"SEG_{source}" for source, segment in SEGMENTS.items()}
        initial = {
            resource: vehicle.name
            for vehicle in VEHICLES
            for resource in (vehicle.start, self.incoming[vehicle.start])
        }
        super().__init__("Controller", variables={"occupied": initial, "events": []})
        self.resources = (*POINTS, *(f"SEG_{source}" for source in SEGMENTS))
        self.add_input_ports("?traffic")
        self.add_output_ports(*(f"!{source}" for source in SEGMENTS))
        self.add_phase("Idle", inf, initial=True)
        self.add_phase("Grant", 0)
        self.add_place("Waiting")
        for resource in self.resources:
            self.add_place(("Free", resource), [] if resource in initial else [True])
            self.add_place(("Reserved", resource), [initial[resource]] if resource in initial else [])
            release = ("Release", resource)
            self.add_transition(release)
            self.add_input_arc(("Reserved", resource), release, "vehicle")
            self.add_output_arc(release, ("Free", resource), lambda binding: True)
        for source, segment in SEGMENTS.items():
            grant = ("Grant", source)
            self.add_transition(grant, guard=lambda b, source=source: b["request"][0] == source)
            self.add_input_arc("Waiting", grant, "request")
            for resource in (f"SEG_{source}", segment.target):
                self.add_input_arc(("Free", resource), grant, "permit")
                self.add_output_arc(grant, ("Reserved", resource), lambda b: b["request"][1])
            self.connect_output_port(grant, f"!{source}", value=lambda b: b["request"][1])

    @property
    def state(self):
        """Expose the existing traffic monitor's state format."""
        return {
            "waiting": dict(self.tokens("Waiting")),
            "reserved": {
                resource: tokens[0]
                for resource in self.resources
                if (tokens := self.tokens(("Reserved", resource)))
            },
            "occupied": self._variable.occupied,
            "events": self._variable.events,
        }

    def next_grant(self):
        for source, vehicle in self.tokens("Waiting"):
            transition = ("Grant", source)
            binding = next(self.enabled_bindings(transition), None)
            if binding is not None:
                return transition, binding
        return None

    def tau(self):
        return 0 if self.phase == "Grant" else inf

    def delta_x(self, event):
        previous = self.phase
        kind, point, vehicle = event.value
        occupied = self._variable.occupied
        if kind == "ready":
            self.set_tokens("Waiting", (*self.tokens("Waiting"), (point, vehicle)))
        elif kind in ("enter", "arrive"):
            resource = f"SEG_{point}" if kind == "enter" else point
            if self.tokens(("Reserved", resource)) != (vehicle,) or resource in occupied:
                raise RuntimeError(f"Invalid {kind} on {resource} by {vehicle}")
            occupied[resource] = vehicle
        elif kind == "clear":
            resources = (point, self.incoming[point])
            for resource in resources:
                if occupied.get(resource) != vehicle or self.tokens(("Reserved", resource)) != (vehicle,):
                    raise RuntimeError(f"Invalid clearance by {vehicle} on {resource}")
            for resource in resources:
                self.fire(("Release", resource), {"vehicle": vehicle})
                del occupied[resource]
        self._variable.events.append(event.value)
        self.phase = "Grant" if self.next_grant() else "Idle"
        return self.phase != previous

    def delta_y(self):
        transition, binding = self.next_grant()
        output, = self.transition_outputs(transition, binding)
        self.fire(transition, binding)
        self._variable.events.append(("grant", *binding["request"]))
        self.phase = "Grant" if self.next_grant() else "Idle"
        return output


def make_model():
    model = CoupledDEVS("AGVSystemPetri")
    controller = PetriController()
    points = {name: ControlPoint(name) for name in POINTS}
    paths = {name: PathSegment(name) for name in SEGMENTS}
    model.add_components(controller, *points.values(), *paths.values())
    for source, segment in SEGMENTS.items():
        point, path = points[source], paths[source]
        model.add_couple((controller, f"!{source}"), (point, "?grant"))
        model.add_couple((point, "!start"), (path, "?start"))
        model.add_couple((path, "!arrive"), (points[segment.target], "?arrive"))
        model.add_couple((point, "!traffic"), (controller, "?traffic"))
        model.add_couple((path, "!traffic"), (controller, "?traffic"))
    return model, controller
