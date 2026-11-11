import pytest

from socs.devs import CoupledDEVS, DEVS
from socs.devs.DEVSInterfaces import PortValue


class AtomicModelStub(DEVS):
    def __init__(self, name, interval=float("inf")):
        super().__init__(name)
        self.interval = interval
        self.state = None
        self.received = []

    def delta_0(self):
        self.state = None
        self.received = []

    def tau(self):
        return self.interval

    def delta_x(self, x):
        self.received.append(x)
        return False

    def delta_y(self):
        return None

    def set_state(self, state):
        self.state = state

    def get_state(self):
        return self.state


def test_add_named_input_and_output_ports():
    model = AtomicModelStub("Generator")
    model.addInputPorts("control")
    model.addOutputPorts("vehicles")

    assert "control" in model.IP
    assert "vehicles" in model.OP
    assert model.IP["control"] == "control"
    assert model.OP["vehicles"] == "vehicles"


def test_duplicate_port_name_does_not_replace_existing_port():
    model = AtomicModelStub("Generator")
    model.addOutputPorts("vehicles")
    original_port = model.OP["vehicles"]

    model.addOutputPorts("vehicles")

    assert model.OP["vehicles"] is original_port


def test_add_coupling_between_child_models():
    system = CoupledDEVS("TrafficSystem")
    generator = AtomicModelStub("Generator")
    intersection = AtomicModelStub("Intersection")
    generator.addOutputPorts("vehicles")
    intersection.addInputPorts("vehicles")
    system.add_child(generator)
    system.add_child(intersection)

    source_port = (generator, "vehicles")
    target_port = (intersection, "vehicles")
    system.addCouple(source_port, target_port)

    assert target_port in system.Z[source_port]


def test_add_couple_requires_endpoints():
    system = CoupledDEVS("TrafficSystem")

    with pytest.raises(TypeError):
        system.addCouple("vehicles", "vehicles")


def test_external_input_coupling_routes_to_child():
    system = CoupledDEVS("TrafficSystem")
    system.addInputPorts("vehicle_request")
    generator = AtomicModelStub("VehicleGenerator")
    generator.addInputPorts("request")
    system.add_child(generator)

    system.addCouple(
        (system, "vehicle_request"),
        (generator, "request"),
    )
    system.Delta_0(0.0)

    payload = {"count": 3}
    system.Delta_x(PortValue(system.IP["vehicle_request"], payload), 0.0)

    assert len(generator.received) == 1
    received = generator.received[0]
    assert isinstance(received, PortValue)
    assert received.port == "request"
    assert received.value is payload


def test_internal_output_coupling_routes_to_child_input():
    system = CoupledDEVS("TrafficSystem")
    generator = AtomicModelStub("Generator")
    intersection = AtomicModelStub("Intersection")
    generator.addOutputPorts("vehicles")
    intersection.addInputPorts("vehicles")
    system.add_child(generator)
    system.add_child(intersection)
    system.addCouple((generator, "vehicles"), (intersection, "vehicles"))
    system.Delta_0(0.0)

    payload = {"vehicle_id": 101}
    translated = system.translateOutputEvent(
        generator, PortValue(generator.OP["vehicles"], payload), 0.0
    )

    assert translated == []
    assert len(intersection.received) == 1
    received = intersection.received[0]
    assert received.port == "vehicles"
    assert received.value is payload


def test_external_output_coupling_reaches_coupled_output_port():
    system = CoupledDEVS("TrafficSystem")
    intersection = AtomicModelStub("Intersection")
    intersection.addOutputPorts("departures")
    system.addOutputPorts("departures")
    system.add_child(intersection)
    system.addCouple(
        (intersection, "departures"),
        (system, "departures"),
    )

    payload = {"vehicle_id": 101}
    translated = system.translateOutputEvent(
        intersection, PortValue(intersection.OP["departures"], payload), 0.0
    )

    assert len(translated) == 1
    assert translated[0].port == "departures"
    assert translated[0].value is payload


def test_route_output_supports_multiple_destinations():
    system = CoupledDEVS("TrafficSystem")
    generator = AtomicModelStub("Generator")
    first = AtomicModelStub("Intersection_01")
    second = AtomicModelStub("Intersection_02")
    generator.addOutputPorts("vehicles")
    first.addInputPorts("vehicles")
    second.addInputPorts("vehicles")
    for model in (generator, first, second):
        system.add_child(model)
    system.addCouple((generator, "vehicles"), (first, "vehicles"))
    system.addCouple((generator, "vehicles"), (second, "vehicles"))
    system.Delta_0(0.0)

    payload = {"vehicle_id": 101}
    system.translateOutputEvent(
        generator, PortValue(generator.OP["vehicles"], payload), 0.0
    )

    assert len(first.received) == 1
    assert len(second.received) == 1
    assert first.received[0].value is payload
    assert second.received[0].value is payload


def test_unconnected_output_does_not_reach_coupled_boundary():
    system = CoupledDEVS("TrafficSystem")
    generator = AtomicModelStub("Generator")
    generator.addOutputPorts("vehicles")
    system.add_child(generator)
    event = PortValue("vehicles", {"vehicle_id": 101})

    translated = system.translateOutputEvent(generator, event, 0.0)

    assert translated == []
