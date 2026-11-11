"""Helpers for resolving and applying generic DEVS control decisions."""

from socs.devs.DEVSInterfaces import PortValue
from socs.devs.Simulatable import Simulatable

from .decision import ControlDecision


def resolve_model(root_model: Simulatable, model_path: str) -> Simulatable:
    """Resolve a model by its hierarchical path within a model tree."""
    matches = []

    def visit(model):
        if model.path == model_path:
            matches.append(model)
        for child in model.children:
            visit(child)

    visit(root_model)
    if not matches:
        raise ValueError(f"Target model {model_path!r} does not exist.")
    if len(matches) > 1:
        raise ValueError(f"Target model path {model_path!r} is ambiguous.")
    return matches[0]


def make_port_value(target_model: Simulatable, xevent):
    """Resolve a symbolic input event to a PortValue for the target model."""
    if isinstance(xevent, PortValue):
        if xevent.port not in target_model.IP:
            raise ValueError("The input event port must be registered on target_model.")
        return xevent

    if isinstance(xevent, str):
        port_name, value = xevent, None
    elif (
        isinstance(xevent, tuple)
        and len(xevent) == 2
        and isinstance(xevent[0], str)
    ):
        port_name, value = xevent
    else:
        raise TypeError(
            "xevent must be a PortValue, input-port name, or (port_name, value) pair."
        )

    if port_name not in target_model.IP:
        raise ValueError(
            f"Input port {port_name!r} does not exist on {target_model.path!r}."
        )
    return PortValue(port_name, value)


def apply_control(engine, decision: ControlDecision) -> None:
    """Queue the external input described by a ControlDecision."""
    if not isinstance(decision, ControlDecision):
        raise TypeError("decision must be a ControlDecision instance.")
    if decision.target_model is None:
        return

    target = resolve_model(engine.model, decision.target_model)
    event = make_port_value(target, decision.xevent)
    engine.inject(target, event)
