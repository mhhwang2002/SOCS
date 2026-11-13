"""Phase graphs for declarative and Python-defined atomic DEVS models."""

from collections.abc import Callable
from copy import deepcopy
from json import dumps

from socs.graph.agraph import AGraph
from socs.graph.attributes import AttrDict

from .DEVS import DEVS
from .TESBase import PortValue


def transition_line_style(attrs):
    """Map the transition's scheduling decision to a diagram line style."""
    decision = attrs.get("reschedule")
    if decision is False:
        return "dashed"
    if callable(decision):
        return "dotted"
    return "solid"


class DEVSph(DEVS):
    """Execute a declared phase graph or observe a subclass's DEVS methods.

    Subclasses implementing their own DEVS methods must maintain self.phase.
    Phase names are nonempty strings. Callback signatures are tau(model),
    match(value), guard(model, event), action(model, event), output(model),
    and reschedule(model, event). Internal callbacks receive event=None.
    """

    def __init__(self, name, parent=None, *, variables=None):
        super().__init__(name, parent)
        self.phase = None
        self._initial_phase = None
        self._initial_variables = deepcopy(AttrDict({} if variables is None else variables))
        self._variable = deepcopy(self._initial_variables)
        self.graph = AGraph()
        self.observed_graph = AGraph()
        self._observed_initial = None

    def add_phase(self, name, tau, *, initial=False):
        """Declare a phase with a constant or callable time advance."""
        if not isinstance(name, str) or not name:
            raise ValueError("Phase names must be nonempty strings.")
        if initial and self._initial_phase is not None:
            raise ValueError("The initial phase is already declared.")
        self.graph.add_vertex(name, {"tau": tau})
        if initial:
            self._initial_phase = self.phase = name

    def add_external_transition(
        self, src, dst, *, input_port,
        reschedule: bool | Callable[["DEVSph", PortValue], bool],
        match=None, guard=None, action=None,
    ):
        """Declare an input transition. Reschedule runs after action and phase update."""
        if input_port not in self.IP:
            raise KeyError(f"Unknown input port {input_port!r}.")
        if not isinstance(reschedule, bool) and not callable(reschedule):
            raise TypeError("reschedule must be a bool or a callable returning bool.")
        self.graph.add_edge(src, dst, dict(
            kind="external", port=input_port, reschedule=reschedule,
            match=match, guard=guard, action=action,
        ))

    def add_internal_transition(
        self, src, dst, *, guard=None, action=None, output=None,
    ):
        """Declare an internal transition with a port name, PortValue, or callback.

        Output is evaluated against the source state before the action runs.
        A callback returns a PortValue or None.
        """
        if isinstance(output, (str, PortValue)):
            port = output if isinstance(output, str) else output.port
            if port not in self.OP:
                raise KeyError(f"Unknown output port {port!r}.")
        elif output is not None and not callable(output):
            raise TypeError("Output must be a port name, a PortValue, a callback, or None.")
        self.graph.add_edge(src, dst, dict(
            kind="internal", guard=guard, action=action, output=output,
        ))

    def set_initial_state(self):
        if self._initial_phase is None:
            raise ValueError("Declare an initial phase with add_phase().")
        self.phase = self._initial_phase
        self._variable = deepcopy(self._initial_variables)

    def tau(self):
        value = self.graph.vertex_attrs(self.phase)["tau"]
        return value(self) if callable(value) else value

    def _enabled_edge(self, kind, event, port=None):
        enabled = []
        for edge in self.graph.out_edges(self.phase, {"kind": kind}):
            attrs = edge.attrs
            if kind == "external" and (
                attrs.port != port or (attrs.match is not None and not attrs.match(event.value))
            ):
                continue
            if attrs.guard is None or attrs.guard(self, event):
                enabled.append(edge)
        if len(enabled) > 1:
            raise ValueError(f"Multiple enabled {kind} transitions from {self.phase!r}.")
        return next(iter(enabled), None)

    def _apply_edge(self, edge, event):
        if edge.attrs.action is not None:
            edge.attrs.action(self, event)
        self.phase = edge.dst

    def delta_x(self, x) -> bool:
        if not isinstance(x, PortValue) or x.port not in self.IP:
            return False
        edge = self._enabled_edge("external", x, x.port)
        if edge is None:
            return False
        self._apply_edge(edge, x)
        reschedule = edge.attrs.reschedule
        decision = reschedule(self, x) if callable(reschedule) else reschedule
        if not isinstance(decision, bool):
            raise TypeError("reschedule must return a bool.")
        return decision

    def delta_y(self):
        edge = self._enabled_edge("internal", None)
        if edge is None:
            raise ValueError(f"No enabled internal transition from {self.phase!r}.")
        output = edge.attrs.output
        if callable(output):
            output = output(self)
        elif isinstance(output, str):
            output = PortValue(self.OP[output])
        elif isinstance(output, PortValue):
            output = deepcopy(output)
        if output is not None and (
            not isinstance(output, PortValue) or output.port not in self.OP
        ):
            raise ValueError("Internal output must use a registered output port.")
        self._apply_edge(edge, None)
        return output

    def get_state(self):
        """Return the phase and variables for the existing snapshot protocol."""
        return self.phase, self._variable

    def set_state(self, state):
        self.phase, variables = state
        self._variable = AttrDict(variables)

    def get_statistics_state(self):
        return self.phase

    def _ensure_observed_phase(self, phase):
        if not isinstance(phase, str) or not phase:
            raise ValueError("DEVSph models must maintain a nonempty string self.phase.")
        if phase not in self.observed_graph.V:
            self.observed_graph.add_vertex(phase)

    def _observe(self, src, kind, event, *, reschedule=None):
        dst = self.phase
        self._ensure_observed_phase(src)
        self._ensure_observed_phase(dst)
        port = event.port if isinstance(event, PortValue) else None
        attrs = dict(kind=kind, port=port)
        if kind == "external":
            attrs["reschedule"] = bool(reschedule)
        edges = self.observed_graph.edges(src, dst, attrs)
        edge = edges[0] if edges else self.observed_graph.add_edge(src, dst, dict(attrs, count=0))
        edge.attrs.count += 1

    def set_init_total_state(self, t):
        super().set_init_total_state(t)
        self.observed_graph = AGraph()
        self._ensure_observed_phase(self.phase)
        self._observed_initial = self.phase

    def Delta_x(self, x, t):
        src = self.phase
        reschedule = super().Delta_x(x, t)
        if not self.graph.V:
            self._observe(src, "external", x, reschedule=reschedule)
        return reschedule

    def Delta_y(self, t):
        src = self.phase
        output = super().Delta_y(t)
        if not self.graph.V:
            self._observe(src, "internal", output)
        return output

    def diagram_edges(self):
        """Yield visible transitions, omitting declared no-op self-loops."""
        graph = self.graph if self.graph.V else self.observed_graph
        for edge in graph.edges():
            attrs = edge.attrs
            if (
                edge.src == edge.dst and attrs.kind == "external"
                and attrs.get("reschedule") is False
                and "action" in attrs and attrs.action is None
                and attrs.get("guard") is None and attrs.get("match") is None
            ):
                continue
            yield edge

    def to_dot(self):
        """Return the declared phase graph or the code-defined model's observed graph."""
        graph = self.graph if self.graph.V else self.observed_graph
        initial = self._initial_phase if self.graph.V else self._observed_initial
        ids = {phase: f"p{index}" for index, phase in enumerate(graph.V)}
        lines = ["digraph DEVSph {", f"  label={dumps(str(self.name))};", "  rankdir=LR;"]
        for phase, attrs in graph.V.items():
            label = phase
            styles = []
            if "tau" in attrs:
                advance = attrs["tau"]
                label += "\nτ = " + ("dynamic" if callable(advance) else str(advance))
                if advance == float("inf"):
                    styles.append("dashed")
            if phase == self.phase:
                styles.append("filled")
            style = f', style="{",".join(styles)}"' if styles else ""
            fill = ', fillcolor="lightblue"' if phase == self.phase else ""
            lines.append(f"  {ids[phase]} [label={dumps(label, ensure_ascii=False)}{style}{fill}];")
        if initial in ids:
            lines.extend(["  initial [shape=point];", f"  initial -> {ids[initial]};"])
        for index, (ports, direction) in enumerate(((self.IP, "Inputs"), (self.OP, "Outputs"))):
            if ports:
                label = "\n".join((direction, *ports))
                lines.append(f"  ports{index} [shape=box, label={dumps(label)}];")
        for edge in self.diagram_edges():
            attrs = edge.attrs
            label = attrs.kind
            port = attrs.get("port") or attrs.get("output")
            if isinstance(port, str):
                label += f" {port}"
            elif isinstance(port, PortValue):
                label += f" {port}"
            elif callable(port):
                label += " (dynamic output)"
            if attrs.get("guard") is not None:
                label += f" [{attrs.guard.__name__}]" if hasattr(attrs.guard, "__name__") else " [guard]"
            if attrs.get("action") is not None:
                name = getattr(attrs.action, "__name__", type(attrs.action).__name__)
                label += f" / action: {name}"
            if "reschedule" in attrs:
                label += f" / reschedule={attrs.reschedule}" if not callable(attrs.reschedule) else " / reschedule=dynamic"
            if "count" in attrs:
                label += f" ×{attrs.count}"
            style = f", style={transition_line_style(attrs)}"
            lines.append(f"  {ids[edge.src]} -> {ids[edge.dst]} [label={dumps(label, ensure_ascii=False)}{style}];")
        return "\n".join((*lines, "}")) + "\n"
