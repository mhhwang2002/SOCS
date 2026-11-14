"""Colored Petri nets for DEVSph controllers."""

from copy import deepcopy

from socs.graph.agraph import AGraph

from .DEVS import DEVS
from .TESBase import PortValue


class _PetriNet:
    """Manage a colored marking and its port connections.

    Places hold ordered lists of token values. Input arcs bind one token each
    and repeated variable names require equal values on distinct tokens.
    Guards take a binding, and output arcs copy a bound value or evaluate an
    expression against the binding. The DEVS model controls when to fire.
    """

    def _init_petri_net(self):
        self.petri_graph = AGraph()
        self._initial_marking = {}
        self._marking = {}
        self._input_connections = {}
        self._output_connections = {}

    def add_place(self, name, tokens=()):
        """Declare a place and its initial tokens."""
        tokens = deepcopy(list(tokens))
        self.petri_graph.add_vertex(name, {"kind": "place"})
        self._initial_marking[name] = tokens
        self._marking[name] = deepcopy(tokens)

    def add_transition(self, name, *, guard=None):
        """Declare a transition with an optional guard(binding)."""
        self.petri_graph.add_vertex(name, {"kind": "transition", "guard": guard})

    def _require_kind(self, name, kind):
        if self.petri_graph.vertex_attrs(name).kind != kind:
            raise ValueError(f"{name!r} is not a Petri net {kind}.")

    def add_input_arc(self, place, transition, variable):
        self._require_kind(place, "place")
        self._require_kind(transition, "transition")
        self.petri_graph.add_edge(place, transition, {"variable": variable})

    def add_output_arc(self, transition, place, expression):
        """Emit a named bound token or the result of expression(binding)."""
        self._require_kind(transition, "transition")
        self._require_kind(place, "place")
        self.petri_graph.add_edge(transition, place, {"expression": expression})

    def connect_input_port(self, port, place, *, tokens=None, replace=False):
        """Connect an input to a place. tokens(value) yields incoming tokens."""
        self._require_kind(place, "place")
        if port not in self.IP:
            raise KeyError(f"Unknown input port {port!r}.")
        self._input_connections.setdefault(port, []).append((place, tokens, replace))

    def connect_output_port(self, transition, port, *, guard=None, value=None):
        """Connect a transition to an output. Callbacks take its binding."""
        self._require_kind(transition, "transition")
        if port not in self.OP:
            raise KeyError(f"Unknown output port {port!r}.")
        self._output_connections.setdefault(transition, []).append((port, guard, value))

    def receive_tokens(self, event):
        """Apply the declared input connections for a PortValue."""
        updates = [
            (place, deepcopy(list(tokens(event.value) if tokens else [event.value])), replace)
            for place, tokens, replace in self._input_connections.get(event.port, ())
        ]
        for place, tokens, replace in updates:
            if replace:
                self._marking[place] = tokens
            else:
                self._marking[place].extend(tokens)

    def transition_outputs(self, transition, binding):
        """Build output events for the supplied binding before it fires."""
        self._require_kind(transition, "transition")
        return [
            PortValue(port, value(deepcopy(binding)) if value else None)
            for port, guard, value in self._output_connections.get(transition, ())
            if guard is None or guard(deepcopy(binding))
        ]

    def tokens(self, place):
        """Return a detached view of a place's current tokens."""
        return tuple(deepcopy(self._marking[place]))

    def set_tokens(self, place, tokens):
        """Replace tokens in a place, for example on a DEVS input event."""
        self._require_kind(place, "place")
        self._marking[place] = deepcopy(list(tokens))

    def _matches(self, transition):
        self._require_kind(transition, "transition")
        arcs = list(self.petri_graph.in_edges(transition))
        guard = self.petri_graph.vertex_attrs(transition).guard

        def match(index, binding, consumed):
            if index == len(arcs):
                if guard is None or guard(deepcopy(binding)):
                    yield binding, consumed
                return

            arc = arcs[index]
            variable = arc.attrs.variable
            for position, token in enumerate(self._marking[arc.src]):
                slot = (arc.src, position)
                if slot in consumed or (variable in binding and binding[variable] != token):
                    continue
                yield from match(index + 1, {**binding, variable: token}, consumed + [slot])

        return match(0, {}, [])

    def enabled_bindings(self, transition):
        """Yield enabled bindings in input arc and token order."""
        for binding, _ in self._matches(transition):
            yield deepcopy(binding)

    def fire(self, transition, binding=None):
        """Fire the first matching binding, or raise if it is disabled."""
        for candidate, consumed in self._matches(transition):
            if binding is None or binding == candidate:
                break
        else:
            raise ValueError(f"No enabled binding for {transition!r}.")

        produced = []
        for arc in self.petri_graph.out_edges(transition):
            expression = arc.attrs.expression
            token = expression(deepcopy(candidate)) if callable(expression) else candidate[expression]
            produced.append((arc.dst, deepcopy(token)))

        for place, position in sorted(consumed, key=lambda slot: slot[1], reverse=True):
            del self._marking[place][position]
        for place, token in produced:
            self._marking[place].append(token)
        return deepcopy(candidate)


class DEVSPetriNet(_PetriNet, DEVS):
    """Use a colored Petri net in an atomic DEVS model."""

    def __init__(self, name, parent=None):
        super().__init__(name, parent)
        self._init_petri_net()

    def set_initial_state(self):
        self._marking = deepcopy(self._initial_marking)

    def get_state(self):
        return deepcopy(self._marking)

    def set_state(self, state):
        self._marking = deepcopy(state)
