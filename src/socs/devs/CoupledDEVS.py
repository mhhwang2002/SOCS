#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################

"""Coupled DEVS model for composing atomic and coupled components."""

from .TESBase import PortValue, TES
from .DEVS import DEVS
from .ModelRegistry import ModelRegistry

class CoupledDEVS(TES):
    """Coupled DEVS model containing child components and port couplings."""

    def __init__(self, name, parent=None, tl=0, tn=float("inf")):
        super().__init__(name, parent, tl, tn)
        self.Z = {}
        self.IMM = set()
        self._model_registry = None

        if parent is None:
            self._model_registry = ModelRegistry()
            self._model_registry.register(self)

    @property
    def M(self):
        """Return the coupled model's child components."""
        return self.children
    
    @property
    def model_registry(self):
        """Return the root model registry."""
        if self.parent is None:
            return self._model_registry

        return self.parent.model_registry

    def add_child(self, child):
        """Add a coupled or FD-DEVS child model."""
        if not isinstance(child, (CoupledDEVS, DEVS)):
            raise TypeError(
                "A CoupledDEVS child must be a CoupledDEVS or FD-DEVS instance."
            )

        super().add_child(child)

        try:
            self.model_registry.register(child)
        except Exception:
            self.children.remove(child)
            child.parent = None
            raise

        self.update_tn(update_parent=True)

    def add_components(self, *args):
        """Add child components using the legacy method name."""
        for component in args:
            self.add_child(component)

    def remove_child(self, child):
        """Remove a child model and unregister its subtree."""
        if not isinstance(child, (CoupledDEVS, DEVS)):
            raise TypeError(
                "A CoupledDEVS child must be a CoupledDEVS or FD-DEVS instance."
            )

        if child.parent is not self or not any(
            existing is child for existing in self.children
        ):
            raise ValueError("Child does not belong to this coupled model.")

        # Unregister the complete subtree before detaching it.
        self.model_registry.unregister(child, recursive=True)

        super().remove_child(child)
        self._remove_child_couplings(child)

        self.update_tn(update_parent=True)

    def move_child_to(self, child, new_parent):
        """Move a child model to another parent within the same system."""
        if not isinstance(child, (CoupledDEVS, DEVS)):
            raise TypeError(
                "Child must be a CoupledDEVS or FD-DEVS instance."
            )

        if not isinstance(new_parent, CoupledDEVS):
            raise TypeError("New parent must be a CoupledDEVS instance.")

        old_parent = self

        # Validate the source relationship.
        if child.parent is not old_parent or not any(
            existing is child for existing in old_parent.children
        ):
            raise ValueError("Child does not belong to this coupled model.")

        if old_parent is new_parent:
            return

        # Both parents must belong to the same system.
        if old_parent.model_registry is not new_parent.model_registry:
            raise ValueError("Models must belong to the same system.")

        # Prevent cycles.
        ancestor = new_parent
        while ancestor is not None:
            if ancestor is child:
                raise ValueError("Moving this child would create a cycle.")
            ancestor = ancestor.parent

        # Validate sibling name uniqueness at the destination.
        if any(
            existing.name == child.name
            for existing in new_parent.children
        ):
            raise ValueError(f"Duplicate child name: {child.name}")

        # Update the hierarchy without changing the model registry.
        child_index = next(
            index
            for index, existing in enumerate(old_parent.children)
            if existing is child
        )
        del old_parent.children[child_index]
        old_parent._remove_child_couplings(child)

        new_parent.children.append(child)
        child.parent = new_parent

        # Recompute scheduling information in both affected branches.
        old_parent.update_tn(update_parent=True)
        new_parent.update_tn(update_parent=True)

    def _remove_child_couplings(self, child):
        for source, destinations in list(self.Z.items()):
            remaining = {endpoint for endpoint in destinations if endpoint[0] is not child}
            if source[0] is child or not remaining:
                del self.Z[source]
            else:
                self.Z[source] = remaining

    def add_couple(self, source, destination):
        """Connect two (model, port_name) endpoints in this coupled model."""
        for endpoint, is_source in ((source, True), (destination, False)):
            if (
                not isinstance(endpoint, tuple) or len(endpoint) != 2
                or not isinstance(endpoint[0], TES)
                or not isinstance(endpoint[1], str)
            ):
                raise TypeError("Coupling endpoints must be (model, port_name) tuples.")
            model, port = endpoint
            if model is not self and model not in self.children:
                raise ValueError("Coupling endpoints must belong to this model or a direct child.")
            ports = model.IP if is_source == (model is self) else model.OP
            if port not in ports:
                direction = "input" if ports is model.IP else "output"
                raise ValueError(f"{port!r} is not a registered {direction} port on {model.path!r}.")
        if source[0] is self and destination[0] is self:
            raise ValueError("A boundary input must couple to a child input.")
        self.Z.setdefault(source, set()).add(destination)

    def add_couples(self, *couples):
        """Add multiple (source_port, destination_port) couplings."""
        for src_port, dest_port in couples:
            self.add_couple(src_port, dest_port)

    def _after_restore_state(self):
        """Recompute derived scheduling state after restoring descendants."""
        self.update_tn(update_parent=False)

    def update_tn(self, update_parent=False):
        """Update imminent components and this model's next event time."""
        next_time = float("inf")
        imminent = set()
        for component in self.M:
            component_time = component.tNext
            if component_time < next_time:
                next_time = component_time
                imminent = {component}
            elif component_time == next_time:
                imminent.add(component)
        self.IMM = imminent
        self.tNext = next_time
        if update_parent and self.parent is not None:
            self.parent.update_tn(update_parent=True)

    def set_init_total_state(self, t):
        """Initialize this coupled model and all child components."""
        self.tNow = t
        self.tLast = t
        for component in self.M:
            component.set_init_total_state(t)
        self.update_tn()

    def Delta_x(self, x, t):
        """Route an input event to destination ports."""
        if x is None:
            self.tNow = self.tLast = t
            rescheduled = [component.Delta_x(None, t) for component in self.M]
            self.update_tn()
            return any(rescheduled)
        if not isinstance(x, PortValue):
            raise TypeError(f"{x!r} is not a PortValue.")
        self.tNow = t
        self.tLast = t
        for model, port in self.Z.get((self, x.port), ()):
            model.Delta_x(PortValue(port, x.value), t)
        self.update_tn()

    def translate_output_event(self, source_model, y, t):
        """Route a child output and return events on this model's output ports."""
        outputs = []
        if not isinstance(y, PortValue):
            return outputs
        for model, port in self.Z.get((source_model, y.port), ()):
            event = PortValue(port, y.value)
            if model is self:
                outputs.append(event)
            else:
                model.Delta_x(event, t)
        return outputs

    def get_all_influencee(self, source, XP):
        """Collect terminal atomic input endpoints influenced by a source endpoint."""
        for model, port in self.Z.get(source, ()):
            if model is self:
                continue
            if isinstance(model, CoupledDEVS):
                model.get_all_influencee((model, port), XP)
            else:
                XP.add((model, port))

    def Delta_y(self, t) -> list[PortValue]:
        """Generate output from an imminent child and update the schedule."""
        self.tNow = t
        self.tLast = t
        outputs = []
        if self.IMM:
            component = self.IMM.pop()
            result = component.Delta_y(t)
            if result is not None:
                events = result if isinstance(result, (list, tuple, set)) else [result]
                for event in events:
                    outputs.extend(self.translate_output_event(component, event, t))
            self.update_tn()
        return outputs

    def to_z_s(self):
        """Format the coupling map as a string."""
        parts = []
        for (model, port), destinations in self.Z.items():
            src_name = f"{model.path}.{port}"
            dest_names = sorted(f"{target.path}.{name}" for target, name in destinations)
            parts.append(src_name + "=>[" + ", ".join(dest_names) + "]")
        return "Z={" + ", ".join(parts) + "}"

    def __str__(self):
        return super().__str__() + "M=" + str(self.M) + "\n" + self.to_z_s()

    def state_to_s(self):
        """Format child states as a tuple-like string."""
        return "(" + ", ".join(component.qstate_to_s() for component in self.M) + ")"

    def is_network_of(self, cclass):
        """Return whether all atomic descendants are instances of cclass."""
        if isinstance(self, cclass):
            return True
        for component in self.M:
            if isinstance(component, CoupledDEVS) and not component.is_network_of(cclass):
                return False
        return True

    def format_tree(self):
        """Return the model hierarchy as a Unicode tree."""
        lines = [self.name]

        def visit(node, prefix=""):
            for index, child in enumerate(node.children):
                is_last = index == len(node.children) - 1
                branch = "└── " if is_last else "├── "
                lines.append(prefix + branch + child.name)

                if isinstance(child, CoupledDEVS):
                    extension = "    " if is_last else "│   "
                    visit(child, prefix + extension)

        visit(self)
        return "\n".join(lines)

    def next_event(self):
        """Return the earliest scheduled FD-DEVS models, or None."""
        atomic_models = []

        def collect(node):
            for child in node.children:
                if isinstance(child, CoupledDEVS):
                    collect(child)
                elif isinstance(child, DEVS):
                    atomic_models.append(child)

        collect(self)

        if not atomic_models:
            return None

        event_time = min(model.tNext for model in atomic_models)

        if event_time == float("inf"):
            return None

        models = [
            model
            for model in atomic_models
            if model.tNext == event_time
        ]

        return event_time, models
