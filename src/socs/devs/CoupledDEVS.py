#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################

"""Coupled DEVS model for composing atomic and coupled components."""

from .Simulatable import Simulatable
from .DEVSInterfaces import Port, InputPort, OutputPort, PortValue
from .FDDEVS import FDDEVS
from .ModelRegistry import ModelRegistry

class CoupledDEVS(Simulatable):
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
        if not isinstance(child, (CoupledDEVS, FDDEVS)):
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

        self.update_tN(update_parent=True)

    def addComponents(self, *args):
        """Add child components using the legacy method name."""
        for component in args:
            self.add_child(component)

    def remove_child(self, child):
        """Remove a child model and unregister its subtree."""
        if not isinstance(child, (CoupledDEVS, FDDEVS)):
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

        self.update_tN(update_parent=True)

    def move_child_to(self, child, new_parent):
        """Move a child model to another parent within the same system."""
        if not isinstance(child, (CoupledDEVS, FDDEVS)):
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

        new_parent.children.append(child)
        child.parent = new_parent

        # Recompute scheduling information in both affected branches.
        old_parent.update_tN(update_parent=True)
        new_parent.update_tN(update_parent=True)

    def addCouple(self, src_port, dest_port):
        """Connect a source port to a destination port."""
        if not isinstance(src_port, Port) or not isinstance(dest_port, Port):
            raise TypeError(f"{src_port!r} and {dest_port!r} must be Port objects.")
        self.Z.setdefault(src_port, set()).add(dest_port)

    def addCouples(self, *couples):
        """Add multiple (source_port, destination_port) couplings."""
        for src_port, dest_port in couples:
            self.addCouple(src_port, dest_port)

    def _after_restore_state(self):
        """Recompute derived scheduling state after restoring descendants."""
        self.update_tN(update_parent=False)

    def update_tN(self, update_parent=False):
        """Update imminent components and this model's next event time."""
        next_time = float("inf")
        imminent = set()
        for component in self.M:
            component_time = component.timeNext()
            if component_time < next_time:
                next_time = component_time
                imminent = {component}
            elif component_time == next_time:
                imminent.add(component)
        self.IMM = imminent
        self.set_timeNext(next_time)
        if update_parent and self.parent is not None:
            self.parent.update_tN(update_parent=True)

    def Delta_0(self, t):
        """Initialize this coupled model and all child components."""
        self.set_timeLast(t)
        for component in self.M:
            component.Delta_0(t)
        self.update_tN()

    def Delta_x(self, x, t):
        """Route an input event to destination ports."""
        self.set_timeLast(t)
        if not isinstance(x, PortValue):
            raise TypeError(f"{x!r} is not a PortValue.")
        for dest_port in self.Z.get(x.port, ()):
            if dest_port.parent is not None:
                dest_port.parent.Delta_x(PortValue(dest_port, x.value), t)
        self.update_tN()

    def translateOutputEvent(self, y, t):
        """Translate a child output through this model's coupling graph."""
        outputs = []
        if not y or not isinstance(y, PortValue):
            return outputs
        if y.port not in self.Z:
            return [y]

        for dest_port in self.Z[y.port]:
            if isinstance(dest_port, InputPort):
                if dest_port.parent is not None:
                    dest_port.parent.Delta_x(PortValue(dest_port, y.value), t)
            elif isinstance(dest_port, OutputPort):
                if isinstance(dest_port.parent, CoupledDEVS):
                    outputs.extend(
                        dest_port.parent.translateOutputEvent(
                            PortValue(dest_port, y.value), t
                        )
                    )
        return outputs

    def getAllInfluencee(self, z, XP):
        """Collect terminal atomic input ports influenced by a port or event."""
        if isinstance(z, PortValue):
            source_port = z.port
        elif isinstance(z, Port):
            source_port = z
        else:
            return

        for dest_port in self.Z.get(source_port, ()):
            if isinstance(dest_port.parent, CoupledDEVS):
                dest_port.parent.getAllInfluencee(dest_port, XP)
            elif isinstance(dest_port.parent, Simulatable):
                XP.add(dest_port)

    def Delta_y(self, t):
        """Generate output from an imminent child and update the schedule."""
        self.set_timeLast(t)
        outputs = []
        if self.IMM:
            component = self.IMM.pop()
            result = component.Delta_y(t)
            if result is not None:
                events = result if isinstance(result, (list, tuple, set)) else [result]
                for event in events:
                    for translated in self.translateOutputEvent(event, t):
                        if translated and translated.port.parent == self:
                            outputs.append(translated)
            self.update_tN()
        return outputs

    def Delta_null(self, t):
        """Update time-dependent state/statistics in all children."""
        for component in self.M:
            component.Delta_null(t)

    def get_statistics_str(self):
        """Return statistics for all descendants."""
        result = self.name + "\n"
        for component in self.M:
            if isinstance(component, CoupledDEVS):
                result += component.get_statistics_str() + "\n"
            else:
                result += "\t" + component.name + ": " + component.get_statistics_str() + "\n"
        return result

    def to_Z_s(self):
        """Format the coupling map as a string."""
        parts = []
        for src_port, dest_ports in self.Z.items():
            src_name = (
                src_port.parent_hier_name_to_s()
                if hasattr(src_port, "parent_hier_name_to_s")
                else str(src_port)
            )
            dest_names = [
                dest.parent_hier_name_to_s()
                if hasattr(dest, "parent_hier_name_to_s")
                else str(dest)
                for dest in dest_ports
            ]
            parts.append(src_name + "=>[" + ", ".join(dest_names) + "]")
        return "Z={" + ", ".join(parts) + "}"

    def __str__(self):
        return super().__str__() + "M=" + str(self.M) + "\n" + self.to_Z_s()

    def state_to_s(self):
        """Format child states as a tuple-like string."""
        return "(" + ", ".join(component.qstate_to_s() for component in self.M) + ")"

    def isNetworkOf(self, cclass):
        """Return whether all atomic descendants are instances of cclass."""
        if isinstance(self, cclass):
            return True
        for component in self.M:
            if isinstance(component, CoupledDEVS) and not component.isNetworkOf(cclass):
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
                elif isinstance(child, FDDEVS):
                    atomic_models.append(child)

        collect(self)

        if not atomic_models:
            return None

        event_time = min(model.timeNext() for model in atomic_models)

        if event_time == float("inf"):
            return None

        models = [
            model
            for model in atomic_models
            if model.timeNext() == event_time
        ]

        return event_time, models
