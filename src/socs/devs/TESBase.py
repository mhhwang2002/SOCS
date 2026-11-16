#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################
"""
Base classes for DEVS models.

Originally written in 2011 and modernized for Python 3.
"""

from copy import deepcopy
from uuid import UUID

from .AlarmClock import AlarmClock
from .Snapshot import ModelSnapshot
from .Base import classname, function_name


class NamedObj:
    """Base class for objects with a name."""

    def __init__(self, name):
        if not isinstance(name, str) or not name.strip():
            raise ValueError("Model name must be a non-empty string.")
        self.name = name

    def __str__(self):
        return self.name_to_s()

    def name_to_s(self):
        """Convert the object's name to a string."""
        return str(self.name)


def hiername(frm, upto=None):
    """Return a hierarchical name using '.' as the delimiter."""
    if frm is None:
        return ""

    if frm is upto and upto is not None:
        return frm.name_to_s()

    parent = getattr(frm, "parent", None)
    if parent is not None:
        ancestor_name = hiername(parent, upto)
        if ancestor_name:
            return f"{ancestor_name}.{frm.name_to_s()}"

    return frm.name_to_s()


class HierNamedObj(NamedObj):
    """A named object that may belong to a model hierarchy."""

    def __init__(self, name, parent=None):
        super().__init__(name)
        self.parent = parent
        self.children = []

    @property
    def path(self):
        """Return the fully qualified hierarchical name."""
        return hiername(self)

    def parent_hier_name_to_s(self):
        """Return this object's name relative to its parent."""
        return hiername(self, self.parent)

    def add_child(self, child):
        """Add a child while preserving model-tree consistency."""
        if not isinstance(child, HierNamedObj):
            raise TypeError("Child must be a HierNamedObj instance.")

        if child is self:
            raise ValueError("A model cannot be its own child.")

        ancestor = self
        while ancestor is not None:
            if ancestor is child:
                raise ValueError("Adding this child would create a cycle.")
            ancestor = getattr(ancestor, "parent", None)

        if child.parent is not None and child.parent is not self:
            raise ValueError("Child already belongs to another parent.")

        if any(
            existing is not child and existing.name == child.name
            for existing in self.children
        ):
            raise ValueError(f"Duplicate child name: {child.name}")

        if child not in self.children:
            self.children.append(child)

        child.parent = self


    def remove_child(self, child):
        """Remove a child while preserving model-tree consistency."""
        if not isinstance(child, HierNamedObj):
            raise TypeError("Child must be a HierNamedObj instance.")

        if child.parent is not self:
            raise ValueError("Child does not belong to this parent.")

        child_index = next(
            (
                index
                for index, existing in enumerate(self.children)
                if existing is child
            ),
            None,
        )

        if child_index is None:
            raise ValueError("Child is not present in the children list.")

        del self.children[child_index]
        child.parent = None


class TES(HierNamedObj, AlarmClock):
    """Timed event system with ports, a clock, and snapshot support."""
 
    def __init__(self, name, parent=None, tl=0, tn=float("inf")):
        super().__init__(name, parent)
        AlarmClock.__init__(self, tl, tn)
        self.IP = {}
        self.OP = {}
        self._model_id = None

    def set_init_total_state(self, t):
        raise NotImplementedError(
            f"{classname(self)}::{function_name()} must be overridden."
        )

    def Delta_x(self, x, t):
        raise NotImplementedError(
            f"{classname(self)}::{function_name()} must be overridden."
        )

    def Delta_y(self, t) -> "PortValue | list[PortValue] | tuple[PortValue, ...] | set[PortValue] | None":
        raise NotImplementedError(
            f"{classname(self)}::{function_name()} must be overridden."
        )

    def add_input_ports(self, *args):
        """Add input ports by name."""
        for ipname in args:
            if not isinstance(ipname, str):
                raise TypeError("Input port names must be strings.")
            if ipname in self.IP:
                print(f"{ipname} already exists in IP")
            else:
                self.IP[ipname] = ipname

    def to_ip_s(self):
        """Return a string representation of input port names."""
        return "IP=" + str(list(self.IP.keys()))

    def add_output_ports(self, *args):
        """Add output ports by name."""
        for opname in args:
            if not isinstance(opname, str):
                raise TypeError("Output port names must be strings.")
            if opname in self.OP:
                print(f"{opname} already exists in OP")
            else:
                self.OP[opname] = opname

    def to_op_s(self):
        """Return a string representation of output port names."""
        return "OP=" + str(list(self.OP.keys()))


    @property
    def model_id(self) -> UUID | None:
        return self._model_id

    def __str__(self):
        """Return a string representation of the model and its interfaces."""
        result = super().__str__() + "\n"
        result += self.to_ip_s() + "\n"
        result += self.to_op_s() + "\n"
        result += "clock=" + self.format_clock() + "\n"
        return result

    def state_to_s(self):
        """Return a string representation of the model state."""
        return ""

    def qstate_to_s(self):
        """Return a string representation of the total state."""
        state = self.state_to_s()
        if state:
            return f"({self.name}:{state},{self.format_clock()})"
        return f"({self.name}:{self.format_clock()})"


    def _snapshot_hooks(self):
        getter = getattr(self, "get_snapshot", None)
        setter = getattr(self, "set_snapshot", None)
        if getter is None and setter is None:
            getter = getattr(self, "get_state", None)
            setter = getattr(self, "set_state", None)
        if callable(getter) != callable(setter):
            raise TypeError(f"{self.path}: snapshot getter and setter must be implemented together.")
        return getter, setter

    def capture_snapshot(self):
        """Capture this model's dynamic state and its descendants."""
        get_state, _ = self._snapshot_hooks()

        has_model_state = callable(get_state)
        model_state = deepcopy(get_state()) if has_model_state else None

        children = {}

        for child in self.children:
            if not isinstance(child, TES):
                raise TypeError(
                    f"{self.path}: child {child.name!r} is not TES."
                )

            if child.name in children:
                raise ValueError(
                    f"{self.path}: duplicate child name {child.name!r}."
                )

            children[child.name] = child.capture_snapshot()

        return ModelSnapshot(
            model_id=id(self),
            model_path=self.path,
            common_state={
                "time_now": self.tNow,
                "time_last": self.tLast,
                "time_next": self.tNext,
            },
            model_state=model_state,
            has_model_state=has_model_state,
            children=children,
        )

    def restore_snapshot(self, snapshot):
        """Restore this model and its descendants from a ModelSnapshot."""
        self._validate_snapshot(snapshot)

        common_state = snapshot.common_state
        self.tNow = common_state.get("time_now", common_state["time_last"])

        if snapshot.has_model_state:
            _, set_snapshot = self._snapshot_hooks()
            set_snapshot(deepcopy(snapshot.model_state))

        self.tLast = common_state["time_last"]
        self.tNext = common_state["time_next"]


        for child in self.children:
            child.restore_snapshot(snapshot.children[child.name])

        self._after_restore_state()

    def _validate_snapshot(self, snapshot):
        """Validate snapshot identity and hierarchy before restoration."""
        if not isinstance(snapshot, ModelSnapshot):
            raise TypeError("snapshot must be a ModelSnapshot instance.")

        if snapshot.model_id != id(self):
            raise ValueError(
                f"Snapshot model mismatch: expected {self.path!r}, "
                f"got {snapshot.model_path!r}."
            )

        current_child_names = [child.name for child in self.children]

        if len(current_child_names) != len(set(current_child_names)):
            raise ValueError(
                f"{self.path}: duplicate child names in model hierarchy."
            )

        if set(current_child_names) != set(snapshot.children):
            raise ValueError(
                f"{self.path}: snapshot hierarchy does not match "
                "the current model hierarchy."
            )

        get_snapshot, _ = self._snapshot_hooks()
        supports_model_state = callable(get_snapshot)

        if supports_model_state != snapshot.has_model_state:
            raise ValueError(
                f"{self.path}: snapshot model-state contract "
                "does not match the model."
            )

        if not isinstance(snapshot.common_state, dict):
            raise TypeError(
                f"{self.path}: snapshot common_state must be a dictionary."
            )

        required_keys = {
            "time_last",
            "time_next",
        }

        if not required_keys.issubset(snapshot.common_state):
            raise ValueError(
                f"{self.path}: snapshot common_state is incomplete."
            )

        for child in self.children:
            child._validate_snapshot(snapshot.children[child.name])

    def _after_restore_state(self):
        """Rebuild derived runtime state after restoration."""


class PortValue:
    """A port-value pair."""

    def __init__(self, port: str | None = None, value=None):
        if port is not None and not isinstance(port, str):
            raise TypeError("port must be a string or None.")
        self.port = port
        self.value = value

    def __eq__(self, other):
        return (
            self.__class__ is other.__class__
            and self.port == other.port
            and self.value == other.value
        )

    def __str__(self):
        if self.port is None:
            return str(self.value)
        result = str(self.port)
        if self.value is not None:
            result += ":" + str(self.value)
        return result

    def __hash__(self):
        return hash(str(self))


if __name__ == "__main__":
    print("TESBase Test")

    seong = HierNamedObj("Seong")
    moon = HierNamedObj("Moon", seong)
    sun = HierNamedObj("Sun", moon)
    su = HierNamedObj("Su", sun)

    print(hiername(su))
    print(hiername(su, sun))
    print(hiername(su, su.parent))
    print(hiername(su, moon))

    model = TES("Model", tl=10)
    print(f"tnow={model.tNow:.3f}")
