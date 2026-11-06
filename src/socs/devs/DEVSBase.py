#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################
"""
Base classes for DEVS models.

Originally written in 2011 and modernized for Python 3.
"""

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


class TES(HierNamedObj):
    """Base class for timed event systems."""

    def __init__(self, name, parent=None):
        super().__init__(name, parent)

    def Delta_0(self, t):
        raise NotImplementedError(
            f"{classname(self)}::{function_name()} must be overridden."
        )

    def Delta_x(self, x, t):
        raise NotImplementedError(
            f"{classname(self)}::{function_name()} must be overridden."
        )

    def Delta_y(self, t):
        raise NotImplementedError(
            f"{classname(self)}::{function_name()} must be overridden."
        )

    def Delta_cont(self, t):
        raise NotImplementedError(
            f"{classname(self)}::{function_name()} must be overridden."
        )


if __name__ == "__main__":
    print("DEVSBase Test")

    seong = HierNamedObj("Seong")
    moon = HierNamedObj("Moon", seong)
    sun = HierNamedObj("Sun", moon)
    su = HierNamedObj("Su", sun)

    print(hiername(su))
    print(hiername(su, sun))
    print(hiername(su, su.parent))
    print(hiername(su, moon))
