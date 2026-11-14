#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################
"""Interval and non-negative time-interval classes."""

import random
import sys

from .Base import isnumeric
from .RandomVariables import RandomVariable


class Interval(RandomVariable):
    """Closed interval with lower bound l and upper bound u."""

    def __init__(self, l=float("-inf"), u=float("inf")):
        if not isnumeric(l) or not isnumeric(u):
            raise TypeError(
                f"Interval bounds must be integers or floats; got "
                f"{type(l).__name__} and {type(u).__name__}."
            )
        self.l = l
        self.u = u

    def __hash__(self):
        return hash((self.l, self.u))

    def make_invalid(self):
        """Make this interval invalid by setting l > u."""
        self.l, self.u = float("inf"), 0
        return self

    def valid(self):
        """Return whether l <= u."""
        return self.l <= self.u

    def invalid(self):
        """Return whether the interval is invalid."""
        return not self.valid()

    def make_universal(self):
        """Set this interval to (-infinity, infinity)."""
        self.l, self.u = float("-inf"), float("inf")
        return self

    def universal(self):
        """Return whether this interval is (-infinity, infinity)."""
        return self.l == float("-inf") and self.u == float("inf")

    def make_singleton(self, v):
        """Set this interval to the singleton [v, v]."""
        if not isnumeric(v):
            raise TypeError("The singleton value must be an integer or float.")
        self.l = self.u = v
        return self

    def singleton(self):
        """Return whether the interval contains exactly one value."""
        return self.l == self.u

    def include(self, inv):
        """Return whether this interval contains another interval."""
        return (
            isinstance(inv, Interval)
            and self.valid()
            and inv.valid()
            and self.l <= inv.l
            and inv.u <= self.u
        )

    def intersect(self, inv):
        """Return the intersection, or an invalid interval if disjoint."""
        if not isinstance(inv, Interval) or self.invalid() or inv.invalid():
            return Interval().make_invalid()
        lower = max(self.l, inv.l)
        upper = min(self.u, inv.u)
        if lower > upper:
            return Interval().make_invalid()
        return Interval(lower, upper)

    def compare(self, other):
        """Compare intervals by their lower bounds, then upper bounds."""
        if not isinstance(other, Interval):
            raise TypeError(f"{other!r} is not an Interval.")
        if self.l < other.l:
            return -1
        if self.l > other.l:
            return 1
        if self.u < other.u:
            return -1
        if self.u > other.u:
            return 1
        return 0

    def __eq__(self, other):
        if not isinstance(other, Interval):
            return NotImplemented
        return self.l == other.l and self.u == other.u

    def __lt__(self, other):
        if not isinstance(other, Interval):
            return NotImplemented
        return self.compare(other) < 0

    def __le__(self, other):
        if not isinstance(other, Interval):
            return NotImplemented
        return self.compare(other) <= 0

    def __gt__(self, other):
        if not isinstance(other, Interval):
            return NotImplemented
        return self.compare(other) > 0

    def __ge__(self, other):
        if not isinstance(other, Interval):
            return NotImplemented
        return self.compare(other) >= 0

    def infinity(self):
        """Return whether this is the singleton [+infinity, +infinity]."""
        return self.valid() and self.l == float("inf") and self.u == float("inf")

    def __neg__(self):
        return Interval(-self.u, -self.l)

    def __sub__(self, other):
        if not isinstance(other, Interval):
            return NotImplemented
        return Interval(self.l - other.u, self.u - other.l)

    def __add__(self, other):
        if not isinstance(other, Interval):
            return NotImplemented
        return Interval(self.l + other.l, self.u + other.u)

    def merge_over(self, other):
        """Expand this interval to cover another interval of the same type."""
        if type(other) is not type(self):
            raise TypeError(f"{type(other).__name__} is not {type(self).__name__}.")
        if not other.valid():
            return self
        if not self.valid():
            self.l, self.u = other.l, other.u
        else:
            self.l = min(self.l, other.l)
            self.u = max(self.u, other.u)
        return self

    def extend_by(self, t):
        """Expand this interval to include the numeric value t."""
        if not isnumeric(t):
            raise TypeError("The value to include must be an integer or float.")
        if not self.valid():
            self.l = self.u = t
        else:
            self.l = min(self.l, t)
            self.u = max(self.u, t)
        return self

    def __str__(self):
        if not self.valid():
            return f"({self.l},{self.u}) is an invalid {type(self).__name__}"
        left = "(" if self.l == float("-inf") else "["
        right = ")" if self.u == float("inf") else "]"
        return f"{left}{self.l},{self.u}{right}"

    def singular(self):
        """Legacy alias for singleton()."""
        return self.singleton()

    def sample(self):
        """Sample uniformly from the interval, approximating infinite bounds."""
        if not self.valid():
            raise ValueError("Cannot sample from an invalid interval.")
        if self.singleton():
            return self.l

        lower, upper = self.l, self.u
        if lower == float("-inf"):
            lower = -sys.float_info.max / 10000
        if upper == float("inf"):
            upper = sys.float_info.max / 10000
        return random.uniform(lower, upper)


class TInterval(Interval):
    """Closed time interval contained in [0, infinity)."""

    def __init__(self, l=0, u=float("inf")):
        super().__init__(l, u)

    def valid(self):
        return super().valid() and 0 <= self.l

    def make_universal(self):
        """Set this time interval to [0, infinity)."""
        self.l, self.u = 0, float("inf")
        return self

    def universal(self):
        return self.l == 0 and self.u == float("inf")

    def intersect(self, inv):
        result = super().intersect(inv)
        if not result.valid():
            return TInterval().make_invalid()
        return TInterval(result.l, result.u)

    def __neg__(self):
        return Interval(-self.u, -self.l)


if __name__ == "__main__":
    interval = Interval()
    print(f"interval = {interval}")
    interval = Interval(-10, 10)
    print(f"interval = {interval}, valid = {interval.valid()}")
    print(f"contains [-3, 3] = {interval.include(Interval(-3, 3))}")
    print(f"intersection = {interval.intersect(Interval(3, 20))}")
    print(f"time interval = {TInterval(1, 5)}")
