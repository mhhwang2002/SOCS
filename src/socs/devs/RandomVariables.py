#########################################################
# Simulation and Optimization of Complex Systems (SOCS) #
# Author: moon.hwang@gmail.com                          #
#   Date: Oct.26.2026                                   #
#########################################################

"""Random-variable classes used by the SOCS simulation framework."""

import math
import random
from collections.abc import Mapping
from numbers import Real


class RandomVariable:
    """Base class for random variables over an arbitrary value set."""

    def sample(self):
        """Return one sample from this random variable."""
        raise NotImplementedError("RandomVariable.sample() must be implemented.")


class RV_Uniform(RandomVariable):
    """Uniform random variable over the interval [low, high]."""

    def __init__(self, low, high):
        if not isinstance(low, Real) or not isinstance(high, Real):
            raise TypeError("low and high must be real numbers.")
        if low > high:
            raise ValueError("low must be less than or equal to high.")
        self.low = low
        self.high = high

    def sample(self):
        return random.uniform(self.low, self.high)


class RV_Triangular(RandomVariable):
    """Triangular random variable defined by low, high, and mode."""

    def __init__(self, low, high, mode):
        if not all(isinstance(value, Real) for value in (low, high, mode)):
            raise TypeError("low, high, and mode must be real numbers.")
        if low > high:
            raise ValueError("low must be less than or equal to high.")
        if not low <= mode <= high:
            raise ValueError("mode must be between low and high.")
        self.low = low
        self.high = high
        self.mode = mode

    def sample(self):
        return random.triangular(self.low, self.high, self.mode)


class RV_Exponential(RandomVariable):
    """Exponential random variable parameterized by its positive mean."""

    def __init__(self, mean):
        if not isinstance(mean, Real):
            raise TypeError("mean must be a real number.")
        if mean <= 0:
            raise ValueError("mean must be greater than zero.")
        self.mean = mean

    def sample(self):
        return random.expovariate(1.0 / self.mean)


class DRV(RandomVariable):
    """Discrete random variable represented by value-to-probability mapping."""

    def __init__(self, table):
        if not isinstance(table, Mapping):
            raise TypeError("table must be a mapping of outcomes to probabilities.")

        self.table = dict(table)
        if not self.table:
            return

        for outcome, probability in self.table.items():
            if not isinstance(probability, Real):
                raise TypeError(f"Probability for {outcome!r} must be a real number.")
            if not math.isfinite(probability) or not 0 <= probability <= 1:
                raise ValueError(
                    f"Probability for {outcome!r} must be finite and in [0, 1]."
                )

        total = math.fsum(self.table.values())
        if not math.isclose(total, 1.0, rel_tol=1e-12, abs_tol=1e-12):
            raise ValueError(f"Probabilities must sum to 1; got {total!r}.")

    def sample(self):
        """Select and return one outcome according to its probability."""
        if not self.table:
            return None

        threshold = random.random()
        cumulative = 0.0
        last_outcome = None
        for outcome, probability in self.table.items():
            cumulative += probability
            last_outcome = outcome
            if threshold < cumulative:
                return outcome

        # Protect against floating-point rounding at the end of the sum.
        return last_outcome


if __name__ == "__main__":
    triangular = RV_Triangular(0, 10, 7)
    samples = [triangular.sample() for _ in range(10)]
    print("RV_Triangular(0, 10, 7)")
    for value in samples:
        print(value)
    print(f"Average = {sum(samples) / len(samples)}")

    uniform = RV_Uniform(0, 10)
    samples = [uniform.sample() for _ in range(10)]
    print("RV_Uniform(0, 10)")
    for value in samples:
        print(value)
    print(f"Average = {sum(samples) / len(samples)}")

    drv = DRV({"s1": 0.2, "s2": 0.6, "s3": 0.2})
    for _ in range(20):
        print("DRV sample =", drv.sample())
