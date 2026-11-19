"""Analytical stop-to-stop profiles used by the robot and its point sensor."""

from dataclasses import dataclass
from math import sqrt


@dataclass(frozen=True)
class Limits:
    acceleration: float
    deceleration: float
    yaw_rate: float
    speed: float = 2.0


# Preserve RMFS_Old.motion parameters. Braking and yaw rates are assumptions.
EMPTY = Limits(.41, .41, 45.0)
LOADED = Limits(.37, .37, 30.0)


@dataclass(frozen=True)
class StraightProfile:
    length: float
    acceleration: float = .37
    deceleration: float = .37
    speed: float = 2.0

    @property
    def peak(self):
        return min(self.speed, sqrt(
            2*self.length/(1/self.acceleration + 1/self.deceleration)
        ))

    @property
    def duration(self):
        return self.length/self.peak + self.peak/2*(
            1/self.acceleration + 1/self.deceleration
        )

    def time_at_distance(self, distance):
        if not 0 <= distance <= self.length:
            raise ValueError("Distance is outside the motion profile.")
        peak = self.peak
        if distance <= peak**2/(2*self.acceleration):
            return sqrt(2*distance/self.acceleration)
        remaining = self.length-distance
        if remaining <= peak**2/(2*self.deceleration):
            return self.duration-sqrt(2*remaining/self.deceleration)
        return distance/peak + peak/(2*self.acceleration)

    def distance_at(self, elapsed):
        elapsed = max(0, min(elapsed, self.duration))
        if elapsed <= self.peak/self.acceleration:
            return .5*self.acceleration*elapsed**2
        remaining = self.duration-elapsed
        if remaining <= self.peak/self.deceleration:
            return self.length-.5*self.deceleration*remaining**2
        return self.peak*(elapsed-self.peak/(2*self.acceleration))

