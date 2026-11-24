"""Analytical stop-to-stop profiles used by the robot and its point sensor."""

from dataclasses import dataclass
from functools import cached_property
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

    @property
    def brake_start(self):
        return self.duration-self.peak/self.deceleration

    def velocity_at(self, elapsed):
        elapsed = max(0., min(elapsed, self.duration))
        return min(self.acceleration*elapsed, self.peak,
                   self.deceleration*(self.duration-elapsed))


@dataclass(frozen=True)
class MovingProfile:
    """A straight motion starting at a given speed and ending at rest."""
    length: float
    initial_speed: float
    acceleration: float
    deceleration: float
    speed: float

    def __post_init__(self):
        if self.length <= 0 or not 0 <= self.initial_speed <= self.speed:
            raise ValueError('Invalid moving profile length or initial speed.')
        if self.length < self.initial_speed**2/(2*self.deceleration)-1e-9:
            raise ValueError('The new endpoint is inside the braking distance.')

    @property
    def peak(self):
        return min(self.speed, sqrt((2*self.length+self.initial_speed**2/self.acceleration)
                                    /(1/self.acceleration+1/self.deceleration)))

    @property
    def acceleration_time(self):
        return max(0., (self.peak-self.initial_speed)/self.acceleration)

    @property
    def acceleration_distance(self):
        return (self.peak**2-self.initial_speed**2)/(2*self.acceleration)

    @property
    def cruise_time(self):
        return max(0., (self.length-self.acceleration_distance
                        -self.peak**2/(2*self.deceleration))/self.peak)

    @property
    def brake_start(self):
        return self.acceleration_time+self.cruise_time

    @property
    def duration(self):
        return self.brake_start+self.peak/self.deceleration

    def distance_at(self, elapsed):
        t = max(0., min(elapsed, self.duration))
        if t <= self.acceleration_time:
            return self.initial_speed*t+.5*self.acceleration*t*t
        if t <= self.brake_start:
            return self.acceleration_distance+self.peak*(t-self.acceleration_time)
        return self.length-.5*self.deceleration*(self.duration-t)**2

    def velocity_at(self, elapsed):
        t = max(0., min(elapsed, self.duration))
        return min(self.initial_speed+self.acceleration*t, self.peak,
                   self.deceleration*(self.duration-t))

    def time_at_distance(self, distance):
        if not -1e-9 <= distance <= self.length+1e-9:
            raise ValueError('Distance is outside the motion profile.')
        d = max(0., min(distance, self.length))
        if d <= self.acceleration_distance:
            return (sqrt(self.initial_speed**2+2*self.acceleration*d)
                    -self.initial_speed)/self.acceleration
        if self.length-d <= self.peak**2/(2*self.deceleration):
            return self.duration-sqrt(2*(self.length-d)/self.deceleration)
        return self.acceleration_time+(d-self.acceleration_distance)/self.peak


@dataclass(frozen=True)
class ExtendedProfile:
    """Preserve the executed prefix exactly; splice a velocity-continuous tail."""
    original: object
    switch_time: float
    length: float
    acceleration: float
    deceleration: float
    speed: float

    @cached_property
    def offset(self):
        return self.original.distance_at(self.switch_time)

    @cached_property
    def tail(self):
        return MovingProfile(self.length-self.offset,
                             self.original.velocity_at(self.switch_time),
                             self.acceleration, self.deceleration, self.speed)

    @cached_property
    def duration(self):
        return self.switch_time+self.tail.duration

    @cached_property
    def brake_start(self):
        return self.switch_time+self.tail.brake_start

    def distance_at(self, elapsed):
        return (self.original.distance_at(elapsed) if elapsed <= self.switch_time else
                self.offset+self.tail.distance_at(elapsed-self.switch_time))

    def velocity_at(self, elapsed):
        if elapsed >= self.duration:
            return 0.
        return (self.original.velocity_at(elapsed) if elapsed <= self.switch_time else
                self.tail.velocity_at(elapsed-self.switch_time))

    def time_at_distance(self, distance):
        return (self.original.time_at_distance(distance) if distance <= self.offset else
                self.switch_time+self.tail.time_at_distance(distance-self.offset))
