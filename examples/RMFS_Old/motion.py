"""MiR600 demo trajectories with stop-to-stop motion in SI units."""

from dataclasses import dataclass
from itertools import pairwise
from math import atan2, degrees, dist, sqrt

from .traffic_layout import STANDBY as HOME
from .traffic_layout import shortest_route

def final_heading(route, fallback=-90):
    if len(route) == 1:
        return fallback
    start, end = route[-2:]
    return degrees(atan2(end[1] - start[1], end[0] - start[0]))


@dataclass(frozen=True)
class Limits:
    acceleration: float
    deceleration: float
    yaw_rate: float
    speed: float = 2.0


# MiR specifies acceleration and maximum speed. Service braking and yaw rates
# are simulation assumptions until measured with the actual rack and lift.
EMPTY = Limits(.41, .41, 45.0)
LOADED = Limits(.37, .37, 30.0)


@dataclass(frozen=True)
class Straight:
    length: float
    limits: Limits

    @property
    def peak(self):
        a, b = self.limits.acceleration, self.limits.deceleration
        return min(self.limits.speed, sqrt(2 * self.length / (1 / a + 1 / b)))

    @property
    def duration(self):
        v = self.peak
        ramp_time = v / self.limits.acceleration + v / self.limits.deceleration
        return self.length / v + ramp_time / 2

    def distance_at(self, time):
        v = self.peak
        a, b = self.limits.acceleration, self.limits.deceleration
        time = max(0, min(time, self.duration))
        if time < v / a:
            return .5 * a * time ** 2
        remaining = self.duration - time
        if remaining < v / b:
            return self.length - .5 * b * remaining ** 2
        return v * (time - .5 * v / a)

    def time_at_distance(self, distance):
        """Invert the stop-to-stop profile at a spatial boundary."""
        v = self.peak
        a, b = self.limits.acceleration, self.limits.deceleration
        distance = min(self.length, max(0.0, distance))
        if distance <= v * v / (2 * a):
            return sqrt(2 * distance / a)
        remaining = self.length - distance
        if remaining <= v * v / (2 * b):
            return self.duration - sqrt(2 * remaining / b)
        return distance / v + v / (2 * a)


@dataclass(frozen=True)
class Action:
    start: tuple
    end: tuple
    heading: float
    turn: float
    duration: float
    straight: Straight | None = None

    def pose(self, time):
        if self.straight is None:
            return self.start, self.heading + self.turn * time / self.duration
        fraction = self.straight.distance_at(time) / self.straight.length
        return tuple(a + (b - a) * fraction for a, b in zip(self.start, self.end)), self.heading


def heading_between(start, end):
    return degrees(atan2(end[1] - start[1], end[0] - start[0]))


def build_actions(route, heading, limits):
    # Combine collinear arcs so intermediate junctions do not force a stop.
    if len(route) == 1:
        return
    corners = [route[0]]
    for before, point, after in zip(route, route[1:], route[2:]):
        if heading_between(before, point) != heading_between(point, after):
            corners.append(point)
    corners.append(route[-1])
    for start, end in pairwise(corners):
        target = heading_between(start, end)
        turn = (target - heading + 180) % 360 - 180
        if turn:
            yield Action(start, start, heading, turn, abs(turn) / limits.yaw_rate)
        straight = Straight(dist(start, end), limits)
        yield Action(start, end, target, 0, straight.duration, straight)
        heading = target


class MissionMotion:
    def __init__(self, slot, station, approach_route, *, home=None, return_home=True, heading=-90):
        self.routes = {
            "TravelToRack": approach_route,
            "TravelToWorkstation": shortest_route(slot, station),
            "ReturnRack": shortest_route(station, slot),
            "ReturnToStandby": shortest_route(slot, home or approach_route[0]) if return_home else (slot,),
        }
        self._build_profiles(heading)

    @classmethod
    def for_loaded_leg(cls, start, end):
        """Build the fixed routes for a single loaded travel experiment."""
        motion = cls.__new__(cls)
        motion.routes = {
            "TravelToRack": (start,),
            "TravelToWorkstation": (start, end),
            "ReturnRack": (end,),
            "ReturnToStandby": (end,),
        }
        motion._build_profiles(heading_between(start, end))
        return motion

    def _build_profiles(self, heading):
        self.start_headings = {}
        self.actions = {}
        for phase, route in self.routes.items():
            limits = LOADED if phase in {"TravelToWorkstation", "ReturnRack"} else EMPTY
            self.start_headings[phase] = heading
            self.actions[phase] = tuple(build_actions(route, heading, limits))
            heading = final_heading(route, heading)
        self.end_heading = heading
        self.travel_times = {
            phase: sum(action.duration for action in actions)
            for phase, actions in self.actions.items()
        }

    def pose(self, phase, elapsed):
        actions = self.actions[phase]
        if not actions:
            return self.routes[phase][0], self.start_headings[phase]
        for action in actions:
            if elapsed <= action.duration:
                return action.pose(max(0, elapsed))
            elapsed -= action.duration
        last = actions[-1]
        return last.pose(last.duration)



