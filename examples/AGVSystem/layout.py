"""Shared geometry and vehicle dimensions. Distances are in metres."""
import json
from dataclasses import dataclass
from pathlib import Path
from math import atan2, cos, dist, isfinite, pi, sin


@dataclass(frozen=True)
class Segment:
    source: str
    target: str
    center: tuple | None = None

    @property
    def length(self):
        if self.center is None:
            return dist(POINTS[self.source], POINTS[self.target])
        return RADIUS * pi / 2

    def pose(self, fraction):
        x, y = POINTS[self.source]
        if self.center is None:
            end_x, end_y = POINTS[self.target]
            dx, dy = end_x - x, end_y - y
            return x + fraction * dx, y + fraction * dy, atan2(dy, dx)
        cx, cy = self.center
        angle = atan2(y - cy, x - cx) + fraction * pi / 2
        return cx + RADIUS * cos(angle), cy + RADIUS * sin(angle), angle + pi / 2


CONFIG = json.loads(Path(__file__).with_name("dimensions.json").read_text(encoding="utf-8-sig"))
ROUTE = CONFIG["route"]
CHARGER = CONFIG["charger"]
ROUTE_WIDTH = ROUTE["width_m"]
ROUTE_DEPTH = ROUTE["depth_m"]
RADIUS = ROUTE["corner_radius_m"]
LANE_WIDTH = ROUTE["lane_width_m"]
MARGIN = ROUTE["floor_margin_m"]
if not all(isfinite(value) and value > 0
           for value in (ROUTE_WIDTH, ROUTE_DEPTH, RADIUS, LANE_WIDTH, MARGIN)):
    raise ValueError("Route dimensions must be positive and finite")
if not LANE_WIDTH / 2 < RADIUS < min(ROUTE_WIDTH, ROUTE_DEPTH) / 2:
    raise ValueError("Corner radius must fit the lane and leave straight segments")

FLOOR_WIDTH = ROUTE_WIDTH + LANE_WIDTH + 2 * MARGIN
FLOOR_DEPTH = ROUTE_DEPTH + LANE_WIDTH + 2 * MARGIN
x, y = ROUTE_WIDTH / 2, ROUTE_DEPTH / 2
straight_x, straight_y = x - RADIUS, y - RADIUS
first, second = ROUTE["long_side_stop_fractions"]
if not 0 < first < second < 1:
    raise ValueError("Stop fractions must satisfy 0 < first < second < 1")
stop_a, stop_b = (straight_x * (2 * fraction - 1) for fraction in (first, second))
POINTS = {
    "P01": (-straight_x, -y), "CH01": (stop_a, -y), "P02": (stop_b, -y),
    "P03": (straight_x, -y), "P04": (x, -straight_y), "P05": (x, straight_y),
    "P06": (straight_x, y), "CH02": (-stop_a, y), "P07": (-stop_b, y),
    "P08": (-straight_x, y), "P09": (-x, straight_y), "P10": (-x, -straight_y),
}
CENTERS = {
    "P03": (straight_x, -straight_y), "P05": (straight_x, straight_y),
    "P08": (-straight_x, straight_y), "P10": (-straight_x, -straight_y),
}
SEGMENTS = {
    source: Segment(source, target, CENTERS.get(source))
    for source, target in zip(POINTS, [*list(POINTS)[1:], next(iter(POINTS))])
}
DWELL = CONFIG["charging_dwell_s"]
if DWELL.keys() != {"CH01", "CH02"} or any(not isfinite(t) or t < 0 for t in DWELL.values()):
    raise ValueError("CH01 and CH02 require finite, nonnegative dwell times")


@dataclass(frozen=True)
class Vehicle:
    name: str
    start: str
    length: float
    width: float
    speed: float
    height: float


VEHICLES = tuple(
    Vehicle(name, data["start"], data["length_m"], data["width_m"],
            data["speed_mps"], data["height_m"])
    for name, data in CONFIG["vehicles"].items()
)
shortest_segment = min(segment.length for segment in SEGMENTS.values())
if len({vehicle.start for vehicle in VEHICLES}) != len(VEHICLES):
    raise ValueError("Initial vehicle points must be distinct")
for vehicle in VEHICLES:
    if vehicle.start not in POINTS:
        raise ValueError(f"{vehicle.name}: unknown start point {vehicle.start}")
    if not all(isfinite(value) and value > 0
               for value in (vehicle.length, vehicle.width, vehicle.height, vehicle.speed)):
        raise ValueError(f"{vehicle.name}: dimensions and speed must be positive and finite")
    if vehicle.length >= shortest_segment or vehicle.width > LANE_WIDTH:
        raise ValueError(f"{vehicle.name}: vehicle must fit the lane and shortest segment")
VEHICLE_BY_NAME = {vehicle.name: vehicle for vehicle in VEHICLES}


def pose_behind(source, distance):
    """Locate a body part using its distance from the front along the loop."""
    keys = list(SEGMENTS)
    while distance < 0:
        source = keys[keys.index(source) - 1]
        distance += SEGMENTS[source].length
    segment = SEGMENTS[source]
    return segment.pose(distance / segment.length)

# Reference mesh dimensions and body offsets used to scale the editable assets.
REFERENCE_SIZE = {
    "Tugger": (4.4, 0.95, 0.83),
    "Unit load": (1.6, 1.0, 0.95),
    "Forklift": (2.65, 1.15, 1.885),
}
if VEHICLE_BY_NAME.keys() != REFERENCE_SIZE.keys():
    raise ValueError("Vehicle profiles must match the three supplied vehicle assets")


def vehicle_scale(vehicle):
    return tuple(
        size / reference
        for size, reference in zip(
            (vehicle.length, vehicle.width, vehicle.height), REFERENCE_SIZE[vehicle.name]
        )
    )


# Blender object names and reference offsets behind the vehicle front.
REFERENCE_OFFSETS = {
    "Tugger": (("Tugger", 0.675), ("Cart 1", 2.0), ("Cart 2", 3.8)),
    "Unit load": (("Unit load", 0.8),),
    "Forklift": (("Forklift", 1.825),),
}

BODY_OFFSETS = {
    vehicle.name: tuple(
        (name, offset * vehicle_scale(vehicle)[0])
        for name, offset in REFERENCE_OFFSETS[vehicle.name]
    )
    for vehicle in VEHICLES
}
