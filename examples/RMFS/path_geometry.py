"""Immutable path data and swept regions; no DEVS or viewer dependencies.

Vertices are n-dimensional floating-point vectors. The supplied sweep backend
handles 2D axis-aligned translation of a fixed rectangular footprint. Other
backends can provide Region implementations without changing Sweep or TuplePath.
"""

from functools import cached_property, lru_cache
from dataclasses import dataclass
from math import isfinite
from typing import Protocol


Vector = tuple[float, ...]


def _vector(values):
    result = tuple(float(value) for value in values)
    if not result or not all(isfinite(value) for value in result):
        raise ValueError("A vector must be nonempty and finite.")
    return result


@dataclass(frozen=True)
class GeometryPolicy:
    """Contact is interference. Tolerance and physical margin are distinct.

    tolerance conservatively expands pairwise gap comparisons (metres).
    safety_margin inflates each object's footprint on every side (metres).
    """

    tolerance: float = 1e-9
    safety_margin: float = 0.0

    def __post_init__(self):
        if any(not isfinite(v) or v < 0 for v in
               (self.tolerance, self.safety_margin)):
            raise ValueError("Tolerance and safety margin must be finite and nonnegative.")


class Region(Protocol):
    @property
    def dimension(self) -> int: ...

    def intersects(self, other: "Region", tolerance: float = 0.0) -> bool: ...


@dataclass(frozen=True)
class AABB:
    lower: Vector
    upper: Vector

    def __post_init__(self):
        object.__setattr__(self, "lower", _vector(self.lower))
        object.__setattr__(self, "upper", _vector(self.upper))
        if len(self.lower) != len(self.upper) or any(
                a > b for a, b in zip(self.lower, self.upper)):
            raise ValueError("Invalid box bounds.")

    @property
    def dimension(self):
        return len(self.lower)

    def intersects(self, other, tolerance=0.0):
        if not isinstance(other, AABB):
            raise TypeError("AABB intersection requires another AABB backend region.")
        if self.dimension != other.dimension:
            raise ValueError("Region dimensions differ.")
        if not isfinite(tolerance) or tolerance < 0:
            raise ValueError("Invalid tolerance.")
        return all(a <= d + tolerance and c <= b + tolerance
                   for a, b, c, d in zip(self.lower, self.upper,
                                         other.lower, other.upper))


def include_path_points(coordinates, points, tolerance=1e-9):
    """Expand a movement command with supplied points on its existing segments.

    This changes neither route geometry nor stop locations. The controller uses
    it so long/macro commands retain every sensor reporting location. No graph
    knowledge is required in the physical robot or sensor.
    """
    coordinates = tuple(tuple(p) for p in coordinates)
    points = tuple({tuple(p) for p in points})
    if len(coordinates) < 2:
        return coordinates
    result = [coordinates[0]]
    for a,b in zip(coordinates,coordinates[1:]):
        delta = tuple(y-x for x,y in zip(a,b))
        length2 = sum(d*d for d in delta)
        if not length2:
            continue
        interior = []
        for point in points:
            if len(point) != len(a):
                continue
            ratio = sum((p-x)*d for p,x,d in zip(point,a,delta))/length2
            if tolerance < ratio < 1-tolerance and all(
                    abs(p-(x+ratio*d)) <= tolerance for p,x,d in zip(point,a,delta)):
                interior.append((ratio,point))
        result.extend(point for _,point in sorted(interior))
        result.append(b)
    return tuple(result)


@dataclass(frozen=True, order=True)
class PathPosition:
    """Canonical progress: vertex k is (k, 0); interior is (k, ratio)."""

    segment: int
    ratio: float = 0.0

    def __post_init__(self):
        if not isinstance(self.segment, int) or self.segment < 0:
            raise ValueError("Segment index must be a nonnegative integer.")
        if not isfinite(self.ratio) or not 0 <= self.ratio <= 1:
            raise ValueError("Ratio must be in [0, 1].")
        if self.ratio == 1:
            object.__setattr__(self, "segment", self.segment + 1)
            object.__setattr__(self, "ratio", 0.0)


@dataclass(frozen=True)
class Path:
    name: str
    vertices: tuple[Vector, ...]

    def __post_init__(self):
        vertices = tuple(_vector(p) for p in self.vertices)
        if not vertices or any(len(p) != len(vertices[0]) for p in vertices):
            raise ValueError("A path needs vertices of one common dimension.")
        object.__setattr__(self, "vertices", vertices)

    @property
    def dimension(self):
        return len(self.vertices[0])

    def at(self, position):
        k, ratio = position.segment, position.ratio
        if k >= len(self.vertices) or (ratio and k == len(self.vertices) - 1):
            raise ValueError("Position is outside this path.")
        if not ratio:
            return self.vertices[k]
        return tuple(a + ratio * (b - a)
                     for a, b in zip(self.vertices[k], self.vertices[k + 1]))

    def point(self, index):
        return Subpath(self, PathPosition(index), PathPosition(index))

    def constituents(self):
        """Original word: p0, e01, p1, e12, ..., pm. No merging."""
        result = []
        for k in range(len(self.vertices)):
            if k:
                result.append(Subpath(self, PathPosition(k - 1), PathPosition(k)))
            result.append(self.point(k))
        return tuple(result)


@dataclass(frozen=True)
class Subpath:
    path: Path
    start: PathPosition
    end: PathPosition

    def __post_init__(self):
        self.path.at(self.start)
        self.path.at(self.end)
        if self.start > self.end:
            raise ValueError("A subpath must preserve original progress order.")

    @property
    def is_point(self):
        return self.start == self.end

    def coordinates(self):
        interior = tuple(self.path.vertices[k] for k in range(
            self.start.segment + 1, self.end.segment + 1)
            if PathPosition(k) < self.end)
        return (self.path.at(self.start), *interior, self.path.at(self.end))


@dataclass(frozen=True)
class Sweep:
    dimension: int
    regions: tuple[Region, ...]

    def __post_init__(self):
        object.__setattr__(self, "regions", tuple(self.regions))
        if self.dimension < 1 or any(r.dimension != self.dimension for r in self.regions):
            raise ValueError("Sweep region dimensions must agree.")

    @cached_property
    def rectangles_2d(self):
        if self.dimension != 2 or not self.regions or any(type(r) is not AABB for r in self.regions):
            return None
        boxes = tuple((*r.lower, *r.upper) for r in self.regions)
        return (min(b[0] for b in boxes), min(b[1] for b in boxes),
                max(b[2] for b in boxes), max(b[3] for b in boxes)), boxes

    def intersects(self, other, tolerance=0.0):
        if self.dimension != other.dimension:
            raise ValueError("Sweep dimensions differ.")
        first, second = self.rectangles_2d, other.rectangles_2d
        if first is not None and second is not None and isfinite(tolerance) and tolerance >= 0:
            (ax, ay, bx, by), left = first
            (cx, cy, dx, dy), right = second
            if ax > dx + tolerance or cx > bx + tolerance or ay > dy + tolerance or cy > by + tolerance:
                return False
            for ax, ay, bx, by in left:
                for cx, cy, dx, dy in right:
                    if (ax <= dx + tolerance and cx <= bx + tolerance
                            and ay <= dy + tolerance and cy <= by + tolerance):
                        return True
            return False
        return any(a.intersects(b, tolerance) for a in self.regions for b in other.regions)


@dataclass(frozen=True)
class RectangularFootprint:
    width: float = 1.5
    depth: float = 1.5

    def __post_init__(self):
        if any(not isfinite(v) or v <= 0 for v in (self.width, self.depth)):
            raise ValueError("Footprint dimensions must be positive and finite.")


@lru_cache(maxsize=32768)
def sweep_2d(subpath, footprint=RectangularFootprint(), policy=GeometryPolicy()):
    """Exact union of segment boxes, with optional physical inflation."""
    if subpath.path.dimension != 2:
        raise ValueError("This sweep backend only implements 2D geometry.")
    h = (footprint.width / 2 + policy.safety_margin,
         footprint.depth / 2 + policy.safety_margin)
    points = subpath.coordinates()
    boxes = []
    for a, b in zip(points, points[1:]):
        if a[0] != b[0] and a[1] != b[1]:
            raise ValueError("This backend requires axis-aligned segments.")
        boxes.append(AABB(tuple(min(x, y) - half for x, y, half in zip(a, b, h)),
                          tuple(max(x, y) + half for x, y, half in zip(a, b, h))))
    return Sweep(2, tuple(boxes))


@dataclass(frozen=True)
class ProgressInterval:
    start: PathPosition
    end: PathPosition

    def __post_init__(self):
        if self.start > self.end:
            raise ValueError("Reversed progress interval.")


@dataclass(frozen=True)
class InterferenceSet:
    path: Path
    opponents: tuple[str, ...]
    intervals: tuple[ProgressInterval, ...]

    def __post_init__(self):
        merged = []
        for interval in sorted(self.intervals, key=lambda x: x.start):
            self.path.at(interval.start)
            self.path.at(interval.end)
            if merged and interval.start <= merged[-1].end:
                merged[-1] = ProgressInterval(merged[-1].start,
                                              max(merged[-1].end, interval.end))
            else:
                merged.append(interval)
        object.__setattr__(self, "opponents", tuple(self.opponents))
        object.__setattr__(self, "intervals", tuple(merged))

    @property
    def initial_boundary(self):
        """Initial non-interfering subpath ends just before this boundary."""
        return self.intervals[0].start if self.intervals else None

    @property
    def terminal_boundary(self):
        return self.intervals[-1].end if self.intervals else None

    def contains(self, position):
        return any(i.start <= position <= i.end for i in self.intervals)

    def extended_boundary_points(self):
        """Nearest safe selected vertices outside the first/last interference.

        None means the two-sided extension is undefined, not infeasibility.
        """
        if not self.intervals:
            return None
        before = [PathPosition(k) for k in range(len(self.path.vertices))
                  if PathPosition(k) < self.initial_boundary]
        after = [PathPosition(k) for k in range(len(self.path.vertices))
                 if PathPosition(k) > self.terminal_boundary]
        return (before[-1], after[0]) if before and after else None


def interference_2d(path, footprint, opponents, policy=GeometryPolicy()):
    """Analytic clipping of each center segment against expanded opponent boxes.

    opponents maps name -> (Path, RectangularFootprint). No time sampling.
    """
    intervals = []
    h = (footprint.width / 2 + policy.safety_margin,
         footprint.depth / 2 + policy.safety_margin)
    # Validate the complete subject geometry, including stationary paths.
    sweep_2d(Subpath(path, PathPosition(0), PathPosition(len(path.vertices) - 1)),
             footprint, policy)
    for other_path, other_footprint in opponents.values():
        other = sweep_2d(Subpath(other_path, PathPosition(0),
                                PathPosition(len(other_path.vertices) - 1)),
                         other_footprint, policy)
        segments = list(enumerate(zip(path.vertices, path.vertices[1:])))
        if len(path.vertices) == 1:
            segments = [(0, (path.vertices[0], path.vertices[0]))]
        for k, (a, b) in segments:
            for box in other.regions:
                lower = tuple(v - half - policy.tolerance for v, half in zip(box.lower, h))
                upper = tuple(v + half + policy.tolerance for v, half in zip(box.upper, h))
                lo, hi = 0.0, 1.0
                for x, y, left, right in zip(a, b, lower, upper):
                    delta = y - x
                    if delta == 0:
                        if x < left or x > right:
                            lo, hi = 1.0, 0.0
                            break
                    else:
                        u, v = (left - x) / delta, (right - x) / delta
                        lo, hi = max(lo, min(u, v)), min(hi, max(u, v))
                if lo <= hi:
                    if len(path.vertices) == 1:
                        start = end = PathPosition(0)
                    else:
                        start, end = PathPosition(k, lo), PathPosition(k, hi)
                    intervals.append(ProgressInterval(start, end))
    return InterferenceSet(path, tuple(opponents), tuple(intervals))


@dataclass(frozen=True)
class TuplePath:
    """Explicit [stage][object] subpaths. No sparse wait encoding."""

    object_names: tuple[str, ...]
    stages: tuple[tuple[Subpath, ...], ...]

    def __post_init__(self):
        object.__setattr__(self, "object_names", tuple(self.object_names))
        object.__setattr__(self, "stages", tuple(tuple(row) for row in self.stages))
        if len(set(self.object_names)) != len(self.object_names) or any(
                len(row) != len(self.object_names) for row in self.stages):
            raise ValueError("A tuple path needs a fixed, unique object order.")
