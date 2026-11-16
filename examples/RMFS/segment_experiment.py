"""Deterministic loaded-rack tests for segment entry control."""

from dataclasses import dataclass
from math import ceil, dist

from .fleet import LOADED_RADIUS, footprints_overlap
from .motion import LOADED, Straight


@dataclass(frozen=True)
class Move:
    start: tuple[float, float]
    end: tuple[float, float]
    segment: str

    @property
    def profile(self):
        return Straight(dist(self.start, self.end), LOADED)

    def position(self, elapsed, profile):
        fraction = profile.distance_at(elapsed) / profile.length
        return tuple(a + (b - a) * fraction for a, b in zip(self.start, self.end))


def first_contacts(first, second, *, delay=None, step=.01):
    profile_a, profile_b = first.profile, second.profile
    if delay is None:
        delay = profile_a.duration if first.segment == second.segment else 0.0
    end = max(profile_a.duration, delay + profile_b.duration)
    circle = square = None
    for index in range(ceil(end / step) + 1):
        time = min(index * step, end)
        a = first.position(time, profile_a)
        b = second.position(time - delay, profile_b)
        if circle is None and dist(a, b) <= 2 * LOADED_RADIUS:
            circle = time
        # The rack stays aligned with the floor axes while its robot turns.
        if square is None and footprints_overlap((a, 0, True), (b, 0, True)):
            square = time
        if circle is not None and square is not None:
            break
    return circle, square


def scenarios():
    west_east = Move((-4, 0), (4, 0), "horizontal")
    south_north = Move((0, -4), (0, 4), "vertical")
    return [
        ("Crossing, independent segments", west_east, south_north, None),
        ("Crossing, shared reservation", west_east,
         Move(south_north.start, south_north.end, "horizontal"), None),
        ("Head-on, opposite arc IDs", west_east, Move((4, 0), (-4, 0), "reverse"), None),
        ("Head-on, shared segment; wait at exit", west_east,
         Move((4, 0), (-4, 0), "horizontal"), None),
        ("Workstation return, wait on approach", Move((0, 0), (0, -4), "approach"),
         Move((0, -3), (0, 0), "approach"), None),
        ("Workstation return, wait in side bay", Move((0, 0), (0, -4), "approach"),
         Move((2.5, -3), (0, 0), "approach"), None),
        ("Parallel lanes, 1.65 m apart", west_east,
         Move((-4, 1.65), (4, 1.65), "parallel"), None),
    ]


def main():
    for label, first, second, delay in scenarios():
        circle, square = first_contacts(first, second, delay=delay)
        times = ["none" if time is None else f"{time:.2f} s" for time in (circle, square)]
        print(f"{label}: circle={times[0]}, rack={times[1]}")


if __name__ == "__main__":
    main()
