"""Bidirectional grids; existing examples keep the 4 by 4 default."""

from math import isfinite


def make_grid(points_per_axis=4, spacing_m=2.0):
    """Return independent grid data without changing the module defaults."""
    if not isinstance(points_per_axis, int) or points_per_axis < 2:
        raise ValueError("A grid needs at least two points per axis.")
    if not isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError("Grid spacing must be positive and finite.")
    points = {(x, y): (x * spacing_m, y * spacing_m)
              for x in range(points_per_axis) for y in range(points_per_axis)}
    segments = tuple((point, neighbor) for point in points
                     for neighbor in ((point[0] + 1, point[1]), (point[0], point[1] + 1))
                     if neighbor in points)
    arcs = tuple(arc for a, b in segments for arc in ((a, b), (b, a)))
    adjacency = {p: [] for p in points}
    for a, b in arcs:
        adjacency[a].append(b)
    return dict(points_per_axis=points_per_axis, spacing_m=spacing_m,
                control_points=points, segments=segments, arcs=arcs,
                adjacency={p: tuple(v) for p, v in adjacency.items()},
                extent=(points_per_axis - 1) * spacing_m)

SPACING_M = 2.0
POINTS_PER_AXIS = 4

# Control-point IDs are integer indices. Coordinates are in metres.
CONTROL_POINTS = {
    (ix, iy): (ix * SPACING_M, iy * SPACING_M)
    for ix in range(POINTS_PER_AXIS)
    for iy in range(POINTS_PER_AXIS)
}

# Store each physical segment once. Each segment supplies two directed arcs.
SEGMENTS = tuple(
    (point, neighbor)
    for point in CONTROL_POINTS
    for neighbor in ((point[0] + 1, point[1]), (point[0], point[1] + 1))
    if neighbor in CONTROL_POINTS
)
ARCS = tuple(arc for start, end in SEGMENTS for arc in ((start, end), (end, start)))
ADJACENCY = {point: [] for point in CONTROL_POINTS}
for start, end in ARCS:
    ADJACENCY[start].append(end)
ADJACENCY = {point: tuple(neighbors) for point, neighbors in ADJACENCY.items()}


if __name__ == "__main__":
    print(f"Control points: {len(CONTROL_POINTS)} ({POINTS_PER_AXIS} x {POINTS_PER_AXIS})")
    print(f"Segments: {len(SEGMENTS)}, directed arcs: {len(ARCS)}")
    last = POINTS_PER_AXIS - 1
    print(f"Spacing: {SPACING_M} m, extent: {CONTROL_POINTS[last, last]} m")
