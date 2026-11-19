"""Static 2 m bidirectional grid with 4 by 4 points and 3 by 3 cells."""

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
