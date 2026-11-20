"""Experimental merge/split operations; original path vertices are retained."""

from .path_geometry import PathPosition, Subpath, TuplePath, RectangularFootprint, GeometryPolicy, sweep_2d


def split_at_vertex(part, index):
    """Restore an existing intermediate vertex as a stopping boundary."""
    boundary = PathPosition(index)
    if not part.start < boundary < part.end:
        raise ValueError('Split requires an existing interior path vertex.')
    part.path.at(boundary)
    return (Subpath(part.path, part.start, boundary),
            Subpath(part.path, boundary, part.end))


def merge_parts(parts):
    """Join consecutive parts without removing intermediate route vertices."""
    parts = tuple(parts)
    if not parts:
        raise ValueError('No parts to merge.')
    if any(a.path != b.path or a.end != b.start for a, b in zip(parts, parts[1:])):
        raise ValueError('Parts must be consecutive on the same original path.')
    return Subpath(parts[0].path, parts[0].start, parts[-1].end)


def safe_row(row, footprints, policy):
    sweeps = [sweep_2d(part, shape, policy) for part, shape in zip(row, footprints)]
    return all(not a.intersects(b, policy.tolerance)
               for i, a in enumerate(sweeps) for b in sweeps[i+1:])


def compact_tuple(plan, footprints=None, policy=GeometryPolicy()):
    """Greedily merge stage blocks only when their full swept unions are safe.

    Removing waits changes synchronization, so per-original-stage safety alone
    is insufficient. Each candidate merged row is checked across ALL objects.
    The returned block ranges refer to original stages, making the decision
    inspectable and the original decomposition available for restoration.
    """
    shapes = tuple((footprints or {}).get(name, RectangularFootprint())
                   for name in plan.object_names)
    rows, blocks = [], []
    start = 0
    while start < len(plan.stages):
        row = plan.stages[start]
        if not safe_row(row, shapes, policy):
            raise ValueError('Input stage is unsafe.')
        end = start + 1
        while end < len(plan.stages):
            candidate = tuple(merge_parts((a,b)) for a,b in zip(row,plan.stages[end]))
            if not safe_row(candidate, shapes, policy):
                break
            row = candidate
            end += 1
        rows.append(row)
        blocks.append((start,end))
        start = end
    return TuplePath(plan.object_names,tuple(rows)), tuple(blocks)
