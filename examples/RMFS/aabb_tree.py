"""AABB broad phase for the current frame of RMFS robot poses."""

from dataclasses import dataclass
from math import cos, radians, sin


def footprint_bounds(pose):
    (x, y), heading, carrying = pose
    if carrying:
        dx = dy = .75
    else:
        angle = radians(heading)
        c, s = abs(cos(angle)), abs(sin(angle))
        dx, dy = .675 * c + .455 * s, .675 * s + .455 * c
    # Keep boundary contacts accepted by the narrow phase.
    pad = 1e-8
    return x - dx - pad, y - dy - pad, x + dx + pad, y + dy + pad


@dataclass(frozen=True)
class Node:
    bounds: tuple
    index: int | None = None
    children: tuple = ()

    def query(self, bounds):
        x0, y0, x1, y1 = self.bounds
        a0, b0, a1, b1 = bounds
        if x1 < a0 or a1 < x0 or y1 < b0 or b1 < y0:
            return
        if self.index is not None:
            yield self.index
        for child in self.children:
            yield from child.query(bounds)


def build_tree(leaves):
    if len(leaves) == 1:
        return leaves[0]
    bounds = (
        min(leaf.bounds[0] for leaf in leaves),
        min(leaf.bounds[1] for leaf in leaves),
        max(leaf.bounds[2] for leaf in leaves),
        max(leaf.bounds[3] for leaf in leaves),
    )
    axis = int(bounds[3] - bounds[1] > bounds[2] - bounds[0])
    leaves.sort(key=lambda leaf: leaf.bounds[axis] + leaf.bounds[axis + 2])
    middle = len(leaves) // 2
    return Node(bounds, children=(build_tree(leaves[:middle]), build_tree(leaves[middle:])))


def candidate_pairs(poses):
    names = list(poses)
    if len(names) < 2:
        return
    leaves = [Node(footprint_bounds(poses[name]), index) for index, name in enumerate(names)]
    tree = build_tree(leaves.copy())
    for leaf in leaves:
        for other in tree.query(leaf.bounds):
            if other > leaf.index:
                yield names[leaf.index], names[other]
