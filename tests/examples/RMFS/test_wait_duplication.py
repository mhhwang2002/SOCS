"""Geometry identities, independent reachability oracle and existing Frogy cases."""

from dataclasses import replace
from itertools import combinations, product

import pytest

from examples.RMFS.path_geometry import (
    AABB, GeometryPolicy, InterferenceSet, Path, PathPosition, ProgressInterval,
    RectangularFootprint, Subpath, Sweep, interference_2d, sweep_2d,
)
from examples.RMFS.wait_duplication import WaitDuplicationSolver


def test_nd_paths_and_position_normalization():
    path = Path("3d", ((0, 0, 0), (2, 4, 6)))
    assert path.at(PathPosition(0, .5)) == (1., 2., 3.)
    assert PathPosition(0, 1) == PathPosition(1)
    assert Sweep(3, (AABB((0, 0, 0), (1, 1, 1)),)).dimension == 3
    with pytest.raises(ValueError, match="2D"):
        sweep_2d(path.point(0))
    with pytest.raises(ValueError):
        Path("bad", ((0, 0), (1,)))


def test_exact_l_sweep_preserves_turn_instead_of_diagonal_or_bounding_box():
    path = Path("L", ((0, 0), (4, 0), (4, 4)))
    sweep = sweep_2d(Subpath(path, PathPosition(0), PathPosition(2)))
    assert len(sweep.regions) == 2
    hole = Sweep(2, (AABB((1, 2), (2, 3)),))
    assert not sweep.intersects(hole)
    assert sweep.intersects(Sweep(2, (AABB((4, 2), (4.1, 2.1)),)))
    with pytest.raises(ValueError, match="axis-aligned"):
        sweep_2d(Subpath(Path("diagonal", ((0, 0), (1, 1))),
                         PathPosition(0), PathPosition(1)))


def test_contact_tolerance_and_margin_are_distinct():
    a = Path("a", ((0, 0),))
    b = Path("b", ((1.5, 0),))
    assert sweep_2d(a.point(0)).intersects(sweep_2d(b.point(0)))
    b = Path("b", ((1.6, 0),))
    x, y = sweep_2d(a.point(0)), sweep_2d(b.point(0))
    assert not x.intersects(y)
    assert x.intersects(y, tolerance=.11)
    assert sweep_2d(a.point(0), policy=GeometryPolicy(safety_margin=.06)).intersects(
        sweep_2d(b.point(0), policy=GeometryPolicy(safety_margin=.06)))
    with pytest.raises(ValueError):
        GeometryPolicy(tolerance=-1)


def test_interference_boundaries_are_analytic_not_samples():
    a = Path("a", ((0, 0), (10, 0)))
    b = Path("b", ((5, -5), (5, 5)))
    shape = RectangularFootprint(1, 1)
    data = interference_2d(a, shape, {"b": (b, shape)}, GeometryPolicy(tolerance=0))
    assert data.intervals == (ProgressInterval(PathPosition(0, .4), PathPosition(0, .6)),)
    assert data.extended_boundary_points() == (PathPosition(0), PathPosition(1))
    assert not data.contains(PathPosition(0))


def test_disconnected_interference_and_single_point_paths():
    a = Path("a", ((0, 0), (10, 0)))
    b = Path("b", ((2, -2), (2, 2), (8, 2), (8, -2)))
    shape = RectangularFootprint(.2, .2)
    data = interference_2d(a, shape, {"b": (b, shape)}, GeometryPolicy(tolerance=0))
    assert len(data.intervals) == 2
    assert data.intervals[0].start.ratio == pytest.approx(.18)
    assert data.intervals[1].end.ratio == pytest.approx(.82)
    assert not data.contains(PathPosition(0, .5))
    single = Path("s", ((2, 0),))
    hit = interference_2d(single, shape, {"b": (b, shape)})
    assert hit.contains(PathPosition(0))
    assert hit.extended_boundary_points() is None


def test_interference_set_merges_touching_intervals():
    p = Path("p", ((0, 0), (4, 0), (8, 0)))
    data = InterferenceSet(p, ("other",), (
        ProgressInterval(PathPosition(1), PathPosition(1, .5)),
        ProgressInterval(PathPosition(0, .5), PathPosition(1))))
    assert data.intervals == (ProgressInterval(PathPosition(0, .5), PathPosition(1, .5)),)


def reference_feasible(paths, widths=None):
    """Independent dense DAG oracle over vertex progress (implicit point steps).

    No production sweep, intersection, constituent or successor methods used.
    This smaller graph is equivalent because original point steps can always
    be inserted while all objects hold their endpoint footprints.
    """
    arrays = [p.vertices for p in paths.values()]
    widths = widths or [(1.5, 1.5)] * len(arrays)
    reachable = set()
    for state in product(*(range(len(p)) for p in arrays)):
        for advancing in product((0, 1), repeat=len(arrays)):
            if not any(advancing):
                continue
            previous = tuple(k - d for k, d in zip(state, advancing))
            if min(previous) < 0 or previous not in reachable:
                continue
            boxes = []
            for points, k, old, shape in zip(arrays, state, previous, widths):
                a, b = points[old], points[k]
                boxes.append((min(a[0], b[0]) - shape[0]/2,
                              min(a[1], b[1]) - shape[1]/2,
                              max(a[0], b[0]) + shape[0]/2,
                              max(a[1], b[1]) + shape[1]/2))
            safe = all(a[2] < b[0] - 1e-9 or b[2] < a[0] - 1e-9 or
                       a[3] < b[1] - 1e-9 or b[3] < a[1] - 1e-9
                       for a, b in combinations(boxes, 2))
            if safe:
                reachable.add(state)
                break
        if not any(state):
            # Initial bodies must be disjoint, including stationary-only paths.
            safe = all(abs(a[0][0] - b[0][0]) > (wa[0]+wb[0])/2 + 1e-9 or
                       abs(a[0][1] - b[0][1]) > (wa[1]+wb[1])/2 + 1e-9
                       for (a, wa), (b, wb) in combinations(zip(arrays, widths), 2))
            if safe:
                reachable.add(state)
    return tuple(len(p)-1 for p in arrays) in reachable


def test_all_directed_edge_pairs_on_small_grid_match_independent_oracle():
    vertices = tuple(product((0., 2.), repeat=2))
    edges = [(a, b) for a in vertices for b in vertices
             if a != b and (a[0] == b[0] or a[1] == b[1])]
    for edge_a, edge_b in product(edges, repeat=2):
        paths = {"a": Path("a", edge_a), "b": Path("b", edge_b)}
        solver = WaitDuplicationSolver(paths)
        result = solver.solve()
        assert (result.status == "feasible") == reference_feasible(paths)
        if result.tuple_path:
            assert solver.validate(result)


@pytest.mark.parametrize("example", [
    "02_01_RobotPath_Crossing_NoControl", "02_02_RobotPath_LeftEdge_NoControl",
    "02_03_RobotPath_SameDirection_Collision", "02_04_RobotPath_SameDirection_Clear",
    "02_05_RobotPath_OppositeDirection_Collision", "02_06_RobotPath_OppositeDirection_Clear",
    "02_07_RobotPath_OppositeDirection_EntryWait", "02_08_RobotPath_DisjointSweeps",
    "02_09_RobotPath_FirstMoverMatters", "02_10_RobotPath_AlternatingProgress",
    "02_11_RobotPath_SharedCorner", "03_01_RobotPath_ThreeWay_Collision",
    "03_02_RobotPath_ThreeFollowing_Collision", "03_03_RobotPath_MixedDirection_Collision",
    "03_04_RobotPath_SweepInterference", "03_05_RobotPath_LocalSafe_Deadlock",
    "03_06_RobotPath_ThreeWay_CyclicWait",
])
def test_existing_examples_against_independent_oracle(example):
    from examples.RMFS.Tests.run_wait_duplication import example_paths
    paths = example_paths(example)
    solver = WaitDuplicationSolver(paths)
    result = solver.solve()
    assert (result.status == "feasible") == reference_feasible(paths)
    if result.tuple_path:
        assert solver.validate(result)


def test_witness_validator_rejects_changed_path():
    paths = {"a": Path("a", ((0, 0), (2, 0))),
             "b": Path("b", ((0, 4), (2, 4)))}
    solver = WaitDuplicationSolver(paths)
    result = solver.solve()
    rows = list(result.tuple_path.stages)
    rows[0] = (paths["a"].point(1), rows[0][1])
    bad = replace(result, tuple_path=replace(result.tuple_path, stages=tuple(rows)))
    with pytest.raises(ValueError):
        solver.validate(bad)


def test_many_objects_with_safe_endpoints_complete():
    paths = {str(i): Path(str(i), ((-4, 4*i), (12, 4*i))) for i in range(5)}
    solver = WaitDuplicationSolver(paths)
    result = solver.solve()
    assert solver.validate(result)


def test_sufficient_condition_with_three_shared_routes():
    paths = {str(i): Path(str(i), ((-4, y), (0, y), (0, 0),
                                 (8, 0), (8, y), (12, y)))
             for i, y in enumerate((-4, 4, 8))}
    for name, path in paths.items():
        data = interference_2d(path, RectangularFootprint(),
                               {k: (v, RectangularFootprint())
                                for k, v in paths.items() if k != name})
        assert data.intervals
        assert not data.contains(PathPosition(0))
        assert not data.contains(PathPosition(len(path.vertices) - 1))
    solver = WaitDuplicationSolver(paths)
    assert solver.validate(solver.solve())


def test_stationary_waiting_footprint_blocks_passage():
    paths = {"a": Path("a", ((0, 0), (4, 0))),
             "b": Path("b", ((2, 0),))}
    result = WaitDuplicationSolver(paths).solve()
    assert result.status == "infeasible"
    assert result.tuple_path is None


def test_finished_object_still_occupies_goal():
    paths = {"a": Path("a", ((0, 0), (2, 0))),
             "b": Path("b", ((4, 0), (2, 0)))}
    assert WaitDuplicationSolver(paths).solve().status == "infeasible"


def test_heterogeneous_footprints_match_oracle():
    paths = {"a": Path("a", ((0, 0), (4, 0))),
             "b": Path("b", ((4, 2), (0, 2)))}
    footprints = {"a": RectangularFootprint(1, 1),
                  "b": RectangularFootprint(1, 3)}
    result = WaitDuplicationSolver(paths, footprints).solve()
    assert (result.status == "feasible") == reference_feasible(paths, [(1, 1), (1, 3)])
