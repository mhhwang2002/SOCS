import pytest

from examples.RMFS.GridPath import CONTROL_POINTS, make_grid


@pytest.mark.parametrize("size", (4, 10, 20))
def test_independent_grid_sizes_preserve_default(size):
    grid = make_grid(size)
    assert len(grid["control_points"]) == size * size
    assert len(grid["segments"]) == 2 * size * (size - 1)
    assert len(grid["arcs"]) == 4 * size * (size - 1)
    assert grid["control_points"][size - 1, size - 1] == ((size - 1) * 2.,) * 2
    assert all(a in grid["adjacency"][b] for a, b in grid["arcs"])
    assert len(CONTROL_POINTS) == 16


@pytest.mark.parametrize("size,spacing", ((1, 2.), (4, 0.), (4, float("inf"))))
def test_invalid_grid_sizes(size, spacing):
    with pytest.raises(ValueError):
        make_grid(size, spacing)


@pytest.mark.parametrize("size,count", ((4, 4), (10, 28), (20, 68)))
def test_boundary_routes_exclude_corners_and_have_unique_interior_goals(size, count):
    from examples.RMFS.boundary_straight import make_scenario
    grid, paths, headings, shared = make_scenario(size)
    assert len(paths) == count and not shared
    starts = [p[0] for p in paths.values()]
    goals = [p[-1] for p in paths.values()]
    assert len(set(starts)) == len(starts)
    assert len(set(goals)) == len(goals)
    for route in paths.values():
        x, y = route[0]
        assert (x in (0, size - 1)) != (y in (0, size - 1))
        assert all(0 < v < size - 1 for v in route[-1])
        assert len(set(route)) == len(route)
        assert all(abs(a[0]-b[0])+abs(a[1]-b[1]) == 1 for a, b in zip(route, route[1:]))
        assert len({p[0] for p in route}) == 1 or len({p[1] for p in route}) == 1


@pytest.mark.parametrize("size", (4, 10, 20))
def test_boundary_examples_report_head_on_impossibility_without_commands(size):
    from examples.RMFS.boundary_straight import make_scenario
    from examples.RMFS.controlled import create_controlled_model
    from socs.devs.TESEngin import TESEnginBase
    from math import isinf
    grid, routes, headings, _ = make_scenario(size)
    model, robots, sensors = create_controlled_model(
        routes, grid=grid, allow_no_plan=True, initial_headings=headings)
    engine = TESEnginBase(model)
    assert model.plan_result.status == "infeasible"
    assert "cannot pass" in model.planning_error
    assert model.plan_result.checked_transitions == 0
    assert isinf(model.tNext)
    assert all(robot.phase == "Wait" for robot in robots)
    assert not model.controller.traffic_manager.command_buffer
    assert not model.controller.roadmap.history
