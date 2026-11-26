from examples.RMFS.GridPath import make_grid
from examples.RMFS.RoadMap import RoadMap
from examples.RMFS.RobotPathPlanner import RobotPathPlanner
from examples.RMFS.motion_graph import compile_motion_graph
from examples.RMFS.path_geometry import Path, PathPosition


def test_route_avoids_remaining_traffic_when_detour_costs_less():
    roadmap = RoadMap({
        "R1": Path("R1", ((0, 0),)),
        "R2": Path("R2", ((2, 0), (28, 0))),
    })
    planner = RobotPathPlanner(roadmap, compile_motion_graph(make_grid(16)))
    assignment = {"goal_position": (30, 0)}
    direct = planner.plan("R1", assignment)
    assert all(y == 0 for _, y in direct)

    traffic = roadmap.remaining_subpath("R2")
    detour = planner.plan("R1", assignment, traffic_routes=(traffic,))
    assert any(y > 0 for _, y in detour)
    assert detour[0] == direct[0] and detour[-1] == direct[-1]
    assert planner.plan(
        "R1", assignment, traffic_routes=(traffic,),
        blocked_positions=[(x, 2) for x in range(0, 31, 2)],
    ) == direct

    roadmap.robots["R2"]["released_progress"] = PathPosition(1)
    assert planner.plan(
        "R1", assignment, traffic_routes=(roadmap.remaining_subpath("R2"),),
    ) == direct
