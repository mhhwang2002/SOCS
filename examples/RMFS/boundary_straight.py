"""All four sides, no corners; straight routes to the opposite last interior point."""

from collections import defaultdict

from .GridPath import make_grid


def make_scenario(size):
    if size < 4:
        raise ValueError("Boundary-to-interior experiments require size >= 4.")
    grid = make_grid(size)
    last = size - 1
    candidates_all = []
    for k in range(1, last):
        candidates = (
            (tuple((x, k) for x in range(0, last)), 0),
            (tuple((x, k) for x in range(last, 0, -1)), 180),
            (tuple((k, y) for y in range(0, last)), 90),
            (tuple((k, y) for y in range(last, 0, -1)), 270),
        )
        for route, heading in candidates:
            candidates_all.append((route, heading))
    # At the four corners of the interior ring, horizontal and vertical
    # routes share a goal. Keep one object, preferring the horizontal route.
    candidates_all.sort(key=lambda item: item[1] not in (0, 180))
    routes, headings, used_goals = {}, {}, set()
    for route, heading in candidates_all:
        if route[-1] in used_goals:
            continue
        name = f"R{len(routes)+1}"
        routes[name], headings[name] = route, heading
        used_goals.add(route[-1])
    goals = defaultdict(list)
    for name, route in routes.items():
        goals[route[-1]].append(name)
    shared_goals = {point: tuple(names) for point, names in goals.items() if len(names) > 1}
    return grid, routes, headings, shared_goals


def run_example(size):
    from importlib import import_module
    from .controlled import create_controlled_model

    grid, routes, headings, shared = make_scenario(size)
    print(f"{size}x{size} vertices; robots={len(routes)}; corners excluded; "
          f"unique goals={len(set(route[-1] for route in routes.values()))}")
    print(f"Shared final points: {shared}")
    print("One robot per interior-corner goal; horizontal routes retained at ties.")

    def factory(**options):
        return create_controlled_model(routes, name=f"BoundaryStraight_{size}x{size}",
                                       grid=grid, allow_no_plan=True, **options)

    import_module("examples.RMFS.Tests.01_01_RobotPath_Straight").main(
        name=f"BoundaryStraight_{size}x{size}", no_rack=True, grid=grid,
        model_factory=factory, robot_paths=routes, initial_headings=headings,
        collision_check=True, stop_on_collision=True)
